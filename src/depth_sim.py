import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String

dbridge = CvBridge()
dbridge1 = CvBridge()

rospy.init_node('sim_depth', anonymous=True)
scale_factor = 1/3.78
curr_pose = PoseStamped() 
state = String()
state.data = "Intialiazing" 
color_img_t = Image()

img_h = 480
img_w = 640
h_fov = 1.9
v_fov = 1.9 * img_h/img_w
d_hoop = 0.8 ## distance from hoop we will be doing y scan at
px = h_fov*d_hoop/img_w
py = v_fov*d_hoop/img_h

def disp_img(fname, frame):
    cv2.namedWindow(fname,cv2.WINDOW_NORMAL)
    cv2.imshow(fname,frame)

def show_image(frame):
    global curr_pose, img_h, img_w, px, py, color_img_t
    disp_img("org",frame*15)
    image = dbridge1.imgmsg_to_cv2(color_img_t,desired_encoding='bgr8')
    print(np.unique(frame,return_counts = True))
    frame1 = np.where(frame<5,frame,0)
    depth_val = 0

    depth_colormap = cv2.applyColorMap(frame1, cv2.COLORMAP_JET)
    # frame = frame*scale_factor # for representation
    color_img = depth_colormap.copy()
    depth_colormap = cv2.GaussianBlur(depth_colormap.copy(), (5, 5), 0)
    edges = cv2.Canny(depth_colormap, 13, 14)

    kernel1 = np.ones((7,7), np.uint8) 
    kernel2 = np.ones((3,3), np.uint8) 
    kernel3 = np.ones((5,5), np.uint8) 

    edges = cv2.dilate(edges, kernel1, iterations=4)
    edges = cv2.erode(edges, kernel1, iterations=3) 
    edges = cv2.dilate(edges, kernel2, iterations=1)
    edges = cv2.erode(edges, kernel2, iterations=6) 
    # edges = cv2.dilate(edges, kernel3, iterations=2)

    disp_img("image1", edges)

    cnts, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    colors = [(255,0,0),(0,255,0),(0,0,255),(255,0,0),(0,255,0),(0,0,255),(255,0,0),(0,255,0),(0,0,255),(255,0,0),(0,255,0),(0,0,255)]
    
    rel_coords_x, rel_coords_y = 0, 0

    for i, cnt in enumerate(cnts):
        mode = 0
        x_h ,y_h = 0,0

        peri = cv2.arcLength(cnt, True)
        # print(peri)
        approx = cv2.approxPolyDP(cnt, 77/1000, True)
        rect = cv2.minAreaRect(approx)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        area = cv2.contourArea(box)
        x,y,w,h = cv2.boundingRect(approx)

        print(x,y,w,h, " - ", (float(h/w))-0.55)
        print(h/w)
        # if abs(h/w - 0.55) <= 0.1 :
        #     cv2.rectangle(color_img,(x,y),(x+w,y+h),(0,255,0),2)

        val_w = 390
        val_h = 100000

        if area/( (w-1)*(h-1) ) == 1:
            ## this means a perfect rectangle
            # print("pr",x,y,w,h)
            if abs(w - val_w) < 45:
                mode = 1
            if abs(h - val_h) < 40:
                if mode == 0:
                    mode = 2
                else:
                    mode = 3
            depth_val = calc_depth(frame1,x,y,w,h)
        else:
            ## Tilted rectangles
            a1 = box[np.argmin(box[:,1]),:]
            a2 = box[np.argmin(box[:,0]),:]
            a3 = box[np.argmax(box[:,1]),:]
            a4 = box[np.argmax(box[:,0]),:]
            box1 = np.array([a1,a2,a3,a4])

            a12 = np.sqrt(np.sum(np.power(np.diff(box1[0:2],axis=0),2)))
            a41 = np.sqrt(np.sum(np.power(np.diff(box1[[0,-1],:],axis=0),2)))
            # print("tr",box1)
            # print("tr",a12,a41)
            # cv2.drawContours(color_img,[box],0,colors[i],2)
            if abs(a12 - val_w) < 45:
                mode = 1
                depth_val = calc_depth(frame1,x,y,w,h)
            if abs(a41 - val_w) < 40:
                mode = 1
                depth_val = calc_depth(frame1,x,y,w,h)

        if mode!=0:
            # print(depth_val,"--", x,y,w,h)
            rel_coords_x = x - img_w/2 +  415/2 
            rel_coords_y = y - img_h/2 + 245/2
            ## rel_x and rel_y are in meters wrt image center 

            cv2.rectangle(color_img,(x,y),(x+415,y+245),(0,255,0),2)
            cv2.rectangle(image,(x,y),(x+415,y+245),(0,255,0),2)

            # cv2.rectangle(color_img,(x,y),(x+w,y+h),(0,0,255),2)
            center_coordinates = (int(x+415/2),int(y+245/2) )
            cv2.circle(image, center_coordinates, 4, (255,0,0), 4) 
            ## could be nice if this was computed via transform b/w drone position and global frame
            y_h = curr_pose.pose.position.y + (rel_coords_x*px)*-1 # y in gazebo is to the left, in opencv x is to right
            # print(y_h)
            msg = Float64()
            msg.data = y_h
            point_pub.publish(y_h)

    cv2.putText(image,"Delta_x {}".format(rel_coords_x), (400,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 1)
    cv2.putText(image,"Delta_y {}".format(rel_coords_y), (400,130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 1)
    cv2.putText(image,state.data, (400,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 6)

    disp_img("Frame", color_img)
    disp_img("Color frame", image)

    return edges

def calc_depth(frame1, x,y,w,h):
    global img_w,img_h
    a = max(0,y-3)
    b = min(img_h,y+h+3)
    c = max(0,x-3)
    d = min(img_w,x+w+3)
    arr = frame1[a:b, c:d]
    depth_val = np.mean(arr[np.where(arr!=0)])
    return depth_val


def callback_pos(pos):
    global curr_pose
    curr_pose = pos

def image_callback(img_msg):
    # global cv_image
    cv_image = dbridge.imgmsg_to_cv2(img_msg,"8UC1")
    edges = show_image(cv_image)
    if cv2.waitKey(1) == 27:
        print("called")     

def color_image_callback(img_msg1):
    global color_img_t
    color_img_t = img_msg1 

def state_cbk(data):
    global state
    state = data

sub_image = rospy.Subscriber("/r200/camera/depth/image_raw", Image, image_callback)
sub_color_image = rospy.Subscriber("/r200/camera/color/image_raw", Image, color_image_callback)
sub_pos = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, callback_pos)
state_sub = rospy.Subscriber('grid/state',String, state_cbk)


point_pub = rospy.Publisher('/points_cluster',Float64,queue_size=10)

def myhook():
    print("shutdown time!")
    exit(0)
rospy.on_shutdown(myhook)
rospy.spin()
