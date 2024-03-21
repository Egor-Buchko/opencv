import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
from math import pi
import os


rospy.init_node('flight')
image_bgr_pub = rospy.Publisher('Img_bgr', Image, queue_size=1)
image_hsv_pub = rospy.Publisher('Img_hsv', Image, queue_size=1)
angle = 120
h = 2
width = 2 * math.tan(angle/2) * h
l_pix = width/320
print(math.tan(pi/3), l_pix)
color = ((30, 189, 239),(31, 191, 241))
arr = []
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
bridge = CvBridge()

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame=False, frame_id='body', auto_arm=False):
    global arr
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    while not rospy.is_shutdown():
        frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        telem = get_telemetry(frame_id='navigate_target')
        
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        telem = get_telemetry(frame_id='aruco_90')
        h = telem.z
        width = 2 * math.tan(pi/3) * h
        l_pix = width/320
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, color[0], color[1])
        image_hsv_pub.publish(bridge.cv2_to_imgmsg(mask, 'mono8'))
        contours, hierarchy = cv2.findContours(image=mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            center, radius = cv2.minEnclosingCircle(contour)
            cX, cY = (int(center[0]), int(center[1]))
            cv2.circle(frame, (cX, cY), int(radius), (0, 255, 0), 2)
            print(cX, cY, telem.y)
            if abs(cY - 120)<5:
                x_rab, y_rab = (cX - 160)*l_pix+telem.y, telem.x
                for elem in arr:
                    if abs(elem[0] - x_rab) < 0.2 and abs(elem[1] - y_rab) < 0.2:
                        break
                else:
                    arr.append(((cX - 160)*l_pix+telem.y, telem.x))
        cv2.drawContours(image=frame, contours=contours, contourIdx=-1, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
        image_bgr_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))

        rospy.sleep(0.2)

rame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
navigate_wait(z=2, frame_id='body', auto_arm=True)
navigate_wait(x=0, y=0, z=1.5, frame_id='aruco_97', speed=0.5)
print(arr)
land()