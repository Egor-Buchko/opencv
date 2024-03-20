import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import os


rospy.init_node('flight')
color = ((0, 185, 230),(11, 243, 255))
cX, cY = 160, 120
image_bgr_pub = rospy.Publisher('Img_bgr', Image, queue_size=1)
image_hsv_pub = rospy.Publisher('Img_hsv', Image, queue_size=1)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

bridge = CvBridge()

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
            raise Exception(res.message)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)
navigate_wait(z=1.5, frame_id='body', auto_arm=True)
while True:
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, color[0], color[1])
    image_hsv_pub.publish(bridge.cv2_to_imgmsg(hsv, 'bgr8'))
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 0:
        contour = max(contours, key = cv2.contourArea)
        center, radius = cv2.minEnclosingCircle(contour)
        cX, cY = (int(center[0]), int(center[1]))
        cv2.circle(frame, (cX, cY), int(radius), (0, 255, 0), 2)
    image_bgr_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
    tg_alfa = (cX-160) / (cY - 140)
    print(tg_alfa * 57.3, math.atan(tg_alfa))
    key = cv2.waitKey(1)
    if key == ord("z"):
        break
land()
