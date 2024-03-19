import rospy
import cv2
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('picture_cap')
bridge = CvBridge()
deley = input("enter the deley: ")
for i in range(deley):
    rospy.sleep(1)
    print(i + 1)
rospy.sleep(float(deley))
img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
cv2.imwrite("image{0}.png".format(random.randint(0,1000)), img)
