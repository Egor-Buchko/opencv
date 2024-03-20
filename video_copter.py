from clover import srv
from std_srvs.srv import Trigger
import rospy
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
land = rospy.ServiceProxy('land', Trigger)

# Инициализация объекта моста и видеозаписи
bridge = CvBridge()
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('flight_video1.mp4', fourcc, 20.0, (320, 240))

def record_video(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    out.write(cv_image)

# Подписка на топик камеры для записи видео
rospy.Subscriber('main_camera/image_raw', Image, record_video)

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def wait_yaw():
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if abs(telem.yaw) < math.radians(1):
            return
        rospy.sleep(0.2)


try:
    navigate_wait(z=2, frame_id='body', auto_arm=True)
    navigate_wait(x=0, y=0, z=2, frame_id='aruco_4', speed=0.5)
    set_yaw(yaw=math.radians(180), frame_id='body')
    wait_yaw()
    navigate_wait(x=0, y=0, z=2, frame_id='aruco_0', speed=0.5)
    land_wait()
except rospy.ROSInterruptException:
    pass
finally:
    # Освобождение ресурсов
    out.release()
    cv2.destroyAllWindows()