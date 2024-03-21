import numpy as np
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import time
from sensor_msgs.msg import Range
import requests

bridge = CvBridge()

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
land = rospy.ServiceProxy('land', Trigger)

# Ожидаем прилёт в заданную точку
def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.1):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
# Функция посадки
def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

# список высот

# Списки для хранения информации о возгорании и пострадавших
fire_arr = []
hurt_arr = []
# hsv-диапазоны для распознования возгорания, пострадавшего и зоны посадки
yellow_min, yellow_max = (25, 78, 129), (42, 172, 177)
red_min, red_max = (175, 109, 175), (183, 157, 226)
green_min, green_max = (59, 88, 136), (82, 146, 178)
blue_min, blue_max = (86, 127, 123), (138, 249, 207)
# image topic для отладки
fire_detect = rospy.Publisher("/fire_detect", Image, queue_size=1)
hurt_detect = rospy.Publisher("/hurt_detect", Image, queue_size=1)
color_debug = rospy.Publisher("/color_debug", Image, queue_size=1)
land_debug = rospy.Publisher("/land_debug", Image, queue_size=1)
# координаты центра изображения
img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
x_center, y_center = img.shape[1] / 2, img.shape[0] / 2 # Центр изображения

# Всё для PID-регулятора
oldtime = 0
realtime = 0
# X PID
y_preverr = 1000000000
PX = 0
IX = 0
DX = 0
# Y PID
x_preverr = 1000000000
PY = 0
IY = 0
DY = 0
# Yaw PID
Pyaw = 0
Iyaw = 0
Dyaw = 0
preverryaw = 1000000000
# Z PID
PZ = 0
IZ = 0
DZ = 0
z_preverr = 1000000000
k = 0
# Коэффициенты PID по Y
Pky = 0.0016
Iky = 0.01
Dky = 0.00016
# Коэффициенты PID по X
Pkx = 0.0016
Ikx = 0.01
Dkx = 0.00016
# Коэффициенты PID по Yaw
Pkyaw = 0.0032
Ikyaw = 0.01
Dkyaw = 0.00016
# Коэффициенты PID по Z
Pkz = 0.08
Ikz = 0.02
Dkz = 0.00016

# Для подсчета корректировки
def PIDresult(Pk, Ik, Dk, P, I, D, err, preverr, tim): # Принимает: 3 коэффициента усиления PID, 3 значения PID-составляющих, ошибку, предыдущую ошибку, время между подсчетами
    P = err # Считаем P составляющую
    if tim < 2:
        I = I + err * tim # Считаем I составляющую
    if preverr != 1000000000 and tim < 2:
        D = (err-preverr) / tim # Считаем D составляющую
    out = P * Pk + I * Ik + D * Dk # Складываем составляющие, умножая их на коэффициенты
    return out
err=0
# функция для центрирования над зоной посадки mask_min, mask_max
def land_image(data):
    global oldtime, x_preverr, y_preverr, z_preverr, h_land, err
    realtime = time.time() # Текущее время
    land_cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    land_cv_image_copy = land_cv_image.copy() # Копия изображения
    land_hsv = cv2.cvtColor(land_cv_image, cv2.COLOR_BGR2HSV) # Переводим из BGR в HSV
    land_mask = cv2.inRange(land_hsv, np.array(mask_now[0]), np.array(mask_now[1])) # Маска
    land_debug.publish(bridge.cv2_to_imgmsg(land_mask, 'mono8')) # Публикуем в топик
    contours, _ = cv2.findContours(land_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # Search moments
    contours.sort(key=cv2.minAreaRect)  # Sort moments
    # Если есть посадочная площадка, летим к ней
    if len(contours) > 0:
        cnt = contours[0]
        if cv2.contourArea(cnt) > 50: 
            rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
            (x_min, y_min), (w_min, h_min), angle = rect
            height = rospy.wait_for_message('rangefinder/range', Range).range # Текущая высота
            cv2.drawContours(land_cv_image_copy, contours, 0, (180, 105, 255), 3)  # Обводим посадочную площадку
            color_debug.publish(bridge.cv2_to_imgmsg(land_cv_image_copy, 'bgr8')) # Публикуем в топик
            y_err = (x_center - x_min) # Ошибка по y
            x_err = (y_center - y_min) # Ошибка по x
            z_err = h_land - height # Ошибка по z
            Xout = PIDresult(Pkx, Ikx, Dkx, PX, IX, DX, x_err, x_preverr, realtime - oldtime) # Обработанная ошибка по x
            Yout = PIDresult(Pky, Iky, Dky, PY, IY, DY, y_err, y_preverr, realtime - oldtime) # Обработанная ошибка по y
            Zout = PIDresult(Pkz, Ikz, Dkz, PZ, IZ, DZ, z_err, z_preverr, realtime - oldtime) # Обработанная ошибка по z
            x_preverr = x_err # Запоминаем ошибку по x
            y_preverr = y_err # Запоминаем ошибку по y
            z_preverr = z_err # Запоминаем ошибку по z
            err=(x_err**2+y_err**2+z_err**2)**0.5
            #print(err)
            set_velocity(vx=Xout, vy=Yout, vz=Zout, frame_id='body', yaw=float('nan'))
            # Запоминаем время для PID-регулятора
            oldtime = time.time()

def regulation():
    global h_land
    h_land = 0.8
    land_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, land_image, queue_size=1) # Центрирование относительно посадочной площадки
    rospy.sleep(5)
    h_land = 0.5
    while err>12:
        print(err)
        pass
    land_sub.unregister()
    set_position(frame_id='aruco_map')
    set_velocity(vx=0, vy=0.0, vz=0, frame_id='body')
    print("ok")

print("fly")
navigate_wait(z=1, frame_id='body', auto_arm=True)
telem = get_telemetry(frame_id='aruco_map')
xstart, ystart = telem.x, telem.y # Запоминаем координаты зоны взлёта/посадки
navigate_wait(z=2, frame_id='aruco_11', auto_arm=True)
mask_now = (red_min, red_max)
regulation()
navigate_wait(z=2, frame_id='aruco_11', auto_arm=True)
mask_now = (yellow_min, yellow_max)
regulation()
navigate_wait(z=2, frame_id='aruco_11', auto_arm=True)
mask_now = (green_min, green_max)
regulation()
navigate_wait(z=2, frame_id='aruco_11', auto_arm=True)
mask_now = (blue_min, blue_max)
regulation()
navigate_wait(x=xstart ,y=ystart ,z=2, frame_id='aruco_map', auto_arm=True)
land_wait()