# Импортируем библиотеки
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
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray,Marker
import os

rospy.init_node('flight')

bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
land = rospy.ServiceProxy('land', Trigger)
color = None

yellow = ((25, 29, 186),(39, 157, 233))
red = ((175, 94, 199),(193, 156, 245))
green = ((53, 64, 155), (85, 129, 198))
blue = ((0, 0, 0), (0, 0, 0))

image_bgr_pub = rospy.Publisher('Img_bgr', Image, queue_size=1)
image_hsv_pub = rospy.Publisher('Img_hsv', Image, queue_size=1)
fire_detect = rospy.Publisher("/fire_detect", Image, queue_size=1)
hurt_detect = rospy.Publisher("/hurt_detect", Image, queue_size=1)
color_debug = rospy.Publisher("/color_debug", Image, queue_size=1)
land_debug = rospy.Publisher("/land_debug", Image, queue_size=1)


pub_b = rospy.Publisher("/visualization_buildings", MarkerArray, queue_size=2)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    print(navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm))
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


arr_color_coord = {"red": (0, 0),
                   "green": (0, 0),
                   "blue": (0, 0),
                   "yellow": (0, 0)}
red_conf, blue_conf, green_conf, yellow_conf = True, True, True, True
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
        pass
    land_sub.unregister()
    set_position(frame_id='aruco_map')
    set_velocity(vx=0, vy=0.0, vz=0, frame_id='body')
    print("ok")

def Mark(x, y, z, lx, ly, lz, id, r, g, b, type):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = type
    marker.id = id
    # Задаем размеры
    marker.scale.x = lx + 0.01
    marker.scale.y = ly + 0.01
    marker.scale.z = lz + 0.01
    # Задаем цвет
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1.0
    # Задаем координаты центра
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    # Задаем поворот
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker

# Cоздание буфера для tf2
tf_buffer = tf2_ros.Buffer()  # tf sbuffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

def euler_from_quaternion(q):
    x, y, z, w=q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z 

def pose(tr="aruco_map"):#Позиция через tf2
    #print(rospy.Time.now())
    transform = tf_buffer.lookup_transform(tr, 'body', rospy.Time(rospy.Time.now().to_sec()-0.05))
    #print(transform)
    q=quaternion_from_euler(0, 0, 0)
    pose_transformed = tf2_geometry_msgs.do_transform_pose(PoseStamped(pose=Pose(position=Point(0, 0, 0), orientation=Quaternion(*q))), transform)
    #print(0)
    q = pose_transformed.pose.orientation
    p = pose_transformed.pose.position
    return p.x, p.y, p.z, euler_from_quaternion(q)



markerArrayB = MarkerArray()
Buildings=[]

def Building_rviz_add(x,y,z,r=1,g=0.5,b=0):#Создание маркера
    markerArrayB.markers.append(Mark(x, y, z/2, 0.51, 0.51, z, len(markerArrayB.markers), 1, 0.5, 0, 1))
    pub_b.publish(markerArrayB)



def getBuilding(target_heights, arg=[1,0.5,0]):#Получаем параметры здания и запоминаем их, а так же ставим маркер
    p=None
    while p==None:
        p=pose("map")
    x,y,z,_,_,_=p
    real_height=int(round((z-rospy.wait_for_message('rangefinder/range', Range).range)/0.25))
    mtch=real_height==target_height
    Buildings.append({"coords":[x,y],"real_height":real_height,
                      "target_height":target_height,"match":mtch})
    Building_rviz_add(x,y,real_height*0.25,r=arg[0],g=arg[0],b=arg[0])


def navigate_wait_color(x=0, y=0, z=0, speed=0.5, frame=False, frame_id='body', auto_arm=False):
    global mask_now, red_conf, blue_conf, green_conf, yellow_conf
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    while not rospy.is_shutdown():
        frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x  ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = hsv[100:140, 140:180]
        mask_yellow = cv2.inRange(hsv, yellow[0], yellow[1])
        mask_red = cv2.inRange(hsv, red[0], red[1])
        mask_blue = cv2.inRange(hsv, blue[0], blue[1])
        mask_green = cv2.inRange(hsv, green[0], green[1])
        mask = mask_yellow + mask_red + mask_blue + mask_green
        image_hsv_pub.publish(bridge.cv2_to_imgmsg(mask, 'mono8'))
        M_yellow= cv2.moments(mask_yellow, 1)['m00']
        M_red = cv2.moments(mask_red, 1)['m00']
        M_blue = cv2.moments(mask_blue, 1)['m00']
        M_green = cv2.moments(mask_green, 1)['m00']
        telem = get_telemetry(frame_id='aruco_map')
        coord = (telem.x, telem.y)
        if M_yellow > 100 and yellow_conf:
            mask_now = (yellow[0], yellow[1])
            regulation()
            yellow_conf = False
            print("yellow okey")
            getBuilding(1)
            arr_color_coord["yellow"] =coord
            res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        elif M_red > 100 and red_conf:
            mask_now = (red[0], red[1])
            regulation()
            red_conf = False
            print("red okey")
            getBuilding(4,[1,0,0])
            arr_color_coord["red"] = coord
            res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        elif M_blue > 100 and blue_conf:
            mask_now = (blue[0], blue[1])
            regulation()
            blue_conf = False
            print("blue okey")
            getBuilding(3,[0,0,1])
            arr_color_coord["blue"] = coord
            res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        elif M_green > 100 and green_conf:
            mask_now = (green[0], green[1])
            regulation()
            green_conf = False
            print("green okey")
            getBuilding(2,[0,1,0])
            arr_color_coord["green"] = coord
            res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        rospy.sleep(0.2)


def buildRequest(): #Запрос на сервер для получения координат здания
    headers = {'accept': 'application/json', 'Authorization': 'Bearer 37bcefee-b9c9-4a66-9ef8-ee5e539d7d75', 'Content-Type': 'application/json'}
    payload = str(Buildings).replace("T","t").replace("F","f").replace("'",'"')
    r=requests.post("http://65.108.156.108/buildings", data=payload, headers=headers)
    return r.ok,r.text


def roadRequest(): #Запрос на сервер для получения координат здания
    headers = {'accept': 'application/json', 'Authorization': 'Bearer 37bcefee-b9c9-4a66-9ef8-ee5e539d7d75', 'Content-Type': 'application/json'}
    payload = str([road_data]).replace("T","t").replace("F","f").replace("'",'"')
    r=requests.post("http://65.108.156.108/roads", data=payload, headers=headers)
    return r.ok,r.text

#поиск дороги
road=[]
road_p=[]

def find_road(data):
    global is_run,psl,r_min, r_max
    x,y,_,_,_,_=pose("map")
    frame=bridge.imgmsg_to_cv2(data, 'bgr8') 
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    frame = cv2.resize(frame, (320, 240), interpolation= cv2.INTER_AREA)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV )
    mask = cv2.inRange(hsv, r_min, r_max)
    is_run=True
    arr = []
    if is_run:
        sr = 0
        for elem in mask[115:125]:
            n = 0
            for el in elem:
                if el > 100:
                    n += 1
            if n!= 0:
                arr.append(n)
                sr += n
        sr = sr/10
        arr_ok = []
        for elem in arr:
            if abs(elem - sr) < 5:
                arr_ok.append(elem)

        if len(arr_ok) > 0:
            road.append(sum(arr_ok)/len(arr_ok))
            road_p.append([x,y])




place=[(4.0, 1.0), (4.0, 4.0), (7.0, 4.0), (7.0, 1.0)]
hs=1.8
xs,ys=0,0
for x,y in place:
    xs+=x
    ys+=y
xs/=4
ys/=4

navigate_wait(z=1, frame_id='body', auto_arm=True)
telem = get_telemetry(frame_id='aruco_map')
xstart, ystart = telem.x, telem.y
rospy.sleep(0.5)

navigate_wait(x=xs, y=ys, z=hs, frame_id='aruco_map', speed=0.3, tolerance=0.2)
rospy.sleep(1)
img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
cv2.imwrite("building_yard.png", img)


frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
arr_coord = [(5, 3), (8, 3), (5, 4), (8, 4), (5, 5), (8, 5)]
navigate_wait_color(x=5, y=3, z=1.5, frame=frame, frame_id='aruco_map', speed=0.5)
for elem in arr_coord:
    navigate_wait_color(x=elem[0], y=elem[1], z=1.5, frame=frame, frame_id='aruco_map', speed=0.5)
    if not red_conf and not blue_conf and not green_conf and not yellow_conf:
        break


print(Buildings)
ok, txt=buildRequest()
if ok:
    x,y=map(float,txt.split('[')[1].split(']')[0].split(','))
    print(x,y)
    navigate_wait(x=x, y=y, z=hs, frame_id='aruco_map', speed=0.3, tolerance=0.2)
    rospy.sleep(1)
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    cv2.imwrite("building_we_interest.png", img)



#Здесь пролет по линиии и find_road подключение к камере

navigate_wait(x=xstart, y=ystart, z=1.5, frame_id=f'aruco_map', speed=0.5)
navigate_wait(x=0.5, y=0.5, z=1.5, yaw=0, frame_id=f'aruco_map', speed=0.5)
frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
navigate_wait(x=0.5, y=3.5, z=1.5, yaw=0, frame_id=f'aruco_map', speed=0.5)
frame.unregister()
navigate_wait(x=0.5, y=0.5, z=1.5, yaw=0, frame_id=f'aruco_map', speed=0.5)
#Обработка дороги
lines = []
m = []
s = 0
sr_line = 15
for i in range(len(road)):
    elem=road[i]
    if s == 0:
        s = elem
        m.append(elem)
    else:
        if abs(s - elem) < 3:
            s = (s*len(m)+elem)/(len(m)+1)
            m.append(elem)
        else:
            if len(m) > 2:
                if abs(s-sr_line)<=5:
                    lines.append([1,i])
                elif abs(s-sr_line*2)<=5:
                    lines.append([2,i])
                elif abs(s-sr_line*3)<=5:
                    lines.append([3,i])
            else:
                s = 0
            m = []
if len(m) > 2:
    if abs(s-sr_line)<=5:
        lines.append([1,len(road)-1])
    elif abs(s-sr_line*2)<=5:
        lines.append([1,len(road)-1])
    elif abs(s-sr_line*3)<=5:
        lines.append([1,len(road)-1])
lines.append([0,-1])

segments=[]
xp,yp=0.5,0.5
for i in range(len(lines)-1):
    if (lines[i][0]!=lines[i+1][0]):
        xn,yn=road_p[lines[i][1]]
        segments.append({"border":{"start":[xp,yp],"end":[xn,yn]}, "length":((xp-xn)**2+(yp-yn)**2)**0.5})
        xp,yp=xn,yn
length=((0.5-xn)**2+(0.5-yn)**2)**0.5
workers=[]
road_data={"length":length,"segments":segments,"workers":workers}
roadRequest()


navigate_wait(x=xstart, y=ystart, z=1.0, frame_id='aruco_map', speed=0.3, tolerance=0.15)
land()