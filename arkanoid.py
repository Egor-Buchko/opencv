from ev3 import Ev3
import time
import cv2
import sys
import os

#-----------------VARIABLES----------------------------------------
id=1
key=-1
h_min, h_max, s_min, s_max, v_min, v_max = 0, 255, 0 , 255, 0, 255
n_min, n_max = 30000, 40000
cXB, cXR, cYB, cYR = 0, 0, 0, 0
count = 0
avto, hand, motors, visual = False, False, False, False
kick = False
ball_x =  0
arr_coord = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0)]
old_x, old_y = 0, 0
k_p = 5.5
#-------------------------------------------------------------------



def show_trackbar(name): 
    if name == 'detect':      
        cv2.namedWindow("detect")
        cv2.resizeWindow("detect", 500, 400) 
        cv2.createTrackbar("sat_min", "detect",s_min,255,sat_min)
        cv2.createTrackbar("val_min", "detect",v_min,255,val_min)
        cv2.createTrackbar("sat_max", "detect",s_max,255,sat_max)
        cv2.createTrackbar("val_max", "detect",v_max,255,val_max)       
    elif name == 'detect2':
        cv2.namedWindow("detect2")
        cv2.resizeWindow("detect2", 500, 400)
        cv2.createTrackbar("hue_min", "detect2",h_min2,179,hue_min2)
        cv2.createTrackbar("sat_min", "detect2",s_min2,255,sat_min2)
        cv2.createTrackbar("val_min", "detect2",v_min2,255,val_min2)
        cv2.createTrackbar("hue_max", "detect2",h_max2,179,hue_max2)
        cv2.createTrackbar("sat_max", "detect2",s_max2,255,sat_max2)
        cv2.createTrackbar("val_max", "detect2",v_max2,255,val_max2)
    elif name == 'detect_cr':
        cv2.namedWindow("detect_cr")
        cv2.resizeWindow("detect_cr", 500, 400)
        cv2.createTrackbar("leftCray", "detect_cr",left_cr,200,left_cray)
        cv2.createTrackbar("rightCray", "detect_cr",right_cr,200,right_cray)
        cv2.createTrackbar("robLeftCr", "detect_cr",robot_left_cr,200,robot_left_cray)
        cv2.createTrackbar("robRightCr", "detect_cr",robot_right_cr,200,robot_right_cray)
        cv2.createTrackbar("robDownCr", "detect_cr",robot_down_cr,200,robot_down_cray)
        cv2.createTrackbar("robUpCr", "detect_cr",robot_up_cr,200,robot_up_cray)

with open(f"{os.getcwd()}/t3.txt", "r") as f:
    text = f.read()
    left_cr, right_cr, robot_left_cr, robot_right_cr, robot_down_cr, robot_up_cr = map(int, text.split())
#------------------------------LINE-----------------------------
def left_cray(current):
    global left_cr
    left_cr=current

def right_cray(current):
    global right_cr
    right_cr=current

def robot_right_cray(current):
    global robot_right_cr
    robot_right_cr=current

def robot_left_cray(current): 
    global robot_left_cr
    robot_left_cr=current

def robot_down_cray(current):
    global robot_down_cr
    robot_down_cr=current

def robot_up_cray(current):
    global robot_up_cr
    robot_up_cr = current

#------------------------------BALL-----------------------------
with open(f"{os.getcwd()}/t.txt", "r") as f:
    text = f.read()
    s_min, s_max, v_min, v_max = map(int, text.split())
    
def sat_min(current):
    global s_min
    s_min=current

def val_min(current):
    global v_min
    v_min=current
    
def sat_max(current):
    global s_max
    s_max=current

def val_max(current):
    global v_max
    v_max=current

cv2.namedWindow("frame")
#------------------------------ROBOT-----------------------------
with open(f"{os.getcwd()}/t2.txt", "r") as f:
    text = f.read()
    h_min2, h_max2, s_min2, s_max2, v_min2, v_max2 = map(int, text.split())


def hue_min2(current):
    global h_min2
    h_min2=current
    
def sat_min2(current):
    global s_min2
    s_min2=current

def val_min2(current):
    global v_min2
    v_min2=current 

def hue_max2(current):
    global h_max2
    h_max2=current
    
def sat_max2(current):
    global s_max2
    s_max2=current

def val_max2(current):
    global v_max2
    v_max2=current

#-----------------------------MAIN----------------------------------

# Указываем последовательный порт в конструкторе windows: 'COM...' linux: '/dev/rfcomm....'
bot = Ev3("COM4")
cap = cv2.VideoCapture(id + cv2.CAP_DSHOW)
bot.set_speed(Ev3.Motors.A, 0)
bot.set_speed(Ev3.Motors.B, 0)
bot.set_speed(Ev3.Motors.C, 0)
bot.set_speed(Ev3.Motors.D, 0)
bot.start(Ev3.Motors.A)
bot.start(Ev3.Motors.B)
bot.start(Ev3.Motors.C)
bot.start(Ev3.Motors.D)
while key != 32:
    ret, frame = cap.read()
    #frame = cv2.flip(frame, 0)
    height, width, _ = frame.shape
    frame = frame[0:height, left_cr:width - right_cr]
    height_new, width_new, _ = frame.shape
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv, (0, s_min, v_min), (5, s_max, v_max)) + cv2.inRange(hsv, (174, s_min, v_min), (179, s_max, v_max))
    thresh2 = cv2.inRange(hsv, (h_min2, s_min2, v_min2), (h_max2, s_max2, v_max2))
    thresh = thresh[robot_up_cr:480 - robot_down_cr, 0:width_new]
    thresh2 = thresh2[robot_up_cr:480, 0:width_new]
    cv2.line(frame, (robot_left_cr, 0), (robot_left_cr, 480), (0, 0, 0), 1)
    cv2.line(frame, (width_new - robot_right_cr, 0), (width_new - robot_right_cr, 480), (0, 0, 0), 1)
    if key == ord('v'):
        show_trackbar('detect2')
        show_trackbar('detect')
        show_trackbar('detect_cr')
        visual = True
    elif key == ord('b'):
        cv2.destroyAllWindows()
        visual = False

    if key == ord('q'):
        avto, hand, motors = True, False, False
        bot.stop(Ev3.Motors.B, Ev3.Stop.FLOAT)
        bot.stop(Ev3.Motors.C, Ev3.Stop.FLOAT)
    elif key == ord('e'):
        avto, hand, motors = False, True, False
        bot.stop(Ev3.Motors.B, Ev3.Stop.FLOAT)
        bot.stop(Ev3.Motors.C, Ev3.Stop.FLOAT)
    elif key == ord('s'):
        bot.stop(Ev3.Motors.A, Ev3.Stop.FLOAT)
        bot.stop(Ev3.Motors.B, Ev3.Stop.FLOAT)
        bot.stop(Ev3.Motors.C, Ev3.Stop.FLOAT)
        bot.stop(Ev3.Motors.D, Ev3.Stop.FLOAT)
        avto, hand, motors = False, False, False
    if avto: # auto
        contours, hierarchy = cv2.findContours(image=thresh2, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(image=frame, contours=contours, contourIdx=-1, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
        if len(contours) != 0:
            contour = max(contours, key = cv2.contourArea)
            center, radius = cv2.minEnclosingCircle(contour)
            cXR, cYR = (int(center[0]), int(center[1])+robot_up_cr)
            cv2.circle(frame, (cXR, cYR), int(radius), (0, 255, 0), 2)
        
        contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(image=frame, contours=contours, contourIdx=-1, color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)
        if len(contours) != 0:
            contour = max(contours, key = cv2.contourArea)
            center, radius = cv2.minEnclosingCircle(contour)
            cXB, cYB = (int(center[0]), int(center[1])+robot_up_cr)
            cv2.circle(frame, (cXB, cYB), int(radius), (0, 255, 0), 2)
        else:
            cXB, cYB = (height_new // 2, width_new // 2)
        x = (cXB, cYB)
        arr_coord.append(x)
        if len(arr_coord) == 100:
            arr_coord = arr_coord[-10:]
        if len(arr_coord) > 9:
            new_x, new_y = arr_coord[-1][0], arr_coord[-1][1]
            old_x, old_y = arr_coord[-10][0], arr_coord[-10][1]
            if new_y - old_y < 0 or abs(new_y - old_y) < 10:#если движение вверх или маленькая скорость
                ball_x = new_x
            elif old_x != 0 and old_y != 0 and old_x != new_x: # old_x != new_x and old_y != new_y
                k = (old_y - new_y) / (old_x - new_x) # tgA = x / y 
                b = old_y - k * old_x # y = kx - b --> b = y - kx
                if k != 0:
                    ball_x = int(abs((height_new - robot_down_cr - b) // k))
                    if ball_x > width_new:
                        ball_x = 2 * width_new - ball_x
        cv2.line(frame, (cXB, cYB), (ball_x, height_new - robot_down_cr), (0, 255, 255), 1)
        cXB = ball_x
        if not motors:
            motors = True
            bot.start(Ev3.Motors.B)
            bot.start(Ev3.Motors.C)
        if abs(cXR - cXB) < 40 or (cXR < cXB and cXR > width_new - robot_right_cr) or (cXR > cXB and cXR < robot_left_cr):
            bot.set_speed(Ev3.Motors.C, 0)
            bot.set_speed(Ev3.Motors.B, 0)
        elif cXB > cXR:
            bot.set_speed(Ev3.Motors.B, -127)# право
            bot.set_speed(Ev3.Motors.C, 127)
        elif cXB < cXR:
            bot.set_speed(Ev3.Motors.B, 127)# лево
            bot.set_speed(Ev3.Motors.C, -127)
        
        if len(arr_coord) > 9:
            speed = (cYB - arr_coord[-10][1])*4
            if not kick:
                if (speed < 10 and cYR - cYB < 70) or (cYR - cYB < speed / k_p):
                    bot.start(Ev3.Motors.A)
                    bot.start(Ev3.Motors.D)
                    kick = True
                    n = 0
                    bot.set_speed(Ev3.Motors.A, -127)
                    bot.set_speed(Ev3.Motors.D, 127)
    elif hand: # manual

        if not motors:
                motors = True
                bot.start(Ev3.Motors.B)
                bot.start(Ev3.Motors.C)
        
        if key == ord('w') and not kick:
            kick = True
            n = 0
            bot.set_speed(Ev3.Motors.A, -120)
            bot.set_speed(Ev3.Motors.D, 120)
            bot.start(Ev3.Motors.A)
            bot.start(Ev3.Motors.D)
        elif key == ord('d'): # право
            bot.set_speed(Ev3.Motors.B, -60)
            bot.set_speed(Ev3.Motors.C, 60)
        elif key == ord('a'): # left
            bot.set_speed(Ev3.Motors.B, 60)
            bot.set_speed(Ev3.Motors.C, -60)
        else:
            bot.set_speed(Ev3.Motors.C, 0)
            bot.set_speed(Ev3.Motors.B, 0)
    if kick:
        n += 1
    if kick and n > 3:
        bot.set_speed(Ev3.Motors.A, 20)
        bot.set_speed(Ev3.Motors.D, -20)
    if kick and n > 20:
        bot.set_speed(Ev3.Motors.A, 0)
        bot.set_speed(Ev3.Motors.D, 0)
        kick = False
        bot.stop(Ev3.Motors.A, Ev3.Stop.FLOAT)
        bot.stop(Ev3.Motors.D, Ev3.Stop.FLOAT)
    if visual:
        if key == ord('z'):
            text=f"{s_min} {s_max} {v_min} {v_max}"
            with open(f"{os.getcwd()}/t.txt", "w") as f:
                f.write(text)
        elif key == ord('x'):
            print('save')
            text=f"{h_min2} {h_max2} {s_min2} {s_max2} {v_min2} {v_max2}"
            with open(f"{os.getcwd()}/t2.txt", "w") as f:
                f.write(text)
        elif key == ord('c'):
            text=f"{left_cr} {right_cr} {robot_left_cr} {robot_right_cr} {robot_down_cr} {robot_up_cr}"
            with open(f"{os.getcwd()}/t3.txt", "w") as f:
                f.write(text)
        contours, hierarchy = cv2.findContours(image=thresh2, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(image=frame, contours=contours, contourIdx=-1, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
        if len(contours) != 0:
            contour = max(contours, key = cv2.contourArea)
            center, radius = cv2.minEnclosingCircle(contour)
            cXR, cYR = int(center[0]), int(center[1])+robot_up_cr
            cv2.circle(frame, (cXR, cYR), int(radius), (0, 255, 0), 2)
        
        contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(image=frame, contours=contours, contourIdx=-1, color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)
        if len(contours) != 0:
            contour = max(contours, key = cv2.contourArea)
            center, radius = cv2.minEnclosingCircle(contour)
            cXB, cYB = int(center[0]), int(center[1])+robot_up_cr
            cv2.circle(frame, (cXB, cYB), int(radius), (0, 255, 0), 2)
        x = (cXB, cYB)
        cv2.imshow("mask", thresh)
        cv2.imshow("mask2", thresh2)
    cv2.imshow("frame",frame)
    key=cv2.waitKey(25)

# Остановка мотора  
bot.stop(Ev3.Motors.A, Ev3.Stop.FLOAT)
bot.stop(Ev3.Motors.B, Ev3.Stop.FLOAT)
bot.stop(Ev3.Motors.C, Ev3.Stop.FLOAT)
bot.stop(Ev3.Motors.D, Ev3.Stop.FLOAT)
# Второй аргумент `type` - тип остановки: BREAK - с фиксацией мотора, FLOAT - без

bot.close()
cap.release()
cv2.destroyAllWindows()
