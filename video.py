import cv2
import numpy as np
vid_capture = cv2.VideoCapture('aaa2.mp4')
h_min, h_max = (25, 95, 156), (33, 212, 255)
fps = vid_capture.get(5)
print('Фреймов в секунду: ', fps,'FPS')
arr_2 = []
k = 0
sr_line = 15
while(vid_capture.isOpened()):
    ret, frame = vid_capture.read()
    if ret == True: 
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        frame = cv2.resize(frame, (320, 240), interpolation= cv2.INTER_AREA)
        slice_1 = frame[115:125, 0:320]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV )
        mask = cv2.inRange(hsv, h_min, h_max)
        arr = []
        if k % 4 == 0:
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
                arr_2.append(sum(arr_ok)/len(arr_ok))
            print(arr_ok)
        k += 1
        cv2.imshow('', mask)
        cv2.imshow('sl', slice_1)
        cv2.imshow('Look', frame)
        key = cv2.waitKey(20)
        if (key == ord('q')) or key == 27:
            break
    else:
        break

print(arr_2)
# Освободить объект захвата видео
vid_capture.release()
cv2.destroyAllWindows()
#print(arr_2)
lines = []
m = []
s = 0
for elem in arr_2:
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
                    lines.append(1)
                elif abs(s-sr_line*2)<=5:
                    lines.append(2)
                elif abs(s-sr_line*3)<=5:
                    lines.append(3)
            else:
                s = 0
            m = []
if len(m) > 2:
    if abs(s-sr_line)<=5:
        lines.append(1)
    elif abs(s-sr_line*2)<=5:
        lines.append(2)
    elif abs(s-sr_line*3)<=5:
        lines.append(3)
print(lines)

"""k_arr = []
for i in range(len(arr_2)):
    m = i
    arr_s = []
    arr_err = []
    for j in range(10):
        if m < 0:
            break
        x = arr_2[m][j]
        arr_s.append(x)
        m -= 1
    k_arr.append(arr_s)
    print(arr_s)"""