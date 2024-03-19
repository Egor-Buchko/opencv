import cv2 #import OpenCV library
from cv2 import aruco
 
id = 0
cap = cv2.VideoCapture(id)

key=-1
while(key!=32):
    ret, frame = cap.read()
    if ret == True: # Display the resulting frame
        cv2.imshow("frame", frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Gray", gray)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        cv2.imshow("Gray", thresh)

        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if len(corners) > 0:
            print(ids.flatten()[0])
            if ids[0] < 10:
                print("yes")
            else:
                print("No")
        key=cv2.waitKey(25) # When everything done, release the video capture object
cap.release()
cv2.destroyAllWindows() #destroy windows

