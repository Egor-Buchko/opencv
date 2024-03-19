# https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
# https://robotclass.ru/tutorials/opencv-python-find-contours/
# https://robotclass.ru/tutorials/opencv-detect-rectangle-angle/
import cv2
import numpy as np
hsv_min = np.array((28, 50, 97), np.uint8)
hsv_max = np.array((68, 120, 131), np.uint8)
if __name__ == "__main__":
    id = 0  # camera id
    cap = cv2.VideoCapture(id)
    key = -1
    while key != 27:  # Esc
        ret, frame = cap.read()
        if ret:  # Display the resulting frame
            cv2.imshow('frame', frame)
            hsv = cv2.cvtColor(cv2.blur(frame, (9, 9)), cv2.COLOR_BGR2HSV)
            thresh = cv2.inRange(hsv, hsv_min, hsv_max)
            cv2.imshow('thresh', thresh)
            contours, hierarchy = cv2.findContours(
                thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) > 2:
                    cv2.drawContours(frame, contour, 0, (255, 0, 0), 2)

            cv2.imshow('contours', frame)
        key = cv2.waitKey(25)
    cap.release()  # When everything done, release the video capture object
    cv2.destroyAllWindows()  # destroy windows
