import cv2

id = 1  # camera id
cap = cv2.VideoCapture(id + cv2.CAP_DSHOW)
key = -1
while key != 27:  # Esc
    ret, frame = cap.read()
    if ret:  # Display the resulting frame
        frame = frame[0:480, 0:640]
        frame = cv2.flip(frame, -1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cv2.imshow('frame', frame)
        cv2.imshow('gray', gray)
        cv2.imshow('hsv', hsv)
    key = cv2.waitKey(25)
cap.release()  # When everything done, release the video capture object
cv2.destroyAllWindows()  # destroy all windows
