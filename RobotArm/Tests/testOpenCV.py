import cv2
from cv2 import cv




cap = cv2.VideoCapture(1)
# print cap.get(cv.CV_CAP_PROP_FRAME_WIDTH), " ", cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
# print cap.set(cv.CV_CAP_PROP_FRAME_WIDTH,  2000)
# print cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 2000)
# print cap.get(cv.CV_CAP_PROP_FRAME_WIDTH), " ", cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)





while True:
    # Capture frame-by-frame
    ret, frame = cap.read()


    cv2.imshow('window', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()