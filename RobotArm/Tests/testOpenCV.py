import cv2


cap = cv2.VideoCapture(1)


while True:
    # Capture frame-by-frame
    ret, frame = cap.read()


    cv2.imshow('window', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()