
import cv2
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(1)

while True:
    ret0, img0 = cap.read()
    #img0 = cv2.imread("F:\Google Drive\Projects\Git Repositories\RobotStorage\RobotArm\stitched.png")
    img = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)




    low  = float(raw_input("Low?:"))    #Default 127
    high = float(raw_input("High?:"))   #Default 255
    #low = 127
    #high = 255

    ret1,  thresh1  = cv2.threshold(img, low, high, cv2.THRESH_BINARY)
    ret2,  thresh2  = cv2.threshold(img, low, high, cv2.THRESH_BINARY_INV)
    ret3,  thresh3  = cv2.threshold(img, low, high, cv2.THRESH_TRUNC)
    ret4,  thresh4  = cv2.threshold(img, low, high, cv2.THRESH_TOZERO)
    ret5,  thresh5  = cv2.threshold(img, low, high, cv2.THRESH_TOZERO_INV)
    ret6,  thresh6  = cv2.threshold(img, low, high, cv2.THRESH_OTSU)
    ret7,  thresh7  = cv2.threshold(img, low, high, cv2.ADAPTIVE_THRESH_GAUSSIAN_C)
    ret8,  thresh8  = cv2.threshold(img, low, high, cv2.CALIB_CB_ADAPTIVE_THRESH)
    ret9,  thresh9  = cv2.threshold(img, low, high, cv2.ADAPTIVE_THRESH_MEAN_C)


    titles = ['Original Image', 'BINARY', 'BINARY_INV', 'TRUNC', 'TOZERO', 'TOZERO_INV', 'OTSU', 'ADAPTIVE GAUSSIAN', 'ADAPTIVE CALIB', 'ADAPTIVE MEAN']
    images = [img, thresh1, thresh2, thresh3, thresh4, thresh5, thresh6, thresh7, thresh8, thresh9]

    for i in xrange(10):
        plt.subplot(5, 4, i + 1), plt.imshow(images[i], 'gray')
        plt.title(titles[i])
        plt.xticks([]), plt.yticks([])

    plt.show()