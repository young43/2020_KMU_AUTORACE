
import cv2
import threading
import time
import numpy as np


from slidewindow import SlideWindow
from warper import Warper


def img_process(img):
    cols, rows, ch = img.shape
    brightness = np.sum(img) / (255 * cols * rows)

    minimum_brightness = 0.55
    ratio = brightness / minimum_brightness
    bright_img = cv2.convertScaleAbs(img, alpha = 1 / ratio, beta = 0)

    gray = cv2.cvtColor(bright_img, cv2.COLOR_BGR2GRAY)

    kernel_size = 5
    blur = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    low_threshold = 50
    high_threshold = 120
    edge = cv2.Canny(np.uint8(blur), low_threshold, high_threshold)

    return edge


def warpper_process(img):
    ret, thres_img = cv2.threshold(img, 80, 255, cv2.THRESH_BINARY)

    kernel = np.ones((5,5), np.uint8)
    dilate = cv2.dilate(thres_img, kernel, 3)

    sharp = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharp_img = cv2.filter2D(dilate, -1, sharp)

    return sharp_img



warper = None
slidewindow  = SlideWindow()

def main():
    flag = False
    cap = cv2.VideoCapture("org2.avi")


    while True:

        # 이미지를 캡쳐
        ret, img = cap.read()

        # 캡쳐되지 않은 경우 처리
        if not ret:
            break
        if cv2.waitKey(1) & 0xFF == 27:
            break

        # Warper 객체 생성 (초기 1번만)
        if not flag:
            flag = True
            global  warper
            warper = Warper(img)

        # warper, slidewindow 실행
        # slideImage, x_location = process_image(img)
        process_img = img_process(img)
        warp_img = warper.warp(process_img)
        process_img2 = warpper_process(warp_img)

        slideImage, x_location = slidewindow.slidewindow(process_img2)





        # cv2.imshow("originImage", img)
        # cv2.imshow("warper", warper.warp(img))
        cv2.imshow("slidewindow", slideImage)
        # cv2.imshow("unwarp", warper.unwarp(slideImage))
        #cv2.imshow("processImg", img1)



if __name__ == '__main__':
    main()