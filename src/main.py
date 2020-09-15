#!/usr/bin/env python


import cv2
import threading
import time
import numpy as np
import math


from slidewindow import SlideWindow
from warper import Warper
from pidcal import PidCal
from CurveDetector import Curve




def img_process(img):
    cols, rows, ch = img.shape
    brightness = np.sum(img) / (255 * cols * rows)

    minimum_brightness = 0.75
    ratio = brightness / minimum_brightness
    bright_img = cv2.convertScaleAbs(img, alpha = 1 / ratio, beta = 0)


    gray = cv2.cvtColor(bright_img, cv2.COLOR_BGR2GRAY)

    kernel_size = 5
    blur = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    low_threshold = 40
    high_threshold = 70
    edge = cv2.Canny(np.uint8(blur), low_threshold, high_threshold)

    return edge


def warpper_process(img):
    ret, thres_img = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY)

    kernel = np.ones((5,5), np.uint8)
    dilate = cv2.dilate(thres_img, kernel, 5)


    sharp = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharp_img = cv2.filter2D(thres_img, -1, sharp)

    return thres_img


def calc_angle(distance):
    # atan2(double y, double x)
    if abs(distance) < 35:
        angle = round((np.arctan2(distance, 220) * 180 / np.pi), 2)
    else:
        angle = round((np.arctan2(distance, 130) * 180 / np.pi), 2)

    if angle < 0:
        angle = max(-50, angle)
    else:
        angle = min(50, angle)

    return angle


slidewindow  = SlideWindow()
pidcal = PidCal()
curve_detector = Curve()
warper = None
def main():
    global warper

    flag = False
    cap = cv2.VideoCapture("../video/org2.avi")

    x_location_old = None

    while True:

        # 이미지를 캡쳐
        ret, img = cap.read()

        # 캡쳐되지 않은 경우 처리
        if not ret:
            break
        if cv2.waitKey(0) & 0xFF == 27:
            break

        if warper == None:
            warper = Warper(img)

        # warper, slidewindow 실행
        process_img = img_process(img)
        warp_img = warper.warp(process_img)
        process_img2 = warpper_process(warp_img)

        # slidewindow.w_slidewindow2(process_img2)
        # slideImage, x_location = slidewindow.slidewindow(process_img2, MODE="PARKING")
        # mid_point = slidewindow.get_midpoint(MODE="PARKING")

        slideImage, x_location = slidewindow.slidewindow(process_img2)
        mid_point = slidewindow.get_midpoint()

        # print(x_location, mid_point)

        if x_location != None:
            # test 4 lines
            if curve_detector.curve_count == 3 and np.abs(x_location - x_location_old) > 40:
                x_location = x_location_old
            else:
                x_location_old = x_location

            pid = round(pidcal.pid_control(int(x_location), curve_detector.curve_count, mid_point), 6)
            degree = np.rad2deg(pid)

            print(pid, degree)


            angle = calc_angle(x_location - mid_point)

        else:
            angle = calc_angle(x_location_old - mid_point)

        curve_detector.update(angle)

        # print(angle, x_location - mid_point)
        # print(Angle, curve.get_avg(), curve.is_curve())

        # cv2.imshow("originImage", img)
        # cv2.imshow("warper", warper.warp(img))
        cv2.imshow("slidewindow", slideImage)
        # cv2.imshow("unwarp", warper.unwarp(slideImage))
        #cv2.imshow("processImg", img1)



if __name__ == '__main__':
    main()