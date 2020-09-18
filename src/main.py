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

    low_threshold = 60
    high_threshold = 70
    edge = cv2.Canny(np.uint8(blur), low_threshold, high_threshold)

    roi = roi_interest(edge)
    # cv2.imshow('roi', roi)


    return edge


def roi_interest(img):
    width, height = img.shape[:2]

    vertices = np.array(
        [[
            (-300, 430),  # 좌하
            (240, 280),   # 좌상
            (420, 280),  # 우상
            (width + 300, 430)   # 우하
          ]], dtype=np.int32)

    mask = np.zeros_like(img)  # mask = img와 같은 크기의 빈 이미지

    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움
    cv2.fillPoly(mask, vertices, 255)

    # 이미지와 color로 채워진 ROI를 합침
    roi_image = cv2.bitwise_and(img, mask)


    return roi_image


def warpper_process(img):
    ret, thres_img = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)

    kernel = np.ones((3,3), np.uint8)
    dilate = cv2.dilate(thres_img, kernel, 3)


    sharp = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharp_img = cv2.filter2D(dilate, -1, sharp)


    return thres_img


def hough_line(img):
    outimg = np.zeros_like(img)
    outimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2BGR)


    minLineLength = 10
    maxLineGap = 50
    lines = cv2.HoughLinesP(img, 1, np.pi / 180, 5, minLineLength, maxLineGap)
    for x in range(0, len(lines)):
        for x1, y1, x2, y2 in lines[x]:
            cv2.line(outimg,(x1,y1),(x2,y2),(0,255,0),10, cv2.LINE_AA)
            pts = np.array([[x1, y1], [x2, y2]], np.int32)
            cv2.polylines(outimg, [pts], True, (0, 255, 0))



    return outimg

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
            angle = np.rad2deg(pid)

            # print(pid, degree)

        else:
            pid = round(pidcal.pid_control(int(x_location_old), curve_detector.curve_count, mid_point), 6)
            angle = np.rad2deg(pid)


        curve_detector.update(pid)
        curve_detector.count_curve()

        print(angle, x_location - mid_point)

        # cv2.imshow("originImage", img)
        # cv2.imshow("warper", warper.warp(img))
        cv2.imshow("slidewindow", slideImage)
        # cv2.imshow("unwarp", warper.unwarp(slideImage))
        #cv2.imshow("processImg", img1)


if __name__ == '__main__':
    main()