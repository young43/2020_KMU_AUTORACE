#!/usr/bin/env python
#-*- coding:utf-8 -*-

import cv2
import numpy as np
import time

class StopDetector:
    def __init__(self):
        self.cnt = 0
        self.previous_time = time.time() + 5
        self.previous_time2 = time.time() + 5
        self.lower_yellow = (20, 100, 100)
        self.upper_yellow = (40, 255, 255)

    def check_time(self):
        if time.time() < self.previous_time + 10:
            return False
        return True

    def check_time2(self):
        if time.time() < self.previous_time2 + 10:
            return False
        return True

    def check_yellow_line(self, img):
        out_img = np.copy(img)

        stop_roi = cv2.cvtColor(img[380:420, 140:550], cv2.COLOR_BGR2HSV)
        cv2.rectangle(out_img, (140, 380), (550, 420), (210, 100, 55), 2)

        lower_yellow = (20, 100, 100)
        upper_yellow = (40, 255, 255)

        img_mask = cv2.inRange(stop_roi, lower_yellow, upper_yellow)

        # cv2.imshow("yellow", out_img)

        if np.count_nonzero(img_mask) > 60 and self.check_time():
            self.on_detected_stopline()
            return True

        return False

    def on_detected_stopline(self):
        print('STOP LINE DETECTED!!!')
        self.previous_time = time.time()
        self.cnt += 1


    def on_detected_crosswallk(self):
        print('CROSS WALK DETECTED!!!')
        self.previous_time2 = time.time()


    def check_crocss_walk(self, warp_img, img):
        out_img = np.copy(warp_img)


        # stop_roi = cv2.cvtColor(out_img[380:420, 140:550], cv2.COLOR_BGR2HSV)
        stop_roi = out_img[390:420, 140:550]
        cv2.rectangle(out_img, (140, 380), (550, 420), (210, 100, 55), 2)

        # low_threshold = np.array([0, 0, 120], dtype=np.uint8)
        # high_threshold = np.array([255, 255, 255], dtype=np.uint8)
        #
        # mask = cv2.inRange(stop_roi, low_threshold, high_threshold)
        #
        # cv2.imshow("mask", mask)

        print("cross:", np.count_nonzero(stop_roi))

        # 정지선 nonzero 값 프린트 해서 조정
        if np.count_nonzero(stop_roi) > 1800 and not self.check_yellow_line(img) and self.check_time2():
            self.on_detected_crosswallk()
            return True

        return False

    # def check_crocss_walk(self, cross_img, warp_img, img):
    #     template = cv2.imread("/home/nvidia/xycar_ws/src/racecar/img/cross.png", cv2.IMREAD_GRAYSCALE)
    #     res = cv2.matchTemplate(cross_img, template, cv2.TM_CCOEFF)
    #     min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    #
    #     out_img = np.copy(warp_img)
    #
    #     # stop_roi = cv2.cvtColor(out_img[380:420, 140:550], cv2.COLOR_BGR2HSV)
    #     stop_roi = out_img[390:420, 140:550]
    #     cv2.rectangle(out_img, (140, 380), (550, 420), (210, 100, 55), 2)
    #
    #     # low_threshold = np.array([0, 0, 120], dtype=np.uint8)
    #     # high_threshold = np.array([255, 255, 255], dtype=np.uint8)
    #     #
    #     # mask = cv2.inRange(stop_roi, low_threshold, high_threshold)
    #     #
    #     # cv2.imshow("mask", mask)
    #
    #     print("cross:", np.count_nonzero(stop_roi), min_loc)
    #
    #     if np.count_nonzero(stop_roi) > 1700 and not self.check_yellow_line(img):  # 정지선 nonzero 값 프린트 해서 조정
    #         self.on_detected_crosswallk()
    #         return True
    #
    #     return False



if __name__ == '__main__':
    stop_counter = StopDetector()

    cap = cv2.VideoCapture('../video/org2.avi')
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        #  stop_counter.check_yellow_line(frame)
        stop_counter.check_crocss_walk(frame)


        # cv2.imshow('frame', detect_img)
        if cv2.waitKey(0) == 27:
            break