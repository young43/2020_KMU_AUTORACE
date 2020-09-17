#!/usr/bin/env python
#-*- coding:utf-8 -*-

import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import *

from warper import Warper


class SlideWindow:
    def __init__(self):
        self.left_fit = None
        self.right_fit = None

        self.pre_leftx = 0
        self.pre_rightx = 0

        self.pre_xlocation = 0
        self.lane_width_hold = 260

        self.line_flag = 0

        self.mid_point = 330

        self.y_margin = 440
        self.x_margin = 0

    def w_slidewindow(self, img, dist_threshold=260):
        height, width = img.shape
        # print("Image Width : {}   Image Height : {}".format(width, height))

        roi_img = img[height-90:height-40, :].copy()
        roi_height, roi_width = roi_img.shape

        # print("ROI Width : {}   ROI Height : {}".format(roi_width, roi_height))

        cf_img = np.dstack((roi_img, roi_img, roi_img))

        window_height = 20
        window_width = 20

        # minpix : 30% number of total window pixel
        minpix = 7
        n_windows = roi_width // window_width // 2

        nonzero = roi_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        x_center = self.mid_point
        y_center = roi_height // 2  # 160

        pts_center = np.array([[x_center, 0], [x_center, roi_height]], np.int32)
        cv2.polylines(cf_img, [pts_center], False, (0, 120, 120), 1)

        left_idx = 0
        right_idx = 0

        find_left = False
        find_right = False

        left_start_x = None
        left_start_y = None

        right_start_x = None
        right_start_y = None


        leftx, rightx = None, None

        # dist_threshold = 180
        dist = None

        for i in range(0, n_windows):
            if find_left is False:
                win_left_y_low = y_center - window_height // 2
                win_left_y_high = y_center + window_height // 2

                win_left_x_high = x_center - left_idx * window_width
                win_left_x_low = x_center - (left_idx + 1) * window_width

            if find_right is False:
                win_right_y_low = y_center - window_height // 2
                win_right_y_high = y_center + window_height // 2

                win_right_x_low = x_center + right_idx * window_width
                win_right_x_high = x_center + (right_idx + 1) * window_width
            # print(win_left_y_low, ' ', win_left_x_low, ' ', win_left_y_high, ' ', win_left_x_high )

            cv2.rectangle(cf_img, (win_left_x_low, win_left_y_low), (win_left_x_high, win_left_y_high), (0, 255, 0), 1)
            cv2.rectangle(cf_img, (win_right_x_low, win_right_y_low), (win_right_x_high, win_right_y_high), (0, 0, 255),
                          1)

            good_left_inds = (
                        (nonzeroy >= win_left_y_low) & (nonzeroy < win_left_y_high) & (nonzerox >= win_left_x_low) & (
                            nonzerox < win_left_x_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_right_y_low) & (nonzeroy < win_right_y_high) & (
                        nonzerox >= win_right_x_low) & (nonzerox < win_right_x_high)).nonzero()[0]

            if len(good_left_inds) > minpix and find_left is False:
                tmp_x = np.int(np.mean(nonzerox[good_left_inds]))
                if abs(tmp_x - x_center) > int((self.lane_width_hold*0.4)):
                    find_left = True
                    left_start_x = np.int(np.mean(nonzerox[good_left_inds]))
                    left_start_y = roi_height // 2

                    for i in range(len(good_left_inds)):
                        cv2.circle(cf_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0, 255, 0), -1)
                else:
                    left_idx += 1
            else:
                left_idx += 1

            if len(good_right_inds) > minpix and find_right is False:

                tmp_x = np.int(np.mean(nonzerox[good_right_inds]))
                if abs(tmp_x - x_center) > int((self.lane_width_hold * 0.4)):
                    find_right = True
                    right_start_x = np.int(np.mean(nonzerox[good_right_inds]))
                    right_start_y = roi_height // 2

                    for i in range(len(good_right_inds)):
                        cv2.circle(cf_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 1, (0, 0, 255), -1)
                else:
                    right_idx += 1
            else:
                right_idx += 1

            if left_start_x is not None and right_start_x is not None:
                dist = right_start_x - left_start_x

                if dist_threshold - 100 < dist and dist < dist_threshold + 100:
                    cv2.circle(cf_img, (right_start_x, right_start_y), 3, (255, 0, 0), -1)
                    cv2.circle(cf_img, (left_start_x, left_start_y), 3, (255, 0, 0), -1)

                    self.pre_leftx = left_start_x
                    self.pre_rightx = right_start_x

                    self.mid_point = (left_start_x + right_start_x) // 2
                    break


        if left_start_x != None and right_start_x == None:
            right_start_x = int(min(width-50, left_start_x + (self.lane_width_hold)))
            self.pre_rightx = right_start_x
            self.line_flag = 1

        if right_start_x != None and left_start_x == None:
            left_start_x = int(max(50, right_start_x - (self.lane_width_hold)))
            self.pre_leftx = left_start_x
            self.line_flag = 2

        # mid_point 저장
        if left_start_x != None and right_start_x != None:
            temp = (left_start_x + right_start_x) // 2
            if abs(temp-self.mid_point) < 100 and (200 < temp < width-200):
                self.mid_point = temp

        return True, left_start_x, right_start_x, cf_img


    # def w_slidewindow(self, img):
    #     out_img = np.dstack((img, img, img))
    #     height, width = img.shape
    #
    #
    #     roi_img = out_img[self.y_margin-70:self.y_margin, self.x_margin:]
    #     roi_h = roi_img.shape[0]
    #     roi_w = roi_img.shape[1]
    #
    #     leftx = None
    #     rightx = None
    #
    #     # if self.pre_leftx != 0 and self.pre_rightx != 0:
    #     #     tmp_mid = (self.pre_rightx + self.pre_leftx) // 2
    #     #     if abs(self.mid_point-tmp_mid) <= 50:
    #     #         self.mid_point = tmp_mid
    #     #     else:
    #     #         self.mid_point = roi_w // 2
    #
    #     # self.mid_point = roi_w // 2
    #
    #
    #     nonzero = roi_img.nonzero()
    #     # print nonzero(행/열)
    #     nonzeroy = np.array(nonzero[0])
    #     nonzerox = np.array(nonzero[1])
    #
    #     cv2.line(roi_img, (self.mid_point, 0), (self.mid_point, roi_h), (255, 0, 0), 2)
    #
    #     margin = 35
    #     minpix = 10
    #     nwindows = 10
    #     win_height = 15
    #
    #     leftx_current = None
    #     rightx_current = None
    #
    #
    #
    #     # left w_window
    #     for window in range(nwindows):
    #         win_y_low = roi_h//2 - win_height
    #         win_y_high = roi_h//2 + win_height
    #
    #         win_xleft_low = (self.mid_point) - (window+1) * margin
    #         win_xleft_high = (self.mid_point) - window * margin
    #
    #         cv2.rectangle(roi_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
    #
    #         good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
    #                 nonzerox < win_xleft_high)).nonzero()[0]
    #
    #         for i in range(len(good_left_inds)):
    #             cv2.circle(roi_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0, 255, 0), -1)
    #
    #         if len(good_left_inds) > minpix:
    #             tmp_l = np.int(np.mean(nonzerox[good_left_inds]))
    #
    #             # 적어도 중간점과 100정도 차이 나야 차선이라고 인식
    #             if abs(tmp_l-self.mid_point) > int((self.lane_width_hold*0.3)):
    #                 if self.pre_leftx == 0 or abs(self.pre_leftx-tmp_l) < 150:
    #                     leftx_current = tmp_l
    #                     break
    #
    #     # right w_window
    #     for window in range(nwindows):
    #         win_y_low = roi_h//2 - win_height
    #         win_y_high = roi_h//2 + win_height
    #
    #         win_xright_low = (self.mid_point) + window * margin
    #         win_xright_high = (self.mid_point) + (window+1) * margin
    #
    #
    #         # Draw the windows on the visualization image
    #         cv2.rectangle(roi_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 0, 255), 2)
    #
    #         good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
    #                 nonzerox < win_xright_high)).nonzero()[0]
    #
    #         for i in range(len(good_right_inds)):
    #             cv2.circle(roi_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 1, (0, 0, 255), -1)
    #
    #
    #         if len(good_right_inds) > minpix:
    #             tmp_r = np.int(np.mean(nonzerox[good_right_inds]))
    #
    #             # print(tmp_r, self.mid_point)
    #
    #             if abs(tmp_r-self.mid_point) > int((self.lane_width_hold*0.3)):
    #                 if self.pre_rightx == 0 or abs(self.pre_rightx-tmp_r) < 150:
    #                     rightx_current = tmp_r
    #                     break
    #
    #
    #
    #     if leftx_current != None and rightx_current != None:
    #         lane_width = rightx_current - leftx_current
    #
    #         # 차선폭이 lane_width 값정도 되야 정상적인 좌표라고 저장함.
    #         if self.lane_width_hold-70 < lane_width < self.lane_width_hold+70:
    #             leftx = leftx_current
    #             rightx = rightx_current
    #
    #             # self.mid_point = (leftx + rightx) // 2
    #
    #             self.pre_leftx = leftx
    #             self.pre_rightx = rightx
    #
    #
    #     self.line_flag = 0
    #
    #     if leftx_current != None and rightx_current == None:
    #         # self.mid_point = int(min(width, leftx_current + (self.lane_width_hold*0.55)))
    #         leftx = leftx_current
    #         rightx = int(min(width, leftx_current + (self.lane_width_hold)))
    #         self.line_flag = 1
    #         self.pre_leftx = leftx
    #
    #     elif rightx_current != None and leftx_current == None:
    #         # self.mid_point = int(max(0, rightx_current - (self.lane_width_hold*0.55)))
    #         rightx = rightx_current
    #         leftx = int(max(0, rightx_current - (self.lane_width_hold)))
    #         self.line_flag = 2
    #         self.pre_rightx = rightx
    #
    #     if leftx == None:
    #         leftx = self.pre_leftx
    #     if rightx == None:
    #         rightx = self.pre_rightx
    #
    #
    #     # cv2.imshow("roi_img", roi_img)
    #
    #     # roi 이미지에서 실제 좌표로 변환
    #     leftx += self.x_margin
    #     rightx += self.x_margin
    #
    #
    #     return leftx, rightx


    def slidewindow(self, img, MODE="default"):

        x_location = None
        # init out_img, height, width

        # 255를 곱해주지 않음
        out_img = np.dstack((img, img, img))

        height = img.shape[0]
        width = img.shape[1]

        ret, leftx_current, rightx_current, cf_img = self.w_slidewindow(img)
        cv2.imshow("w_roi", cf_img)


        # print(self.mid_point,  leftx_current, rightx_current)

        histogram = np.sum(img[height // 2:, :], axis=0)
        if leftx_current == None and rightx_current == None:
            # midpoint = histogram.shape[0] // 2
            leftx_current = np.argmax(histogram[:self.mid_point-80])
            rightx_current = np.argmax(histogram[self.mid_point+80:]) + (self.mid_point+80)


        # find nonzero location in img, nonzerox, nonzeroy is the array flatted one dimension by x,y
        nonzero = img.nonzero()
        # print nonzero(행/열)
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])


        margin = 25
        minpix = 10
        left_lane_inds = []
        right_lane_inds = []
        nwindows = 15
        window_height = 12

        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = (img.shape[0]) - (window + 1) * window_height
            win_y_high = (img.shape[0]) - window * window_height

            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin

            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin


            if self.line_flag == 1:     # left
                win_xright_low = (leftx_current+(self.lane_width_hold)) - margin
                win_xright_high = (leftx_current+(self.lane_width_hold)) + margin
            elif self.line_flag == 2:   # right
                win_xleft_low = (rightx_current-(self.lane_width_hold)) - margin
                win_xleft_high = (rightx_current-(self.lane_width_hold)) + margin


            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 0, 255), 2)

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                        nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                        nonzerox < win_xright_high)).nonzero()[0]

            for i in range(len(good_left_inds)):
                cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0, 255, 0), -1)

            for i in range(len(good_right_inds)):
                cv2.circle(out_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 1, (0, 0, 255), -1)

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))

                if self.line_flag == 2:
                    self.line_flag = 0
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
                if self.line_flag == 1:
                    self.line_flag = 0

            # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        lx_current = rx_current = None

        left_fit = []
        right_fit = []

        # Fit a second order polynomial to each
        if lefty != [] and leftx != []:
            ymin, ymax = np.min(lefty), np.max(lefty)

            if ymax-ymin > 80:
                left_fit = np.polyfit(lefty, leftx, 2)
                lx_current = np.int(np.polyval(left_fit, win_y_high))

        if righty != [] and rightx != []:
            ymin, ymax = np.min(righty), np.max(righty)

            if ymax - ymin > 80:
                right_fit = np.polyfit(righty, rightx, 2)
                rx_current = np.int(np.polyval(right_fit, win_y_high))


        # if lx_current == None and rx_current != None:
        #     print("only right")
        #     left_fit = np.polyfit(righty, rightx - (self.lane_width_hold), 2)
        #     lx_current = np.int(np.polyval(left_fit, win_y_high))
        #
        # if rx_current == None and lx_current != None:
        #     print("only left")
        #     right_fit = np.polyfit(lefty, leftx + (self.lane_width_hold), 2)
        #     rx_current = np.int(np.polyval(right_fit, win_y_high))

        # print(lx_current, rx_current)

        if lx_current != None and rx_current != None:
            x_location = (lx_current + rx_current) // 2

        else:
            # 검출하지 못했을 때 예외처리
            if lx_current == None and rx_current != None:
                print("only right2")
                x_location = rx_current - int(self.lane_width_hold * 0.5)

            if rx_current == None and lx_current != None:
                print("only left2")
                x_location = lx_current + int(self.lane_width_hold * 0.5)

        # 그 이전 좌표를 가져오도록 함.
        if x_location != None:
            self.pre_xlocation = x_location
        else:
            x_location = self.pre_xlocation


        # print(lx_current, rightx_current)

        if MODE == "PARKING":
            cv2.line(out_img, (x_location, win_y_high), (380, height - 1), (0, 255, 255), 3)
        else:
            cv2.line(out_img, (x_location, win_y_high), (self.mid_point, height-1), (0,255,255), 3)

        return out_img, x_location


    def get_midpoint(self, MODE="default"):
        if MODE == "PARKING":
            return 380

        return self.mid_point


