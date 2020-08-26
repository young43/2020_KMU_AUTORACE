import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import *
from matplotlib.pyplot import *


class SlideWindow:
    def __init__(self):
        self.left_fit = None
        self.right_fit = None

        self.leftx = None
        self.rightx = None

        self.pre_leftx = None
        self.pre_rightx = None

        self.pre_xlocation = 320

    def w_slidewindow(self, img):
        out_img = np.dstack((img, img, img))
        height, width = img.shape

        y_margin = 300
        x_margin = 0

        roi_img = out_img[y_margin:height-50, x_margin:-30]
        roi_h = roi_img.shape[0]
        roi_w = roi_img.shape[1]
        mid_point = roi_w // 2

        if self.pre_leftx != None and self.pre_rightx != None:
            mid_point = (self.pre_rightx + self.pre_leftx) // 2

        nonzero = roi_img.nonzero()
        # print nonzero(행/열)
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        cv2.line(roi_img, (mid_point, 0), (mid_point, roi_h), (255, 0, 0), 2)
        lane_width_hold = 270

        margin = 40
        minpix = 30
        nwindows = 10
        win_height = 15

        leftx_current = None
        rightx_current = None

        # left w_window
        for window in range(nwindows):
            win_y_low = roi_h//2 - win_height
            win_y_high = roi_h//2 + win_height

            win_xleft_low = (mid_point) - (window+1) * margin
            win_xleft_high = (mid_point) - window * margin

            cv2.rectangle(roi_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                    nonzerox < win_xleft_high)).nonzero()[0]

            for i in range(len(good_left_inds)):
                cv2.circle(roi_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0, 255, 0), -1)

            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))

                # 적어도 중간점과 150정도 차이 나야 차선이라고 인식
                if abs(leftx_current-mid_point) > 80:
                    break

        # right w_window
        for window in range(nwindows):
            win_y_low = roi_h//2 - win_height
            win_y_high = roi_h//2 + win_height

            win_xright_low = (mid_point) + window * margin
            win_xright_high = (mid_point) + (window+1) * margin


            # Draw the windows on the visualization image
            cv2.rectangle(roi_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 0, 255), 2)

            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                    nonzerox < win_xright_high)).nonzero()[0]

            for i in range(len(good_right_inds)):
                cv2.circle(roi_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 1, (0, 0, 255), -1)


            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
                if abs(rightx_current-mid_point) > 80:
                    break


        if leftx_current != None and rightx_current != None:
            lane_width = rightx_current - leftx_current

            # 차선폭이 lane_width 값정도 되야 정상적인 좌표라고 저장함.
            if lane_width_hold-60 < lane_width < lane_width_hold+60:
                self.leftx = self.pre_leftx = leftx_current
                self.rightx = self.pre_rightx = rightx_current

        else:
            if leftx_current == None:
                self.leftx = self.pre_leftx

            elif rightx_current == None:
                self.rightx = self.pre_rightx

            else:
                print("None!!!")


        # print(self.rightx - self.leftx)
        cv2.imshow("roi_img", roi_img)

        # roi 이미지에서 실제 좌표로 변환
        real_leftx = real_rightx = None
        if self.leftx != None and self.rightx != None:
            real_leftx = self.leftx + x_margin
            real_rightx = self.rightx + x_margin

        return real_leftx, real_rightx


    def slidewindow(self, img):

        x_location = None
        # init out_img, height, width

        # 255를 곱해주지 않음
        out_img = np.dstack((img, img, img))

        leftx_current, rightx_current = self.w_slidewindow(img)

        # 혹시 w_slidewindow 를 통해 좌표가 구해지지 않았으면 히스토그램을 통해 좌표를 가져오도록 함.
        if leftx_current == None or rightx_current == None:
            # Take a histogram of the bottom half of the image
            histogram = np.sum(img[img.shape[0] // 2:, :], axis=0)
            midpoint = histogram.shape[0] // 2
            leftx_base = np.argmax(histogram[100:midpoint]) + 100
            rightx_base = np.argmax(histogram[midpoint:-100]) + (midpoint)

            if leftx_current == None:
                leftx_current = leftx_base
            if rightx_current == None:
                rightx_current = rightx_base

        height = img.shape[0]
        width = img.shape[1]

        # find nonzero location in img, nonzerox, nonzeroy is the array flatted one dimension by x,y
        nonzero = img.nonzero()
        # print nonzero(행/열)
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])


        margin = 20
        minpix = 30
        left_lane_inds = []
        right_lane_inds = []
        nwindows = 15
        window_height = 10

        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = img.shape[0] - (window + 1) * window_height
            win_y_high = img.shape[0] - window * window_height

            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

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
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

            # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        lx_current = rx_current = None

        # Fit a second order polynomial to each
        if lefty != [] and leftx != []:
            left_fit = np.polyfit(lefty, leftx, 2)
            lx_current = np.int(np.polyval(left_fit, win_y_high))

        if righty != [] and righty != []:
            right_fit = np.polyfit(righty, rightx, 2)
            rx_current = np.int(np.polyval(right_fit, win_y_high))

        if lx_current != None and rx_current != None:
            x_location = (lx_current + rx_current) // 2
        else:
            # 검출하지 못했을 때 예외처리
            if lx_current == None and rx_current != None:
                x_location = rx_current - int(width * 0.22)
                print("only right")
            elif rx_current == None and lx_current != None:
                x_location = lx_current + int(width * 0.22)
                print("only left")

        # 그 이전 좌표를 가져오도록 함.
        if x_location != None:
            self.pre_xlocation = x_location
        else:
            x_location = self.pre_xlocation

        cv2.circle(out_img, (x_location, win_y_high), 5, (0, 255, 255), -1)


        return out_img, x_location