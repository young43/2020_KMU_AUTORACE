#!/usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import time
import numpy as np
import math
from enum import Enum
from datetime import datetime

import roslib
import sys
import rospy

from rospy import ROSException
import sensor_msgs.msg
import actionlib
import rostopic
import rosservice
from rosservice import ROSServiceException

from std_msgs.msg import String
from sensor_msgs.msg import Image
from obstacle_detector.msg import Obstacles
from cv_bridge import CvBridge, CvBridgeError
from xycar_motor.msg import xycar_motor
from ar_track_alvar_msgs.msg import AlvarMarkers

from ObstacleDetector import ObstacleDetector, Position
from posecal import Pose
from pidcal import PidCal
from slidewindow import SlideWindow
from warper import Warper
from CurveDetector import Curve
from StopDetector import StopDetector
from parking import Parking

bridge = CvBridge()
cv_image = None
obstacles = None
motor_pub = None
motor_msg = None
marker = True
MODE = 2

now = datetime.now()

warper = None
slidewindow = SlideWindow()
pidcal = PidCal()

OBSTACLE_NUM = 3
obs_cnt = 0


def img_callback(data):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.resize(cv_image, dsize=(640, 480), interpolation=cv2.INTER_AREA)

    except CvBridgeError as e:
        print(e)


def motor_callback(msg):
    global motor_msg
    motor_msg = msg


def obstacle_callback(data):
    global obstacles
    obstacles = data


def get_marker(msg):
    global marker

    if len(msg.markers) != 0 and MODE == 3:
        for tag in msg.markers:
            if tag.id == 1:
                # pose = msg.pose.pose.position
                # distance = np.sqrt(pose.x**2 + pose.y ** 2)
                # print(distance)

                marker = True


def drive(Angle, Speed):
    global motor_pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    motor_pub.publish(msg)


def img_process(img):
    cols, rows, ch = img.shape
    brightness = np.sum(img) / (255 * cols * rows)

    minimum_brightness = 1
    ratio = brightness / minimum_brightness
    bright_img = cv2.convertScaleAbs(img, alpha=1 / ratio, beta=0)

    gray = cv2.cvtColor(bright_img, cv2.COLOR_BGR2GRAY)

    kernel_size = 5
    blur = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    low_threshold = 45
    high_threshold = 55
    edge = cv2.Canny(np.uint8(blur), low_threshold, high_threshold)

    roi = roi_interest(edge)

    # cv2.imshow("edge", edge)

    return roi


def roi_interest(img):
    width, height = img.shape[:2]

    vertices = np.array(
        [[
            (-300, 430),  # 좌하
            (240, 280),  # 좌상
            (420, 280),  # 우상
            (width + 300, 430)  # 우하
        ]], dtype=np.int32)

    mask = np.zeros_like(img)  # mask = img와 같은 크기의 빈 이미지

    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움
    cv2.fillPoly(mask, vertices, 255)

    # 이미지와 color로 채워진 ROI를 합침
    roi_image = cv2.bitwise_and(img, mask)

    return roi_image


def warpper_process(img):
    ret, thres_img = cv2.threshold(img, 60, 255, cv2.THRESH_BINARY)

    kernel = np.ones((3, 3), np.uint8)
    dilate = cv2.dilate(thres_img, kernel, 3)

    sharp = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharp_img = cv2.filter2D(dilate, -1, sharp)

    return thres_img


# def calc_speed(MODE, curve_detector):
#     speed = 11
#
#     if MODE == 1 or MODE == 2:  # 미션 구간(장애물, 횡단보도)
#         speed = 7.5
#     elif MODE == 3:     # 주차
#         speed = 4
#     elif curve_detector.curve_count >= 4:
#         speed = 11
#
#
#     return speed

def calc_speed(MODE, curve_detector):
    speed = 5

    if MODE == 1 or MODE == 2:  # 미션 구간(장애물, 횡단보도)
        speed = 5
    elif MODE == 3:  # 주차
        speed = 5
    elif curve_detector.curve_count >= 4:
        speed = 5

    return speed


# def avoidance(POS):
#     global obs_cnt
#
#     angle_lst = sorted(np.linspace(0, 19, 20), reverse=True)
#
#     if POS == 1:  # LEFT
#         # 우회전 / 좌회전 / 우회전 / 좌회전
#         print("1")
#         for theta in angle_lst:
#             rad = np.deg2rad(theta)
#             drive(rad, 6)
#             time.sleep(0.05)
#
#         drive(0, 0)
#         time.sleep(0.2)
#
#         print("2")
#         for theta in angle_lst:
#             rad = -np.deg2rad(theta)
#             drive(rad, 6)
#             time.sleep(0.07)
#
#         drive(0, 0)
#         time.sleep(0.2)
#
#         print("3")
#         for theta in angle_lst:
#             rad = np.deg2rad(theta)
#             drive(rad, 6)
#             time.sleep(0.07)
#
#         drive(0, 0)
#         time.sleep(0.2)
#
#         print("4")
#         for theta in angle_lst:
#             rad = -np.deg2rad(theta)
#             drive(rad, 6)
#             time.sleep(0.05)
#
#     else:  # RIGHT
#
#         print("1")
#         # 좌회전 /  우회전 / 좌회전 / 우회전
#         for theta in angle_lst:
#             rad = -np.deg2rad(theta)
#             drive(rad, 6)
#             time.sleep(0.05)
#
#         drive(0, 0)
#         time.sleep(0.2)
#
#         print("2")
#         for theta in angle_lst:
#             rad = np.deg2rad(theta)
#             drive(rad, 6)
#             time.sleep(0.07)
#
#         drive(-0.34, 0)
#         time.sleep(0.2)
#
#         print("3")
#         for theta in angle_lst:
#             rad = -np.deg2rad(theta)
#             drive(rad, 6)
#             time.sleep(0.07)
#
#         drive(0, 0)
#         time.sleep(0.2)
#
#         print("4")
#         for theta in angle_lst:
#             rad = np.deg2rad(theta)
#             drive(rad, 6)
#             time.sleep(0.03)
#
#         print("5")
#         for theta in angle_lst:
#             rad = np.deg2rad(theta)
#             drive(rad, 6)
#             time.sleep(0.03)


def avoidance(pattern, circle, POS):
    global obs_cnt

    reverse_angle = []
    # print("mid_pose:", mid_pose)

    print(circle)

    pose = Pose(0, circle.y)
    while circle.y < -0.6:
        drive(0, 4)
        time.sleep(0.05)
        _, circle.y = pose.calc_ahead(0, -6)
        print("STRAIGHT : {}".format(circle.y))

    # LEFT
    if (pattern == ["LEFT", "RIGHT", "LEFT"] and (obs_cnt == 0 or obs_cnt == 2)) or (
            pattern == ["RIGHT", "LEFT", "RIGHT"] and (obs_cnt == 1)):

        pose = Pose()
        cur_pose = pose.get_curpose()
        mid_pose = pose.get_midpoint(circle, MODE="LEFT")
        cnt = 0

        # if POS == 2:
        #     print("obstacle reverse detect!")
        #     circle.x, circle.y = -0.2, -0.8
        #     mid_pose = pose.get_midpoint(circle, MODE="LEFT")
        #     goal_pose = pose.get_goalpoint(circle)

        while cur_pose[1] > mid_pose[1]:
            y = abs(cur_pose[1] - mid_pose[1])
            x = abs(mid_pose[0] - cur_pose[0]) * 2
            angle = np.arctan2(y, x)
            reverse_angle.append(angle)

            drive(angle, 6)
            time.sleep(0.05)
            cnt += 1

            cur_pose = pose.calc_ahead(np.rad2deg(angle), 8)
            print("Left({}) : {}".format(np.rad2deg(angle), cur_pose))

        drive(0, 0)
        rospy.sleep(0.05)

        for c in range(cnt - 3):
            angle = -reverse_angle.pop(0)
            drive(angle, 4)
            time.sleep(0.1)


    elif (pattern == ["LEFT", "RIGHT", "LEFT"] and (obs_cnt == 1)) or (
            pattern == ["RIGHT", "LEFT", "RIGHT"] and (obs_cnt == 0 or obs_cnt == 2)):

        pose = Pose()
        cur_pose = pose.get_curpose()
        mid_pose = pose.get_midpoint(circle, MODE="RIGHT")
        goal_pose = pose.get_goalpoint(circle)
        cnt = 0

        # if POS == 1:
        #     print("obstacle reverse detect!")
        #     circle.x, circle.y = 0.2, -0.8
        #     mid_pose = pose.get_midpoint(circle, MODE="RIGHT")
        #     goal_pose = pose.get_goalpoint(circle)

        while cur_pose[1] > mid_pose[1]:
            y = abs(cur_pose[1] - mid_pose[1])
            x = abs(mid_pose[0] - cur_pose[0]) * 2
            angle = -np.arctan2(y, x)

            reverse_angle.append(angle)

            drive(angle, 6)
            time.sleep(0.1)
            cnt += 1

            cur_pose = pose.calc_ahead(np.rad2deg(angle), 6)
            print("Right({}) : {}".format(np.rad2deg(angle), cur_pose))

        drive(0, 0)
        rospy.sleep(0.05)

        # for c in range(cnt-3):
        #     angle = -reverse_angle.pop(0)
        #     drive(angle, 6)
        #     time.sleep(0.1)


def finish():
    print("------ xycar finish ------")


def main():
    global motor_pub
    global cv_image
    global obstacles
    global MODE
    global obs_cnt

    rospy.init_node("racecar")
    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    # motor_sub = rospy.Subscriber("xycar_motor", xycar_motor, motor_callback)
    img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size=1)
    armarker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, get_marker)

    h, w = 480, 640

    out = cv2.VideoWriter(
        '/home/nvidia/xycar_ws/src/racecar/video/slide{}{}{}.avi'.format(now.day, now.hour, now.minute),
        cv2.VideoWriter_fourcc("M", "J", "P", "G"), 30, (w, h))

    out2 = cv2.VideoWriter(
        "/home/nvidia/xycar_ws/src/racecar/video/origin{}{}{}.avi".format(now.day, now.hour, now.minute),
        cv2.VideoWriter_fourcc("M", "J", "P", "G"), 30, (w, h))

    print("------------- auto_race start!!! -------------")
    drive(0, 0)
    rospy.sleep(1)

    obstacle_detector = ObstacleDetector()
    curve_detector = Curve()
    stop_detector = StopDetector()
    parker = Parking()

    obs_time = 0
    start_time = time.time()
    cross_cnt = 0

    speed_default = 11
    speed_obstacle = 5

    first_detect = None

    x_location = None
    x_location_old = None

    stop_cnt = 0

    while not rospy.is_shutdown():
        global warper

        if cv2.waitKey(1) & 0xFF == 27:
            break

        if warper == None:
            warper = Warper(cv_image)

        process_img = img_process(cv_image)
        warp_img = warper.warp(process_img)
        process_img2 = warpper_process(warp_img)

        slideImage, x_location = slidewindow.slidewindow(process_img2)
        # curve 2번 돌고나서 obstacle
        POS, circle, distance = obstacle_detector.check(obstacles)

        # 시작점 체크
        # 횡단보도 및 시작점
        if MODE != 3 and stop_detector.check_crocss_walk(warp_img):
            if cross_cnt % 2 == 0:  # cross
                print("------ CROSS WALK DETECT ------")
                drive(0, 0)
                rospy.sleep(5.7)

                for t in range(15):
                    drive(0, 4)
                    rospy.sleep(0.1)

            else:  # start
                print("------ STOP LINE DETECT ------")
                MODE = 0
                obs_cnt = 0
                curve_detector.curve_count = 0
                start_time = time.time()
                stop_cnt += 1

            cross_cnt += 1

        # curve 2번 돌고나서 obstacle
        if MODE == 2:
            # Part1. 오왼오
            if POS.value == 1:  # 왼쪽
                for theta in range(270, 510, 8):
                    st = 0.23 * np.sin(theta * np.pi / 180)
                    drive(-st, 5)
                    print(-st)
                    time.sleep(0.06)

                obs_cnt += 1
                drive(0, 0)
                time.sleep(0.5)
            elif POS.value == 2:  # 오른쪽
                for theta in range(270, 360, 9):
                    st = 0.24 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    print(st)
                    time.sleep(0.06)

                for theta in range(360, 500, 9):
                    st = 0.19 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    print(st)
                    time.sleep(0.06)

                obs_cnt += 1
                drive(0, 0)
                time.sleep(0.5)

        if MODE == 2 and (obs_cnt == OBSTACLE_NUM or (obs_time != 0 and time.time() - obs_time > 12)):
            # Part1. 오왼오 다시 조정 코드
            for theta in range(500, 540, 9):
                st = 0.2 * np.sin(theta * np.pi / 180)
                drive(st, 5)
                print(st)
                time.sleep(0.05)


            MODE = 0
            obs_cnt = 4
            drive(0, 0)
            rospy.sleep(0.1)
            print("obstacle detect finish")

        # parking
        if MODE == 3:
            if time.time() - start_time > 14:
                print("------ parking mode on ------")
                drive(0, 0)
                time.sleep(0.5)

                parker.obs_dis = 0.43

                pose = Pose()
                cur_pose = pose.get_curpose()
                add_pose = [0, -parker.calc_add_distance()]

                while cur_pose[1] > add_pose[1]:
                    drive(0, 2.5)
                    time.sleep(0.105)
                    cur_pose = pose.calc_ahead(0, 2.5)

                pose = Pose()
                cur_pose = pose.get_curpose()
                mid_pose, goal_pose = parker.calc_drive_pose()

                print("parking: steer RIGHT!!!")
                cnt = 0
                drive(0, 0)
                time.sleep(0.1)
                while cur_pose[1] < mid_pose[1]:
                    drive(1, -2.5)
                    time.sleep(0.105)
                    cur_pose = pose.calc_behind(20, -2.5)
                    print("cur_pose:", cur_pose)
                    cnt += 1

                print("parking: steer LEFT!!")
                for t in range(cnt):
                    drive(-1, -2.5)
                    time.sleep(0.1)
                    cur_pose = pose.calc_behind(-20, -2.5)
                    print("cur_pose:", cur_pose)

                for theta in range(450, 540, 10):
                    st = 0.24 * np.sin(theta * np.pi / 180)
                    drive(st, 2.5)
                    time.sleep(0.0001)

                drive(0, 0)
                time.sleep(0.1)

                print("Straight")
                for t in range(5):
                    drive(0, 2)
                    time.sleep(0.01)

                rospy.on_shutdown(finish)
                break

        speed_default = calc_speed(MODE, curve_detector)

        if x_location != None:
            if x_location_old != None:
                if np.abs(x_location - x_location_old) < 60:
                    x_location_old = x_location
                else:
                    x_location = x_location_old

            pid = round(pidcal.pid_control(int(x_location), curve_detector.curve_count), 6)
            if abs(pid) > 0.28 and MODE != 3:
                speed_default = 10
            drive(pid, speed_default)

        else:
            x_location = x_location_old
            pid = round(pidcal.pid_control(int(x_location_old), curve_detector.curve_count), 6)
            if abs(pid) > 0.28 and MODE != 3:
                speed_default = 10
            drive(pid, speed_default)

        curve_detector.update(pid)
        curve_detector.count_curve(start_time)

        # cv2.imshow("warper", warp_img)
        # cv2.imshow("origin", re_image)
        # cv2.imshow("processImage", tempImg)

        # print(round(pid, 2), x_location)
        cv2.putText(slideImage, 'PID %f' % pid, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(slideImage, 'x_location %d' % x_location, (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                    2)
        cv2.putText(slideImage, 'curve_cnt %d' % curve_detector.curve_count, (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255),
                    2)

        cv2.putText(slideImage, 'MODE %d' % MODE, (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255),
                    2)

        cv2.line(slideImage, (x_location, 380), (318, 479), (0, 255, 255), 3)
        # cv2.imshow("slidewindow", slideImage)

        if MODE == 0 and curve_detector.curve_count == 2:
            MODE = 1  # cross_walk
        # elif MODE == 1 and obs_cnt < OBSTACLE_NUM:
        #     MODE = 2    # obstacle
        elif MODE == 0 and stop_cnt == 3:
            MODE = 3  # parking

        out.write(slideImage)
        out2.write(cv_image)

    out.release()
    out2.release()


def test():
    global motor_pub
    global cv_image
    global obstacles
    global MODE

    rospy.init_node("racecar")
    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size=1)
    # armarker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, get_marker)

    h, w = 480, 640

    out = cv2.VideoWriter(
        '/home/nvidia/xycar_ws/src/racecar/video/test_slide{}{}{}.avi'.format(now.day, now.hour, now.minute),
        cv2.VideoWriter_fourcc("M", "J", "P", "G"), 30, (w, h))

    out2 = cv2.VideoWriter(
        "/home/nvidia/xycar_ws/src/racecar/video/test_origin{}{}{}.avi".format(now.day, now.hour, now.minute),
        cv2.VideoWriter_fourcc("M", "J", "P", "G"), 30, (w, h))

    print("------------- auto_race start!!! -------------")
    drive(0, 0)
    rospy.sleep(1)

    pose = Pose()
    obstacle_detector = ObstacleDetector()
    curve_detector = Curve()
    stop_detector = StopDetector()

    speed_default = 4
    speed_obstacle = 7.5

    obs_cnt = 0
    stop_cnt = 0
    cross_cnt = 0
    cross_time = 0

    obs_time = 0

    start_time = time.time()

    x_location = None
    x_location_old = None

    first_detect = None

    while not rospy.is_shutdown():
        global warper

        if cv2.waitKey(1) & 0xFF == 27:
            break

        if warper == None:
            warper = Warper(cv_image)

        re_image = cv2.resize(cv_image, dsize=(640, 480), interpolation=cv2.INTER_AREA)
        process_img = img_process(re_image)
        warp_img = warper.warp(process_img)
        process_img2 = warpper_process(warp_img)

        slideImage, x_location = slidewindow.slidewindow(process_img2)
        # curve 2번 돌고나서 obstacle
        POS, circle, distance = obstacle_detector.check(obstacles)

        if MODE == 2:
            # Part1. 오왼오
            if POS.value == 1:  # 왼쪽
                for theta in range(270, 510, 8):
                    st = 0.23 * np.sin(theta * np.pi / 180)
                    drive(-st, 5)
                    print(-st)
                    time.sleep(0.06)

                obs_cnt += 1
                drive(0, 0)
                time.sleep(0.5)
            elif POS.value == 2:  # 오른쪽
                for theta in range(270, 360, 9):
                    st = 0.24 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    print(st)
                    time.sleep(0.06)

                for theta in range(360, 500, 9):
                    st = 0.19 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    print(st)
                    time.sleep(0.06)

                obs_cnt += 1
                drive(0, 0)
                time.sleep(0.5)



        if MODE == 2 and (obs_cnt == OBSTACLE_NUM or (obs_time != 0 and time.time() - obs_time > 12)):
            # Part1. 오왼오 다시 조정 코드
            for theta in range(500, 540, 9):
                st = 0.2 * np.sin(theta * np.pi / 180)
                drive(st, 5)
                print(st)
                time.sleep(0.05)

            MODE = 0
            obs_cnt = 4
            drive(0, 0)
            rospy.sleep(0.1)
            print("obstacle detect finish")

        # 시작점 체크
        # if stop_detector.check_yellow_line(warp_img):
        #     MODE = 0
        #     obs_cnt = 0
        #     curve_detector.curve_count = 0
        #     start_time = time.time()
        #     stop_cnt = 0

        # 횡단보도 및 시작점
        if stop_detector.check_crocss_walk(warp_img):
            if cross_cnt % 2 == 0:  # cross
                print("------ CROSS WALK DETECT ------")
                drive(0, 0)
                rospy.sleep(5)

                for t in range(15):
                    drive(0, 4)
                    rospy.sleep(0.1)

            else:  # start
                print("------ STOP LINE DETECT ------")
                MODE = 0
                obs_cnt = 0
                curve_detector.curve_count = 0
                start_time = time.time()
                stop_cnt += 1

            cross_cnt += 1

        speed_default = calc_speed(MODE, curve_detector)

        if x_location != None:
            if x_location_old != None:
                if np.abs(x_location - x_location_old) < 60:
                    x_location_old = x_location
                else:
                    x_location = x_location_old

            pid = round(pidcal.pid_control(int(x_location), curve_detector.curve_count), 6)
            if abs(pid) > 0.28 and MODE != 3:
                speed_default = 5
            drive(pid, speed_default)

        else:
            x_location = x_location_old
            pid = round(pidcal.pid_control(int(x_location_old), curve_detector.curve_count), 6)
            if abs(pid) > 0.28 and MODE != 3:
                speed_default = 5
            drive(pid, speed_default)

        curve_detector.update(pid)
        curve_detector.count_curve(start_time)

        # print(round(pid, 2), x_location)

        cv2.putText(slideImage, 'PID %f' % pid, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(slideImage, 'x_location %d' % x_location, (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                    2)
        cv2.putText(slideImage, 'curve_cnt %d' % curve_detector.curve_count, (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255),
                    2)

        cv2.putText(slideImage, 'MODE %d' % MODE, (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)

        # cv2.line(slideImage, (x_location, 380), (318, 479), (0, 255, 255), 3)
        # cv2.imshow("slidewindow", slideImage)

        if MODE == 0 and curve_detector.curve_count == 2:
            MODE = 1  # cross_walk
        elif MODE == 1 and obs_cnt < OBSTACLE_NUM:
            MODE = 2  # obstacle
        elif MODE == 0 and stop_cnt == 3:
            MODE = 3  # parking

        # out.write(slideImage)
        # out2.write(re_image)

    # out.release()
    # out2.release()


def test_avoid():
    global motor_pub
    rospy.init_node("racecar")
    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)

    avoidance(2)


if __name__ == "__main__":
    # main()
    test()
    # test_avoid()




