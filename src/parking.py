#!/usr/bin/env python
#-*- coding:utf-8 -*-

import math
import time
import numpy as np
from posecal import Pose

class Parking:
    def __init__(self):
        self.car_width = 0.4
        self.parking_space = 1.2    # 실제보다 안전거리만큼 값을 더 주기
        self.obs_dis = 0.0      # temp

    def check_wall(self, obstacles):
        if obstacles == None:
            return False

        for circle in obstacles.circles:
            p = circle.center

            if -0.2 < p.y < 0.1:
                if 0.2 < p.x < 0.7:
                    self.obs_dis = p.x - 0.1
                    if p.x > 0.7:
                        self.obs_dis -= 0.15
                    elif p.x > 0.6:
                        self.obs_dis -= 0.12
                    elif p.x > 0.5:
                        self.obs_dis -= 0.1
                    print("obstacle:", p.x, p.y, self.obs_dis)
                    return True

        return False


    def check_final_wall(self, obstacles):
        if obstacles == None:
            return None

        for circle in obstacles.circles:
            p = circle.center
            if -1 < p.y < -0.1 and abs(p.x) < 1.5:
                return abs(p.y)

        return None

    def calc_add_distance(self):
        # res = math.pow((self.obs_dis + self.car_width), 2) / (self.parking_space)
        # res = (self.obs_dis + self.car_width/1.8) / ((self.car_width+0.4+self.obs_dis)/self.parking_space)
        obs_dis = self.obs_dis #- 0.05
        res = math.pow((obs_dis + self.car_width), 2) / (self.parking_space)
        # res = (obs_dis + self.car_width) / ((self.car_width//2 + 0.5 + obs_dis) / self.parking_space) - 0.5

        # res = (self.parking_space+0.6) - (0.4 + self.obs_dis)/np.tan(40 * np.pi / 180)
        return res

    def calc_add_time(self, speed):
        add_dis = self.calc_add_distance()
        time = add_dis / (speed * 0.1)
        return time

    def calc_drive_pose(self):
        all_path_y = (self.parking_space-0.35 + self.calc_add_distance())
        all_path_x = self.car_width + self.obs_dis
        mid_pose = [all_path_x/2, all_path_y/2]
        goal_pose = [all_path_x, all_path_y]

        return mid_pose, goal_pose


if __name__ == "__main__":
    import rospy
    from obstacle_detector.msg import Obstacles
    from xycar_motor.msg import xycar_motor

    from posecal import Pose

    obstacles = None
    motor_pub = None

    def obstacle_callback(data):
        global obstacles
        obstacles = data


    def drive(Angle, Speed):
        global motor_pub

        msg = xycar_motor()
        msg.angle = Angle
        msg.speed = Speed

        motor_pub.publish(msg)


    # test main
    rospy.init_node("parking_test")
    obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size=1)
    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)

    parker = Parking()


    while not rospy.is_shutdown():
        drive(0, 2.5)
        time.sleep(0.1)

        if parker.check_wall(obstacles):
            print("------ parking mode on ------")
            drive(0, 0)
            time.sleep(0.5)
            add_time = int(math.ceil(parker.calc_add_time(2)))
            # print("obs_distance:", parker.obs_dis)
            print("add_distance:", -parker.calc_add_distance(), add_time)

            pose = Pose()
            cur_pose = pose.get_curpose()
            add_pose = [0, -parker.calc_add_distance()]

            while cur_pose[1] > add_pose[1]:
                drive(0, 2.5)
                time.sleep(0.1)
                cur_pose = pose.calc_ahead(0, 2.5)


            pose = Pose()
            cur_pose = pose.get_curpose()
            mid_pose, goal_pose = parker.calc_drive_pose()

            print("parking: steer RIGHT!!!")
            print("mid:", mid_pose)
            cnt = 0
            drive(0, 0)
            time.sleep(0.1)
            while cur_pose[1] < mid_pose[1]:
                drive(1, -2.5)
                time.sleep(0.1)
                cur_pose = pose.calc_behind(20, -2.5)
                print("cur_pose:", cur_pose)
                cnt += 1


            print("parking: steer LEFT!!")
            print("goal:", goal_pose)
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
            for t in range(6):
                drive(0, 2)
                time.sleep(0.03)

            break


    # test log
    # parker = Parking()
    # pose = Pose()
    # cur_pose = pose.get_curpose()
    # mid_pose, goal_pose = parker.calc_drive_pose()
    #
    # print(parker.calc_add_distance(), math.ceil(parker.calc_add_time(2)))
    # print("mid:", mid_pose)
    # while cur_pose[1] < mid_pose[1]:
    #     time.sleep(0.1)
    #     cur_pose = pose.calc_behind(20, -2)
    #     print(cur_pose)
    #
    # print("goal:", goal_pose)
    # while cur_pose[1] < goal_pose[1]:
    #     time.sleep(0.1)
    #     cur_pose = pose.calc_behind(-20, -2)
    #     print(cur_pose)

