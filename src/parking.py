#!/usr/bin/env python
#-*- coding:utf-8 -*-

import math
from posecal import Pose

class Parking:
    def __init__(self):
        self.car_width = 0.4
        self.parking_space = 1    # 실제보다 안전거리만큼 값을 더 주기
        self.obs_dis = 0.3      # temp

    def check_wall(self, obstacles):
        for circle in obstacles.circles:
            p = circle.center

            if -0.3 < p.y <= 0:
                if 0.15 < p.x < 0.65:
                    self.obs_dis = p.x
                    return True

        return False

    def calc_add_distance(self):
        res = math.pow((self.obs_dis + self.car_width), 2) / (self.parking_space)
        return res

    def calc_add_time(self, speed):
        add_dis = self.calc_add_distance()
        time = add_dis / (speed * 0.1)
        return time

    def calc_drive_pose(self):
        all_path_y = (self.parking_space + self.calc_add_distance())
        all_path_x = self.car_width + self.obs_dis
        mid_pose = [all_path_x/2, all_path_y/2]
        goal_pose = [all_path_x, all_path_y]

        return mid_pose, goal_pose


if __name__ == "__main__":
    # import rospy
    # from obstacle_detector.msg import Obstacles
    # from xycar_motor.msg import xycar_motor
    #
    # from posecal import Pose
    #
    # def obstacle_callback(data):
    #     global obstacles
    #     obstacles = data
    #
    #
    # def drive(Angle, Speed):
    #     global motor_pub
    #
    #     msg = xycar_motor()
    #     msg.angle = Angle
    #     msg.speed = Speed
    #
    #     motor_pub.publish(msg)
    #
    #
    # # test main
    # rospy.init_node("parking_test")
    # obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size=1)
    # motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    #
    # parker = Parking()
    #
    # print("------ parking mode on ------")
    # if parker.check_wall(obstacles):
    #     add_time = parker.calc_add_time(2)
    #     for t in range(add_time):
    #         drive(0, 2)
    #         time.sleep(1)
    #
    #     pose = Pose()
    #     cur_pose = pose.get_curpose()
    #     mid_pose, goal_pose = parker.calc_drive_pose()
    #
    #     print("parking: forward!!!")
    #     while cur_pose[1] < mid_pose[1]:
    #         drive(0.34, -2)
    #         time.sleep(0.1)
    #         cur_pose = pose.calc_behind(20, -2)
    #
    #     print("parking: backward!!!")
    #     while cur_pose[1] < goal_pose[1]:
    #         drive(-0.34, -2)
    #         time.sleep(0.1)
    #         cur_pose = pose.calc_behind(-20, -2)




    parker = Parking()
    pose = Pose()
    cur_pose = pose.get_curpose()
    mid_pose, goal_pose = parker.calc_drive_pose()

    print(parker.calc_add_distance(), math.ceil(parker.calc_add_time(2)))
    print(mid_pose)
    print(goal_pose)


