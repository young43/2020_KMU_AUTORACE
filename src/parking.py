#!/usr/bin/env python
#-*- coding:utf-8 -*-

import math
from posecal import Pose

class Parking:
    def __init__(self):
        self.car_width = 0.45
        self.parking_space = 1.6
        self.obs_dis = None
        self.obstacles = obstacles


    def check_wall(self, obstacles):
        for circle in obstacles.circles:
            p = circle.center

            if -0.3 < p.y <= 0:
                if 0.15 < p.x < 0.5:
                    self.obs_dis = p.x
                    return True

        return False

    def calc_add_distance(self):
        res = math.pow((self.obs_dis + self.car_width), 2) / self.parking_space
        return res

    def calc_add_time(self, speed):
        add_dis = self.calc_add_distance()
        time = add_dis / (speed * 0.1)
        return time

    def calc_drive_pose(self):
        all_path_y = (self.parking_space + self.calc_add_distance())
        all_path_x = self.car_width + self.obs_dis
        mid_pose = [all_path_x//2, all_path_y//2]
        goal_pose = [all_path_x, all_path_y]

        return mid_pose, goal_pose









