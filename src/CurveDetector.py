#!/usr/bin/env python
#-*- coding:utf-8 -*-

import numpy as np
import time

class Curve:

    def __init__(self, N=20):
        self.angle_lst = []
        self.length = N
        self.time_old = 0
        self.curve_count = 0

    def check_time(self):
        if time.time() - self.time_old < 3:
            return False
        else:
            return True

    def update(self, angle):
        if len(self.angle_lst) >= self.length:
            self.angle_lst.pop(0)

        self.angle_lst.append(angle)

    def get_avg(self):
        avg = np.mean(self.angle_lst)

        return avg

    def is_curve(self):
        if abs(self.get_avg()) < 30 or len(self.angle_lst) < self.length or not self.check_time():
            return False

        print("Curve Detect!!!")
        return True

    def count_curve(self):
        if self.is_curve():
            self.time_old = time.time()
            self.curve_count += 1
            self.angle_lst = []

        return self.curve_count