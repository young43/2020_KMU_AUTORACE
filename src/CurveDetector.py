#!/usr/bin/env python
#-*- coding:utf-8 -*-

import numpy as np
import time

class Curve:

    def __init__(self):
        self.time_old = 0
        self.curve_count = 0
        self.pid_list = [0.0 for i in range(30)]

    def check_time(self):
        if time.time() - self.time_old < 3:
            return False
        else:
            return True

    def update(self, pid):
        self.pid_list.pop(0)
        self.pid_list.append(pid)


    def count_curve(self):
        if self.check_time():
            if abs(sum(self.pid_list)) > 2:
                self.time_old = time.time()
                self.curve_count += 1
                print("Curve Detect!!!", self.curve_count)


    def get_cnt(self):
        return self.curve_count