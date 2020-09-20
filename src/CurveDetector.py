#!/usr/bin/env python
#-*- coding:utf-8 -*-

import numpy as np
import time

class Curve:

    def __init__(self):
        self.time_old = 0
        self.curve_count = 0
        self.pid_list = [0.0 for i in range(20)]

    def check_time(self):
        if time.time() - self.time_old < 2:
            return False
        else:
            return True

    def update(self, pid):
        if len(self.pid_list) >= 20:
            self.pid_list.pop(0)
            self.pid_list.append(pid)


    def count_curve(self):
        if self.is_curve():
            self.time_old = time.time()
            self.curve_count += 1
            return True
        return False

    def is_curve(self):
        if self.check_time():
            if abs(sum(self.pid_list)) > 2:
                print("Curve Detect!!!", self.curve_count)
                return True
        return False


    def get_cnt(self):
        return self.curve_count

