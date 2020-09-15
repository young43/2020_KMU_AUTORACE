
import numpy as np
import math
import cv2
import time

x = -0.1
y = -0.2
bottom = np.sqrt(x*x + y*y)
x = x/bottom
y = y/bottom

print(math.degrees(math.acos(x)))

print(math.acos(x) * 180 / np.pi)
# print(-np.arccos(x) * 180 / np.pi)

