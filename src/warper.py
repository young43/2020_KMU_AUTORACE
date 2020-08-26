import cv2
import numpy as np


class Warper:
    def __init__(self, image=None):
        height = image.shape[0]
        width = image.shape[1]

        h = image.shape[0]
        w = image.shape[1]
        print("h : ", height)
        print("w : ", width)

        # distort scr to dst
        # src = np.float32([
        #     [w * 1.6, h * 1.3],     # 우하
        #     [w * (-0.1), h * 1.3],  # 좌하
        #     [0, h * 0.62],          # 좌상
        #     [w, h * 0.62],          # 우상
        # ])
        # dst = np.float32([
        #     [w * 0.65, h * 0.98],
        #     [w * 0.35, h * 0.98],
        #     [w * (-0.3), 0],
        #     [w * 1.3, 0],
        # ])

        src = np.float32([
            [0, 320],  # 좌상
            [0, 410],  # 좌하
            [width, 320],  # 우상
            [width, 410],  # 우하
        ])
        dst = np.float32([
            [-200, 0],
            [150, height],
            [width + 100, 0],
            [width - 200, height],
        ])

        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)

    def warp(self, img):
        return cv2.warpPerspective(
            img,
            self.M,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )

    def unwarp(self, img):
        return cv2.warpPerspective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )