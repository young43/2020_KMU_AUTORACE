#!/usr/bin/env python
#-*- coding:utf-8 -*-

import math
import time
import numpy as np

from posecal import Pose




class Parking:
    def __init__(self):
        self.car_width = 0.4
        self.parking_space = 1    # 실제보다 안전거리만큼 값을 더 주기
        self.obs_dis = 0.40         # temp

    def check_wall(self, obstacles):
        if obstacles == None:
            return False

        for circle in obstacles.circles:
            p = circle.center

            if -0.3 < p.y < 0.1:
                if 0.15 < p.x < 0.7:
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
        all_path_y = (self.parking_space-0.3 + self.calc_add_distance())
        all_path_x = self.car_width + self.obs_dis
        mid_pose = [all_path_x/2, all_path_y/2]
        goal_pose = [all_path_x, all_path_y]

        return mid_pose, goal_pose



if __name__ == "__main__":
    import rospy
    from obstacle_detector.msg import Obstacles
    from xycar_motor.msg import xycar_motor
    from ar_track_alvar_msgs.msg import AlvarMarkers
    from tf.transformations import quaternion_from_euler, euler_from_quaternion
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import Image

    from posecal import Pose

    obstacles = None
    motor_pub = None
    yaw_data = None
    distance = None

    bridge = CvBridge()
    cv_image = None

    def obstacle_callback(data):
        global obstacles
        obstacles = data


    def drive(Angle, Speed):
        global motor_pub

        msg = xycar_motor()
        msg.angle = Angle
        msg.speed = Speed

        motor_pub.publish(msg)


    def get_marker(msg):
        global yaw_data
        global distance
        if len(msg.markers) != 0:
            for tag in msg.markers:
                if tag.id == 1:
                    orientation = tag.pose.pose.orientation
                    ori_lst = [orientation.x, orientation.y, orientation.z, orientation.w]
                    position = tag.pose.pose.position

                    roll, pitch, yaw_data = euler_from_quaternion(ori_lst)
                    distance = np.sqrt(position.x**2 + position.y**2 + position.z**2)

                    # pose = msg.pose.pose.position
                    # distance = np.sqrt(pose.x**2 + pose.y ** 2)
                    # print(distance)


    def img_callback(data):
        global cv_image
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    # test main
    rospy.init_node("parking_test")
    obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size=1)
    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    armarker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, get_marker)
    img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    parker = Parking()

    drive(0, 0)
    time.sleep(3)
    while not rospy.is_shutdown():
        print(yaw_data, distance)
        print("------ parking mode on ------")
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
            time.sleep(0.096)
            cur_pose = pose.calc_behind(20, -2.5)
            print("cur_pose:", cur_pose)
            cnt += 1

        print("parking: steer LEFT!!")
        print("goal:", goal_pose)
        for t in range(cnt):
            drive(-1, -2.5)
            time.sleep(0.096)
            cur_pose = pose.calc_behind(-20, -2.5)
            print("cur_pose:", cur_pose)


        drive(0, 0)
        time.sleep(0.1)


        while True:
            if distance == None and yaw_data == None:
                # drive(0, 0)
                # time.sleep(0.1)
                continue

            print(yaw_data, distance)

            if distance < 0.29:
                break

            drive(yaw_data*3, 2)
            time.sleep(0.5)

            time.sleep(1)

        break


        #
        # print("Straight")
        # for t in range(11):
        #     drive(0, 2)
        #     time.sleep(0.03)
        #
        # break




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


