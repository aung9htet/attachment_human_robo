#!/usr/bin/env python3

# import core modules
import os
import rospy
import numpy as np
import time

# import functionalities
from attachment_pub import Attachment_Pub
from attachment_sub import Attachment_Sub

SPEED = 0.5
MIN_WALL = 0.5

class MiRoExplore(object):

    def __init__(self, mode = "parent"):

        self.mode = mode
        
        # initialise node for publishing and odom message
        self.miro_pub = Attachment_Pub()
        self.miro_sub = Attachment_Sub()

        # set robot init
        self.wall_distance = self.miro_sub.range
        self.explore_robot = True
        check_dir = np.random.randint(1, 3)
        if check_dir == 1:
            self.angle_turn = (np.pi/2)/3 * 8
        else:
            self.angle_turn = -(np.pi/2)/3 * 8
        self.rand_turn = 0

        # record start time of MiRo node
        self.start_time = time.time()

    """
        Check whether the robot should turn or go straight
    """
    def check_turn(self):
        time_elasped = int(time.time() - self.start_time)
        # check if robot is in the middle of turning
        if self.explore_robot == False:
            if ((time_elasped % 2) == 0):
                self.explore_robot = True
            print("turning", self.miro_sub.range)
            return True
        else:
            # check if the wall is too near
            if self.miro_sub.range < MIN_WALL:
                self.start_turn = rospy.get_rostime()
                self.explore_robot = False
                print("wall ahead", self.miro_sub.range)
                return True
            else:
                print("going straight")
                return False

    """
        Carry out exploration function
    """
    def explore_action(self):
        time_elasped = time.time() - self.start_time
        if self.check_turn() == False:
            if ((time_elasped % 2) < 0.1):
                max_angle_turn = np.pi/2 # 90 degrees max angle
                self.rand_turn = np.random.randint(-np.ceil(max_angle_turn * 10), np.ceil(max_angle_turn * 10))/10
            self.miro_pub.set_move_cmd(linear=SPEED, angular=self.rand_turn)
        else:
            check_dir = np.random.randint(1, 3)
            if ((time_elasped % 4) == 0):
                if check_dir == 1:
                    self.angle_turn = (np.pi/2)/3 * 8
                else:
                    self.angle_turn = -(np.pi/2)/3 * 8
            self.miro_pub.set_move_cmd(linear=0, angular=self.angle_turn * 2)