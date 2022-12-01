#!/usr/bin/env python

import os
import rospy
import numpy as np

# import messages
from std_msgs.msg import UInt32MultiArray, Float32MultiArray
from geometry_msgs.msg import TwistStamped

# import wheel speed conversion
try:
    from miro2.lib import wheel_speed2cmd_vel
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel
    
class MiRoPub(object):

    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")    # topic base name

        # publish illumination of body
        self.color_pub = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )

        # control cosmetic joints
        self.cos_joint_pub = rospy.Publisher(
            topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )

        # control movement
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

    """
        Set color format in proper form
    """
    def convert_color(self, red = 0.0, green = 0.0, blue = 0.0):
        self.color_detail = (int(red), int(green), int(blue))
        color = '0xFF%02x%02x%02x'%self.color_detail
        color = int(color, 16)
        return color

    """
        Change color
    """
    def set_color_cmd(self, color1, color2, color3, color4, color5, color6):
        self.color_change = UInt32MultiArray()
        self.color_change.data = [color1,
            color2,
            color3,
            color4,
            color5,
            color6,
        ]
        self.color_pub.publish(self.color_change)

    """
        Changes the position of the cosmetic joints
            droop => head height
            wag => tail position
            eyel => eye lids left
            eyer => eye lids right
            earl => ear twisting left
            earr => ear twisting right
    """
    def set_cos_joint(self, droop=0.0, wag=0.0, eyel=0.0, eyer=0.0, earl=0.0, earr=0.0):
        wag_tail = Float32MultiArray()
        wag_detail = [droop, wag, eyel, eyer, earl, earr]
        wag_tail.data = wag_detail
        self.cos_joint_pub.publish(wag_tail)

    """
        Changes the speed at the robot should travel
            linear => speed moving forward/backward
            angular => angular velocity
    """
    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        vel_cmd = TwistStamped()
        vel_cmd.twist.linear.x = linear
        vel_cmd.twist.angular.z = angular
        self.vel_pub.publish(vel_cmd)

    """
        Changes the speed through the use of each wheel
    """
    def drive(self, speed_l=0.1, speed_r=0.1):
        vel_cmd = TwistStamped()
        wheel_speed = [speed_l, speed_r]
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)
        vel_cmd.twist.linear.x = dr
        vel_cmd.twist.angular.z = dtheta 
        self.vel_pub.publish(vel_cmd)