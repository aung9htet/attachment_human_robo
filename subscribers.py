#!/usr/bin/env python

import rospy
import os
import cv2
from math import degrees
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError

# import messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Range
from std_msgs.msg import UInt16

class MiRoSub(object):
    
    def __init__(self):

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # for odom
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0

        # for range
        self.field_of_view = 0.0
        self.min_range = 0.0
        self.max_range = 0.0
        self.range = 0.0

        # for camera
        self.input_camera = [None, None]
        self.new_frame = [False, False]
        self.image_convert = CvBridge()
        
        # for touch
        self.touch_body = '00000000000000'
        self.touch_head = '00000000000000'

        # subscribe to topics
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.odom_subscriber = rospy.Subscriber(topic_base_name + "/sensors/odom", Odometry, self.odom_cb)
        self.range_subscriber = rospy.Subscriber(topic_base_name + "/sensors/sonar", Range, self.range_cb)
        self.sub_caml = rospy.Subscriber(
            topic_base_name + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_caml,
            queue_size = 1,
            tcp_nodelay = True
        )
        self.sub_camr = rospy.Subscriber(
            topic_base_name + "/sensors/camr/compressed",
            CompressedImage,
            self.callback_camr,
            queue_size = 1,
            tcp_nodelay = True
        )
        self.sub_touch_body = rospy.Subscriber(
            topic_base_name + "/sensors/touch_body",
            UInt16,
            self.callback_touch_body("body")
        )
        self.sub_touch_head = rospy.Subscriber(
            topic_base_name + "/sensors/touch_head",
            UInt16,
            self.callback_touch_head("head")
        )

    """
        Odom callback
    """
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        if yaw<0:
            self.yaw = self.round(degrees(yaw), 4)+360
        else:
            self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

    """
        Range callback
    """
    def range_cb(self, odom_data):
        self.field_of_view = odom_data.field_of_view
        self.min_range = odom_data.min_range
        self.max_range = odom_data.max_range
        self.range = odom_data.range

    def callback_caml(self, ros_image):
        self.callback_cam(ros_image, 0)
    
    def callback_camr(self, ros_image):
        self.callback_cam(ros_image, 1)

    """
        Camera callback
    """
    def callback_cam(self, ros_image, index):
        try:
            image = self.image_convert.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.input_camera[index] = image
            self.frame_height, self.frame_width, channels = image.shape
            self.x_centre = self.frame_width / 2.0
            self.y_centre = self.frame_height / 2.0
            self.new_frame[index] = True
        except CvBridgeError as e:
            pass

    """
        Touch Body Callback
    """
    def callback_touch_body(self, touch_data):
        self.touch_body = f'{touch_data.data:014b}'
    
    """
        Touch Head Callback
    """
    def callback_touch_head(self, touch_data):
        self.touch_head = f'{touch_data.data:014b}'
    
    """
        Rounding floats
    """
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)