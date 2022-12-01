#!/usr/bin/env python

import rospy
from subscribers import MiRoSub

from attachment_model.msg import Action, Care

class Attachment_Sub(MiRoSub):

    """
        Mode can either be parent or child
    """
    def __init__(self, mode = "parent"):
        super.__init__()
        self.mode = mode
        self.emotional_distance = 0
        self.physical_distance = 0
        self.need = 0
        self.care = False
        self.action = None

        # subscribe to topics
        self.action_sub = rospy.Subscriber("/" + self.mode + "/controller", Action, self.action_cb)
        self.care_sub = rospy.Subscriber(
            "/" + self.mode + "/care_detection", Care, self.care_cb
        )

    def action_cb(self, action_data):
        self.emotional_distance = action_data.emotional_distance
        self.physical_distance = action_data.physical_distance
        self.need = action_data.need
        self.action = action_data.action
    
    def care_cb(self, care_data):
        self.care = care_data.initiating
    