#!/usr/bin/env python3

# import core modules
import os
import time

# import functionalities
from attachment_pub import Attachment_Pub
from attachment_sub import Attachment_Sub
from actions.apriltag_perception import AprilTagPerception

SPEED = 0.5
MIN_RANGE = 0.35

class MiRoApproach(object):
    def __init__(self, mode = "parent"):

        # set to parent or child
        self.mode = mode

        # set variables
        self.tag_size = 1
        self.tag_found = False              # check for tags
        self.condition_satisfied = False    # check if MiRo is near tag
        
        # import required subscribers and modules
        self.miro_data = Attachment_Sub()
        self.miro_pub = Attachment_Pub()
        self.april_tag = AprilTagPerception(size=self.tag_size, family='tag36h11')

        # required variables for camera and tag
        self.input_camera = self.miro_data.input_camera
        self.new_frame = self.miro_data.new_frame
        self.range = self.miro_data.range
        self.tag = [None, None]
        self.start_time = time.time()
        self.current_time = time.time() - self.start_time
        self.condition_satisfied = False

    """
        Update camera, frame and range
    """
    def update_variables(self):
        self.input_camera = self.miro_data.input_camera
        self.new_frame = self.miro_data.new_frame
        self.range = self.miro_data.range
        self.current_time = time.time() - self.start_time

    """
        April tag detection
    """
    def lookForTag(self, set_id = None):
        
        # detect the tags and identify their id
        for index in range(2):
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            self.tag[index] = self.april_tag.detect_tags(image)
        
        # check all tag if None
        if set_id is None:
            if not self.tag[0] or not self.tag[1]:
                if ((self.current_time % 2) < 0.2):
                    self.miro_pub.drive(SPEED, -SPEED)
                    self.tag_found = False
            else:
                self.tag_found = True
        else:
            # if tag is not found and the tag id is not valid for left and right eye
            if not (self.tag[0] and ((self.tag[0][0].id in set_id) or (self.tag[1][0].id in set_id))):
                self.miro_pub.drive(SPEED, -SPEED)
                self.tag_found = False
            else:
                self.tag_found = True

    """
        how much interaction is made
    """
    def care_touch(self):
        touch_care_amount = self.miro_data.touch_body.count('1') + self.miro_data.touch_head.count('1')
        return touch_care_amount

    """
        Approach april tag on detection
    """
    def approach(self, set_id = None, human = False):
        if not human:
            self.update_variables()
            self.lookForTag(set_id)
            if not self.condition_satisfied:
                if self.tag_found == True:
                    if self.tag[0] and self.tag[1]:
                        self.miro_pub.drive(SPEED, SPEED)
                    # if in left camera turn left
                    elif self.tag[0] and not self.tag[1]:
                        self.miro_pub.drive(0, SPEED)
                    # if in right camera turn right
                    elif not self.tag[0] and  self.tag[1]:
                        self.miro_pub.drive(SPEED, 0)
                    if self.range < MIN_RANGE:
                        self.condition_satisfied = True
        else:
            if self.care_touch < 4:
                print("YES")
            else:
                print("NO")