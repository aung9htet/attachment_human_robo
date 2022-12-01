#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../../')

# import approach
from attachment_sub import Attachment_Sub
from actions.approach import Approach

class ChildApproacher:

    def __init__(self):
        rospy.init_node('child_approach')
        self.miro_sub = Attachment_Sub()
        self.action = Approach()

    def run(self):
        # to explore
        while not rospy.is_shutdown():
            if self.miro_sub.controller == "approach":
                self.action.approach(human = True)      

if __name__ == "__main__":
    approach = ChildApproacher()
    approach.run()