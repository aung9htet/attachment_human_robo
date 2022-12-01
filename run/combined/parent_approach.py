#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../../')

# import subscription modules
from actions.approach import ParentApproach
from subscribers.action_sub import BothAction   # subscribe from one controller only
from attachment_model.msg import Care

class ParentExploration:

    def __init__(self):
        rospy.init_node('parent_approach')
        self.approach = ParentApproach()
        self.action = BothAction()
        # initialise publisher
        self.pub = rospy.Publisher(
            '/parent/approacher_action', Care, queue_size= 0
        )
        self.sound = Care()

    def run(self):
        # to explore
        while not rospy.is_shutdown():
            if self.action.parent == 1:
                if self.approach.condition_satisfied == True:
                    self.sound.initiating = True
                    print(1)
                else:
                    self.approach.approach()
                    self.sound.initiating = False
                    print(2)
            else:
                self.approach.condition_satisfied = False
                self.sound.initiating = False
                print(3)
            self.pub.publish(self.sound)
    
    def still_run(self):
        while not rospy.is_shutdown():
            if self.action.parent == 1:
                self.sound.initiating = True
            else:
                self.sound.initiating = False
            self.pub.publish(self.sound)
            # add gesture to find MiRo

if __name__ == "__main__":
    approach = ParentApproach()
    approach.run()