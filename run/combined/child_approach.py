#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../../')

# import subscription modules
from actions.approach import ChildApproach
from subscribers.action_sub import BothAction   # subscribe from one controller only
from attachment_model.msg import Care   

class ChildExploration:

    def __init__(self):
        rospy.init_node('child_approach')
        self.approach = ChildApproach()
        self.action = BothAction()
        # initialise publisher
        self.pub_sound = rospy.Publisher(
            '/child/approacher_action', Care, queue_size= 0
        )
        self.sound = Care()

    def run(self):
        # to explore
        while not rospy.is_shutdown():
            if self.action.child == 1:
                if self.approach.condition_satisfied == True:
                    self.sound.initiating = True
                    self.pub_sound.publish(self.sound)
                else:
                    self.approach.approach()
                    self.sound.initiating = False
                    self.pub_sound.publish(self.sound)
            else:
                self.approach.condition_satisfied = False
                self.sound.initiating = False
                self.pub_sound.publish(self.sound)
            print(self.sound.initiating)

if __name__ == "__main__":
    approach = ChildApproach()
    approach.run()