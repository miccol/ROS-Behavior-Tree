#! /usr/bin/env python
# Author: Anas Abou Allaban

import rospy
from bt_action import BTAction


class Start(BTAction):

    def __init__(self):
        self.condition = true
        # Behavior tree action server
        super(Start, self).__init__('Start')

    def execute_cb(self, goal):
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo("{} halted".format(self._action_name))
                self._as.set_preempted()
                self.set_status(False)
                break
            if condition:
                self.set_status(True)
            else:
                self.set_status(False)
