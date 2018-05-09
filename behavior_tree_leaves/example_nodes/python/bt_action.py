#! /usr/bin/env python
# Author: Anas Abou Allaban

import rospy
import actionlib
import behavior_tree_core.msg
from abc import ABCMeta, abstractmethod


class BTAction:
    __metaclass__ = ABCMeta
    _feedback = behavior_tree_core.msg.BTFeedback()
    _result = behavior_tree_core.msg.BTResult()

    def __init__(self, name):
        # Behavior tree action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_tree_core.msg.BTAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("{} Server Started".format(self._action_name))

    @abstractmethod
    def execute_cb(self, goal):
        raise NotImplementedError()

    def set_status(self, status):
        if status:
            self._feedback.status = 1
            self._result.status = self._feedback.status
            rospy.loginfo('Action %s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            self._feedback.status = 2
            self._result.status = self._feedback.status
            rospy.loginfo('Action %s: Failed' % self._action_name)
            self._as.set_succeeded(self._result)
