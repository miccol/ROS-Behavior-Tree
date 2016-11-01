#! /usr/bin/env python


import rospy

import actionlib

import behavior_tree_core.msg




class BTAction(object):
  # create messages that are used to publish feedback/result
  _feedback = behavior_tree_core.msg.BTFeedback()
  _result   = behavior_tree_core.msg.BTResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, behavior_tree_core.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
		  def execute_cb(self, goal):
		    # publish info to the console for the user
		    rospy.loginfo('Starting Action')
		    
		    # start executing the action
		    while #your condition:
		      # check that preempt has not been requested by the client
		      if self._as.is_preempt_requested():
			#HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
			rospy.loginfo('Action Halted')
			self._as.set_preempted()
			success = False
			break

		      rospy.loginfo('Executing Action')      
		      #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
		      
		      #IF THE ACTION HAS SUCCEEDED
		      self.set_status('SUCCESS')
		      #IF THE ACTION HAS FAILED
		      self.set_status('FAILURE')

    	

  def set_status(self,status):
      if status == 'SUCCESS':
        self._feedback.status = 1
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
      elif status == 'FAILURE':
        self._feedback.status = 2
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Failed' % self._action_name)
        self._as.set_succeeded(self._result)
      else:
        rospy.logerr('Action %s: has a wrong return status' % self._action_name)



if __name__ == '__main__':
  rospy.init_node(#NAME OF YOUR ACTION)
  BTAction(rospy.get_name())
  rospy.spin()
