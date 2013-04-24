#!/usr/bin/env python

import sys

import roslib
roslib.load_manifest('raw_vfk_dummy')

import rospy
import actionlib
from tf.transformations import *
from geometry_msgs.msg import *
from raw_vfk_dummy.srv import *
from raw_vfk_dummy.msg import *

class DeliverBoxesServer:
	def __init__(self):
		# initialize services
		rospy.Service('get_pose', GetPose, self.get_pose_callback)
		
		# initialize actions
		self.server = actionlib.SimpleActionServer('deliver_boxes', DeliverBoxesAction, self.deliver_boxes_callback, False)
		self.server.start()
		
		# globals
		self.standby_pose = PoseStamped()
		self.standby_pose.pose.position.x = 1.11
		self.standby_pose.pose.position.y = 2.22
		q = quaternion_from_euler(0, 0, 0) # transform from roll pitch yaw to quaternions
		self.standby_pose.pose.orientation.x = q[0]
		self.standby_pose.pose.orientation.y = q[1]
		self.standby_pose.pose.orientation.z = q[2]
		self.standby_pose.pose.orientation.w = q[3]
		
		self.current_robot_pose = self.standby_pose

	# deliver boxes action
	def deliver_boxes_callback(self, goal):
		rospy.loginfo("deliver_boxes called with box_ids %s.",str(goal.box_ids))
		
		# check a valid incoming goal
		# * only accept goals with at least one box_id 
		# * only accept box_ids < 10
		if len(goal.box_ids) < 1:
			self.server.set_aborted(res)
			rospy.logerr("not accepting goal with no box_ids")
			return
		for box_id in goal.box_ids:
			if box_id >=10:
				self.server.set_aborted(res)
				rospy.logerr("not accepting goal with box_ids >= 10.")
				return
		
		# process boxes
		res = DeliverBoxesResult()
		for box_id in goal.box_ids:
			# check if action is ctrl-c'ed
			if rospy.is_shutdown():
				sys.exit(1)
			
			# check if action got canceled
			if self.server.is_preempt_requested():
				rospy.loginfo("action was canceled")
				self.server.set_preempted(res)
				return
			
			rospy.loginfo("working on box %d",box_id)
			
			# return feedback
			feedback = DeliverBoxesFeedback()
			feedback.current_box_id = box_id
			feedback.percent_complete = 100.0 * len(res.deliveries) / len(goal.box_ids)
			self.server.publish_feedback(feedback)
			
			# wait for the box to be delivered
			rospy.sleep(3)
			self.current_robot_pose = goal.search_pose # robot is at search_pose
			print "robot at search_pose"
			rospy.sleep(3)
			self.current_robot_pose = goal.target_pose # robot is at target_pose
			print "robot at target_pose"
			rospy.sleep(3)
			
			# store result
			delivery = Delivery()
			delivery.box_id = box_id
			delivery.pose = goal.target_pose 
			res.deliveries.append(delivery)
			
			# return feedback
			feedback = DeliverBoxesFeedback()
			feedback.current_box_id = box_id
			feedback.percent_complete = 100.0 * len(res.deliveries) / len(goal.box_ids)
			self.server.publish_feedback(feedback)

		# action finished
		self.current_robot_pose = self.standby_pose
		print "robot at standy_pose"
		self.server.set_succeeded(res)
		return 

	# get pose service
	def get_pose_callback(self,req):
		rospy.loginfo("get_pose called")
		
		# fill response message
		res = GetPoseResponse()
		res.pose = self.current_robot_pose
	
		# return response
		return res

if __name__ == "__main__":
	# initialize node
	rospy.init_node('raw_vfk_dummy')

	# Dinitialize DeliverBoxes Server
	DeliverBoxesServer()
	
	# calling spin() to wait for callbacks
	rospy.loginfo("Dummy interface running.")
	rospy.spin()
