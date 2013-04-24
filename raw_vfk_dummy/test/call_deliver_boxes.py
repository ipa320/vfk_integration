#!/usr/bin/env python

import sys

import roslib
roslib.load_manifest('raw_vfk_dummy')

import rospy
import actionlib
from tf.transformations import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from raw_vfk_dummy.msg import *

if __name__ == "__main__":
	# initialize node
	rospy.init_node('raw_vfk_deliver_boxes_client')
	
	# define box_ids
	box_ids = [2,5,7]
	# define search_pose
	search_pose = PoseStamped()
	search_pose.header.stamp = rospy.Time.now()
	search_pose.header.frame_id = "/map"
	search_pose.pose.position.x = 6.78
	search_pose.pose.position.y = 7.89
	q = quaternion_from_euler(0, 0, 1.5708) # transform from roll pitch yaw to quaternions
	search_pose.pose.orientation.x = q[0]
	search_pose.pose.orientation.y = q[1]
	search_pose.pose.orientation.z = q[2]
	search_pose.pose.orientation.w = q[3]
	# define target_pose
	target_pose = PoseStamped()
	target_pose.header.stamp = rospy.Time.now()
	target_pose.header.frame_id = "/map"
	target_pose.pose.position.x = 4.56
	target_pose.pose.position.y = 5.67
	q = quaternion_from_euler(0, 0, 0.7854) # transform from roll pitch yaw to quaternions
	target_pose.pose.orientation.x = q[0]
	target_pose.pose.orientation.y = q[1]
	target_pose.pose.orientation.z = q[2]
	target_pose.pose.orientation.w = q[3]

	# wait for service to be up
	rospy.loginfo ("waiting for action: deliver_boxes")
	client = actionlib.SimpleActionClient('deliver_boxes', DeliverBoxesAction)
	client.wait_for_server()

	# fill goal message
	goal = DeliverBoxesGoal()
	goal.box_ids = [2,5,7]
	goal.search_pose = search_pose
	goal.target_pose = target_pose
	
	# send goal
	client.send_goal(goal)
	
	# wait for result
	client.wait_for_result()
	
	# print result
	if client.get_state() == GoalStatus.SUCCEEDED:
		print client.get_result()
	else:
		rospy.logerr("goal not successfull. Goal status is %d",client.get_state()) 
