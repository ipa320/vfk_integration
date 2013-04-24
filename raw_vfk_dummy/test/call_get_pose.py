#!/usr/bin/env python

import roslib
roslib.load_manifest('raw_vfk_dummy')

import rospy
from raw_vfk_dummy.srv import *

if __name__ == "__main__":
	# initialize node
	rospy.init_node('raw_vfk_get_pose_client')

	# wait for service to be up
	rospy.loginfo ("waiting for service: get_pose")
	rospy.wait_for_service('get_pose')
	try:
		# initialize service client
		get_pose = rospy.ServiceProxy('get_pose', GetPose)
		
		# fill request message
		req = GetPoseRequest()
		
		# call service
		res = get_pose(req)
		
		# print response
		print res

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
