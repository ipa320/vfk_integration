#!/usr/bin/env python

import sys

import roslib
roslib.load_manifest('raw_vfk_dummy')

import rospy
import actionlib
from actionlib_msgs.msg import *
from raw_vfk_dummy.msg import *

if __name__ == "__main__":
    # initialize node
    rospy.init_node('raw_vfk_deliver_boxes_cancel_client')

    # wait for service to be up
    rospy.loginfo ("canceling action: deliver_boxes")
    client = actionlib.SimpleActionClient('deliver_boxes', DeliverBoxesAction)
    client.wait_for_server()
    client.cancel_all_goals()