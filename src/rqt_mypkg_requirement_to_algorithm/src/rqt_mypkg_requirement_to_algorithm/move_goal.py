#!/usr/bin/env python
# license removed for brevity
# -*- coding: utf-8 -*-
import rospy
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped 

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    # data.header.frame_id = "map"
    ## /goal_when_stop
    rospy.set_param('/goal_when_stop', {
        'header': {
            'seq':data.header.seq,
            'stamp':{
                'secs':data.header.stamp.secs,
                'nsecs':data.header.stamp.nsecs},
            'frame_id':data.header.frame_id}, 
        'pose': {
            'position':{
                'x':data.pose.position.x,
                'y':data.pose.position.y,
                'z':data.pose.position.z},
            'orientation':{
                'x':data.pose.orientation.x,
                'y':data.pose.orientation.y,
                'z':data.pose.orientation.z,
                'w':data.pose.orientation.w}}
            })

def listener():
    rospy.init_node('goal_listen_pub', anonymous=True)
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, callback)
    rospy.spin()

listener()
