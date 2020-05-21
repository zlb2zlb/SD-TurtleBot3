#!/usr/bin/env python
# license removed for brevity
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist 

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('cmd_vel_stop', anonymous=True)
rate = rospy.Rate(10) # 10hz
move_cmd = Twist()
move_cmd.linear.x=0
while not rospy.is_shutdown():
    rospy.loginfo(move_cmd)
    pub.publish(move_cmd)
    rate.sleep()
