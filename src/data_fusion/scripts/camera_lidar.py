#!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import os
import rospy
import rospkg
import roslaunch
import time
import rosnode
from std_msgs.msg import String
from sensor_msgs.msg import Image
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from dynamic_reconfigure.parameter_generator_catkin import *

flag = True
count = 10
f = None
def main():
    global f

    rospy.init_node('camera_lidar',anonymous=False)        
    # rospy.Subscriber('/camera/rgb/image_raw',Image,get_image)
    # rospy.Subscriber('/',String,AchieveGoal)
    # rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,AmclStartGoal)
    # pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    # pub_initialpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    # pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    os.environ['MAP_FILE'] = '/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/world_cylinder_4_0.yaml'
    os.environ['WORLD_DIR'] = '/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/world_cylinder_4_0.world'
    os.environ['X_START'] ='1.0'
    os.environ['Y_START'] ='1.0'
    time.sleep(1)

    f = open('/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/data_fusion/data/image.txt',mode='a+')     #打开文件，若文件不存在系统自动创建。 
    
    # 启动gazebo
    gazebo_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/launch/world.launch"])
    gazebo_launch.start()
    time.sleep(10)

    # 启动导航rviz
    nav_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/launch/navigation.launch"])
    nav_launch.start()
    time.sleep(10)
    
    #手动控制
    # os.system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py")
    
    rospy.spin()
    f.close()              #关闭文件


def get_image(data):
    global flag,count,f

    print("----------------------------------------------------------------------------------------------------------{0}----------------------------------------------------------------------------------------------------------------".format(count))
    if count:
        count -= 1
        # print(data)
        f.write(data)  # write 写入
        


if __name__ == '__main__':
    main()

