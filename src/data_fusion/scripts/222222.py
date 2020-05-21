#!/usr/bin/python3
# -*- coding: utf-8 -*-
f = open('/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/data_fusion/data/image.txt',mode='a+')     #打开文件，若文件不存在系统自动创建。 
f.write("data")  # write 写入
f.close()              #关闭文件
if 0:
    print("-----------")