# -*- coding: utf-8 -*-

import os
import xlrd
import rospy
import rospkg
import roslaunch
import time
import rosnode
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from dynamic_reconfigure.parameter_generator_catkin import *


class MyPlugin_mapping(Plugin):
    __rviz_launch_file = "/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/launch/navigation_rviz_turtlebot3.launch"
    __amcl_launch_file = "/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/launch/amcl.launch"
    __goal_when_stop_launch_file = "/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/launch/goal_when_stop.launch"
    __navigation_stop_launch_file = "/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/launch/navigation_stop.launch"
    __move_base_launch_dir = "/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/launch/move_base/"
    # __move_base_launch_file = "/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/turtlebot3/turtlebot3_navigation/launch/move_base.launch"
    __move_base_launch_file = "/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/launch/dwa_local_planner/move_base.launch"
    #  __move_base_launch_file =  "/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/launch/dwa_local_planner/move_base.launch"
    __gazebo_launch_file = "/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/launch/world.launch"
    __mapping_file = "/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/src/rqt_mypkg_requirement_to_algorithm/mapping.xlsx"

    def __init__(self, context):
        super(MyPlugin_mapping, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin_mapping')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q",
                            "--quiet",
                            action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg_requirement_to_algorithm'),
                               'resource', 'MyPlugin_mapping.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPlugin_mappingUi')
        self._widget.setStyleSheet(
            "QWidget#MyPlugin_mappingUi{border-top: 1px solid #000000;}")
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        # 将按钮的clicked信号连接到回调函数
        self._widget.startRvizButton.clicked.connect(self.run_rviz)

        self._widget.startAmclButton.clicked.connect(self.run_amcl)
        self._widget.stopAmclButton.clicked.connect(self.stop_amcl)

        self._widget.startMoveBaseButton.clicked.connect(self.run_move_base)
        self._widget.stopMoveBaseButton.clicked.connect(self.stop_move_base)
        
        self._widget.map_1.clicked.connect(self.map_1_selected)
        self._widget.map_2.clicked.connect(self.map_2_selected)
        self._widget.map_3.clicked.connect(self.map_3_selected)

        self._widget.time_1.clicked.connect(self.time_1_selected)
        self._widget.time_2.clicked.connect(self.time_2_selected)
        self._widget.time_3.clicked.connect(self.time_3_selected)

        self._widget.comfort_1.clicked.connect(self.comfort_1_selected)
        self._widget.comfort_2.clicked.connect(self.comfort_2_selected)
        self._widget.comfort_3.clicked.connect(self.comfort_3_selected)
        
        self._widget.safety_1.clicked.connect(self.safety_1_selected)
        self._widget.safety_2.clicked.connect(self.safety_2_selected)
        self._widget.safety_3.clicked.connect(self.safety_3_selected)
        
        # self._widget.stopButton.clicked.connect(self.stop_rviz)
        # self._widget.testButton.clicked.connect(self.run_world)
        self._widget.glButton.clicked.connect(self.run_GL)
        # self._widget.saveButton.clicked.connect(self.savePerformance)

        self._uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self._uuid)
        self._rviz_launch = None
        self._amcl_launch = None
        self._move_base_launch = None
        self._gazebo_launch = None


        ##################################
        self._goal_when_stop_launch = None
        self._navigation_stop_launch = None
        self._base_global_planner ="navfn/NavfnROS"
        self._base_local_planner = "base_local_planner/TrajectoryPlannerROS"
        '''
        bgp_all = ["global_planner/GlobalPlanner","navfn/NavfnROS"] # 全局规划算法的集合
        blp_all = ["eband_local_planner/EBandPlannerROS","dwa_local_planner/DWAPlannerROS","base_local_planner/TrajectoryPlannerROS"] # 局部....

        navfn/NavfnROS + base_local_planner/TrajectoryPlannerROS
        navfn/NavfnROS + dwa_local_planner/DWAPlannerROS
        global_planner/GlobalPlanner + dwa_local_planner/DWAPlannerROS
        global_planner/GlobalPlanner + base_local_planner/TrajectoryPlannerROS
        
        '''
        
        wb = xlrd.open_workbook(self.__mapping_file,encoding_override='utf-8')
        sheet1 = wb.sheet_by_name('Sheet1')

        self._dict_mapping = {}
        for row_num in range(2,sheet1.nrows):
            row = sheet1.row_values(row_num)
            row_safety = row[0]
            row_time = row[1]
            row_comfort = row[2]
            row_local_planner = row[3]
            row_global_planner = row[4]
            requirement = row_safety +" " +  row_time + " " + row_comfort
            self._dict_mapping[requirement] = row_global_planner + " + " +row_local_planner
        print(self._dict_mapping)
        ##################################
        
        self._global_planner = "car"
        self._local_planner = "tra"

        self._map = "1"
        self._map_flag = True
        self._time = "low"
        self._safety = "high"
        self._comfort = "high"
        #设置默认地图
        os.environ['X_START'] ='1.0'
        os.environ['Y_START'] ='1.0'
        os.environ['MAP_FILE'] ='/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/world_cylinder_4_0.yaml'
        os.environ['WORLD_DIR'] ='/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/world_cylinder_4_0.world'
        # self._world_launch = None

        self.sub_msg_cd = rospy.Subscriber("/msg_cd", String, self.msg_cd_handler)
        self.sub_msg_tl = rospy.Subscriber("/msg_tl", String, self.msg_tl_handler)
        self.sub_msg_ts = rospy.Subscriber("/msg_ts", String, self.msg_ts_handler)
        self.sub_msg_et = rospy.Subscriber("/msg_et", String, self.msg_et_handler)
        self.sub_msg_pv = rospy.Subscriber("/msg_pv", String, self.msg_pv_handler)
  
    def savePerformance(self):
        msg_ts_final = self._widget.label_ts.text()
        msg_cd_final = self._widget.label_cd.text()
        msg_tl_final = self._widget.label_tl.text()
        msg_et_final = self._widget.label_et.text()
        msg_pv_final = self._widget.label_pv.text()
        rospy.loginfo("*******************************************"+msg_ts_final)
        with open('/home/zlb/performance.txt', 'a+') as f:
            f.write(self._global_planner + " & " + self._local_planner + "\n" + \
            msg_ts_final + "\n" + msg_cd_final + "\n" + msg_tl_final + "\n" + \
            msg_et_final + "\n" + msg_pv_final + "\n ------------ \n")
    
    def msg_ts_handler(self,data):
        # msg_ts = "Trajectory Smoothness: " + data.data
        self._widget.label_ts.setText(data.data)
    def msg_cd_handler(self,data):
        # msg_cd = "Closest Distance: " + data.data
        self._widget.label_cd.setText(data.data)
    def msg_tl_handler(self,data):
        # msg_tl = "Trajectory Length: " + data.data
        self._widget.label_tl.setText(data.data)
    def msg_et_handler(self,data):
        # msg_et = "Excution Time: " + data.data
        self._widget.label_et.setText(data.data)
    def msg_pv_handler(self,data):
        # msg_pv = "Plan Evaluation: " + data.data
        self._widget.label_pv.setText(data.data)
    def run_GL(self):
        ##########
        requirement = self._safety + " " + self._time + " " + self._comfort
        combine =  self._dict_mapping[requirement].split(" + ")
        self._base_global_planner = combine[0]
        self._base_local_planner = combine[1]
        ##########
        sh_command = "rosrun dynamic_reconfigure dynparam set /move_base '{'base_global_planner': " + self._base_global_planner +", 'conservative_reset_dist': 3.0, 'groups': {'base_global_planner': "+ self._base_global_planner + ", 'planner_frequency': 5.0, 'parent': 0, 'conservative_reset_dist': 3.0, 'shutdown_costmaps': False, 'restore_defaults': False, 'groups': {}, 'oscillation_timeout': 10.0, 'id': 0, 'controller_patience': 15.0, 'name': 'Default', 'parameters': {}, 'type': '', 'clearing_rotation_allowed': True, 'state': True, 'oscillation_distance': 0.2, 'max_planning_retries': -1, 'base_local_planner': " + self._base_local_planner + " , 'recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}, 'controller_patience': 15.0, 'max_planning_retries': -1, 'shutdown_costmaps': False, 'clearing_rotation_allowed': True, 'restore_defaults': False, 'oscillation_distance': 0.2, 'planner_frequency': 5.0, 'oscillation_timeout': 10.0, 'base_local_planner': " + self._base_local_planner + "',recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}'"
        os.system(sh_command)

        if self._base_local_planner == "base_local_planner/TrajectoryPlannerROS":
            sh_command = 'rosparam load /home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/param/base_local_planner_params.yaml'
        elif self._base_local_planner == 'dwa_local_planner/DWAPlannerROS':
            sh_command = 'rosparam load /home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/param/dwa_local_planner_params_waffle.yaml'
        os.system(sh_command)
        
        # if self._base_local_planner == "base_local_planner/TrajectoryPlannerROS":
        #     sh_command = "rosparam load /home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/param/base_local_planner_params.yaml.yaml"
        #  elif self._base_local_planner=='dwa_local_planner/DWAPlannerROS':
        #      sh_command = "rosparam load /home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/nav_launch/param/dwa_local_planner_params_waffle.yaml"
    
    def run_rviz(self):
        print("run rviz!")
        # launch被shut down之后只能重新赋新对象？
        if (self._rviz_launch is None or self._rviz_launch._shutting_down):
            self._rviz_launch = roslaunch.parent.ROSLaunchParent(
                self._uuid, [self.__rviz_launch_file])
        self._rviz_launch.start(False)
        ##实现自定位
        # if self._map == "1":
        #     pass
        # elif self._map == "2" or self._map == "3":
        #     pub_initialpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        #     msg_initialpose = PoseWithCovarianceStamped()
        #     # pose部分,包括 坐标position 和 方向orientation
        #     msg_initialpose.pose.pose.position.x = 1 $START
        #     msg_initialpose.pose.pose.position.y = 1
        #     msg_initialpose.pose.pose.position.z = 0
        #     msg_initialpose.pose.pose.orientation.x = 0
        #     msg_initialpose.pose.pose.orientation.y = 0
        #     msg_initialpose.pose.pose.orientation.z = 0
        #     msg_initialpose.pose.pose.orientation.w = 1
        #     msg_initialpose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, \
        #     0.0, 0.25, 0.0, 0.0, 0.0, 0.0, \
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        #      # 发布初始位置信息
        #     try:
        #         msg_initialpose.header.stamp = rospy.Time.now()
        #         msg_initialpose.header.frame_id = "map"
        #         pub_initialpose.publish(msg_initialpose)
        #     except KeyError as e:
        #         print(e)

    def run_amcl(self):
        print("run amcl!")
        if (self._amcl_launch is None or self._amcl_launch._shutting_down):
            self._amcl_launch = roslaunch.parent.ROSLaunchParent(
                self._uuid, [self.__amcl_launch_file])
        self._amcl_launch.start()
        self._widget.startAmclButton.setEnabled(False)
        self._widget.stopAmclButton.setEnabled(True)

    def stop_amcl(self):
        print("stop amcl!")
        if (self._amcl_launch is not None
                and not self._amcl_launch._shutting_down):
            self._amcl_launch.shutdown()
            self._widget.startAmclButton.setEnabled(True)
            self._widget.stopAmclButton.setEnabled(False)

    def run_move_base(self):
        rospy.Subscriber
        ##开启move_base节点
          # 启动gazebo
        if (self._gazebo_launch is None or self._gazebo_launch._shutting_down):
            self._gazebo_launch = roslaunch.parent.ROSLaunchParent(self._uuid, [self.__gazebo_launch_file])
        self._gazebo_launch.start()
        if (self._move_base_launch is None or self._move_base_launch._shutting_down):
            self._move_base_launch = roslaunch.parent.ROSLaunchParent(self._uuid, [self.__move_base_launch_file])
        self._move_base_launch.start()
        self._widget.startMoveBaseButton.setEnabled(False)
        self._widget.stopMoveBaseButton.setEnabled(True)


    def stop_move_base(self):
        print("stop move_base!")
        
        #################################
        ##打开"cmd_vel_stop"节点，发送速度为0的指令到 /cmd_vel，让车子停下来
        if (self._navigation_stop_launch is None or self._navigation_stop_launch._shutting_down):
            self._navigation_stop_launch = roslaunch.parent.ROSLaunchParent(self._uuid, [self.__navigation_stop_launch_file])
            self._navigation_stop_launch.start()
        ##################################
        if (self._move_base_launch is not None
                and not self._move_base_launch._shutting_down):
            self._move_base_launch.shutdown()

            self._widget.startMoveBaseButton.setEnabled(True)
            self._widget.stopMoveBaseButton.setEnabled(False)
            self._widget.baseGlobalBox.setEnabled(True)
            self._widget.baseLocalBox.setEnabled(True)

   
   
    def map_1_selected(self):
        print("map_1 selected")
        self._map = "1"
        # 设置环境变量
        os.environ['X_START'] ='-2.0'
        os.environ['Y_START'] ='-0.5'
        os.environ['MAP_FILE'] ='/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/turtlebot3_world.yaml'
        os.environ['WORLD_DIR'] ='/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/turtlebot3_world.world'
        #  if self._map_flag:
        #     print("map_1 selected")
        #     self._map = "1"
        #     # 设置环境变量
        #     os.environ['X_START'] ='-2.0'
        #     os.environ['Y_START'] ='-0.5'
        #     os.environ['MAP_FILE'] ='/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/turtlebot3_world.yaml'
        #     os.environ['WORLD_DIR'] ='/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/turtlebot3_world.world'
        #     self._map_flag = False
        
   
    def map_2_selected(self):
        #  if self._map_flag:
        print("map_2 selected!")
        self._map = "2"
        # 设置环境变量
        os.environ['X_START'] ='1.0'
        os.environ['Y_START'] ='1.0'
        os.environ['MAP_FILE'] ='/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/world_cylinder_4_0.yaml'
        os.environ['WORLD_DIR'] ='/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/world_cylinder_4_0.world'
            # self._map_flag = False
    
    def map_3_selected(self):
        # if self._map_flag:
        print("map_3 selected!")
        self._map = "3"
        os.environ['X_START'] ='1.0'
        os.environ['Y_START'] ='1.0'
        os.environ['MAP_FILE'] ='/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/world_cylinder_8_10.yaml'
        os.environ['WORLD_DIR'] ='/home/zlb/dev/ros_collections/catkin_ws_turtlebot3/src/rqt_mypkg_requirement_to_algorithm/maps/world_cylinder_8_10.world'
            # self._map_flag = False

    def time_1_selected(self):
        print("time_1 selected!")
        self._time = "high"
   
    def time_2_selected(self):
        print("time_2 selected!")
        self._time = "middle"
    
    def time_3_selected(self):
        print("time_3 selected!")
        self._time = "low"
     
    def safety_1_selected(self):
        print("safety_1_selected selected!")
        self._safety = "high"
    def safety_2_selected(self):
        print("safety_2_selected selected!")
        self._safety = "middle"
    def safety_3_selected(self):
        print("safety_3_selected selected!")
        self._safety = "low"

    def comfort_1_selected(self):
        print("comfort_1 selected!")
        self._comfort = "high"
    def comfort_2_selected(self):
        print("comfort_2 selected!")
        self._comfort = "middle"
    def comfort_3_selected(self):
        print("comfort_3 selected!")
        self._comfort = "low"





    # def stop_rviz(self):
    #     print("stop rviz!")
    #     if (self._rviz_launch is not None
    #             and not self._rviz_launch._shutting_down):
    #         self._rviz_launch.shutdown()

    # def run_world(self):
    #     print("run world!")
    #     if (self._world_launch is None or self._world_launch._shutting_down):
    #         self._world_launch = roslaunch.parent.ROSLaunchParent(
    #             self.uuid, [self.__world_launch_file])
    #     self._world_launch.start()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        pass




