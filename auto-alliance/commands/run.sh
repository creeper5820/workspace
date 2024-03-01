#!/bin/zsh

source /root/driver/devel/setup.zsh 
roslaunch livox_ros_driver2 msg_MID360.launch

source /root/mapping/devel/setup.zsh
roslaunch fast_lio mapping_mid360.launch

source source /root/positioning/devel/setup.zsh
roslaunch fast_lio_localization localization_MID360.launch

rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0 