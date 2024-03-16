#!/bin/bash

source /opt/ros/humble/setup.bash
source /root/package/driver/install/setup.bash
source /root/package/fast_lio/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml