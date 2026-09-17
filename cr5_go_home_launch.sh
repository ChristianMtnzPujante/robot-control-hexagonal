#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/chris/Desktop/robot-control-hexagonal/install/setup.bash
exec ros2 run robot_node cr5_go_home_demo --host 192.168.5.1
