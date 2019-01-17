#!/bin/bash

control_pc_ip_address=$1
workstation_ip_address="$(ifconfig eno1 2>/dev/null|awk '/inet addr:/ {print $2}'|sed 's/addr://')"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Start rosmaster in a new gnome-terminal
start_rosmaster_path="$DIR/start_rosmaster.sh"
gnome-terminal -e "bash $start_rosmaster_path"

sleep 19

# Ssh to the control pc and start iam_robolib in a new gnome-terminal
start_iam_robolib_on_control_pc_path="$DIR/start_iam_robolib_on_control_pc.sh"
gnome-terminal -e "bash $start_iam_robolib_on_control_pc_path $control_pc_ip_address $workstation_ip_address"

sleep 19

# Ssh to the control pc and start ROS action server in a new gnome-terminal
start_ros_action_lib_on_control_pc_path="$DIR/start_ros_action_lib_on_control_pc.sh"
gnome-terminal -e "bash $start_ros_action_lib_on_control_pc_path $control_pc_ip_address $workstation_ip_address"

sleep 19

# Start realsense camera on the workstation pc in a new gnome-terminal
start_realsense_on_workstation_path="$DIR/start_realsense_on_workstation.sh"
gnome-terminal -e "bash $start_realsense_on_workstation_path"

