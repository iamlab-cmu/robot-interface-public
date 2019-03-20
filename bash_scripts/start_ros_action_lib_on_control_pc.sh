#!/bin/bash

control_pc_uname=$1
control_pc_ip_address=$2
workstation_ip_address=$3
control_pc_use_passwd=$4
control_pc_robot_lib_path=$5

rosmaster_path="/bash_scripts/set_rosmaster.sh"

echo $control_pc_uname
echo $control_pc_ip_address
echo $control_pc_robot_lib_path

if [ "$control_pc_use_passwd" = "0" ]; then
ssh -T $control_pc_uname@$control_pc_ip_address << EOSSH
source $control_pc_robot_lib_path$rosmaster_path $control_pc_ip_address $workstation_ip_address
source $control_pc_robot_lib_path"/catkin_ws/devel/setup.zsh"
roslaunch franka_action_lib franka_ros_interface.launch
EOSSH
else
sshpass -p "!@m-guardians" ssh -tt -o StrictHostKeyChecking=no $control_pc_uname@$control_pc_ip_address << EOSSH
source ~/Documents/robot-interface/bash_scripts/set_rosmaster.sh $control_pc_ip_address $workstation_ip_address
source ~/Documents/robot-interface/catkin_ws/devel/setup.bash
roslaunch franka_action_lib franka_ros_interface.launch
EOSSH
fi
