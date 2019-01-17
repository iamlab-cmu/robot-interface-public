#!/bin/bash

control_pc_uname=$1
control_pc_ip_address=$2
workstation_ip_address=$3
control_pc_use_passwd=$4

if [ "$(control_pc_use_passwd)" -eq 0 ]; then
ssh -T $control_pc_uname@$control_pc_ip_address << EOSSH
source ~/Documents/robot-interface/src/catkin_ws/src/franka_action_lib/scripts/bash_scripts/set_rosmaster.sh $control_pc_ip_address $workstation_ip_address
source ~/Documents/robot-interface/src/catkin_ws/devel/setup.bash
roslaunch franka_action_lib execute_skill_action_server_node.launch
EOSSH
else
sshpass -p "!am-guardians" ssh -tt -o StrictHostKeyChecking=no $control_pc_uname@$control_pc_ip_address << EOSSH
source ~/Documents/robot-interface/src/catkin_ws/src/franka_action_lib/scripts/bash_scripts/set_rosmaster.sh $control_pc_ip_address $workstation_ip_address
source ~/Documents/robot-interface/src/catkin_ws/devel/setup.bash
roslaunch franka_action_lib execute_skill_action_server_node.launch
EOSSH
fi
