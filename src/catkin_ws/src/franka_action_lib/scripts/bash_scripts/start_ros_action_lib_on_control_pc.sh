#!/bin/bash

control_pc_ip_address=$1
workstation_ip_address=$2

sshpass -p "!@m-guardians" ssh -tt -o StrictHostKeyChecking=no iam-lab@$control_pc_ip_address << EOSSH
	source ~/Documents/robot-interface/src/catkin_ws/src/franka_action_lib/scripts/bash_scripts/set_rosmaster.sh $control_pc_ip_address $workstation_ip_address
	source ~/Documents/robot-interface/src/catkin_ws/devel/setup.bash
	roslaunch franka_action_lib execute_skill_action_server_node.launch
EOSSH
