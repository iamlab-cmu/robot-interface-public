#!/bin/bash

control_pc_uname=$1
control_pc_ip_address=$2
control_pc_robolib_path=$3
control_pc_use_password=$4
control_pc_password=$5


echo $control_pc_uname
echo $control_pc_ip_address
echo $control_pc_robolib_path
echo $control_pc_use_password
echo $control_pc_password

if [ "$control_pc_use_password" = "0" ]; then
ssh -tt $control_pc_uname@$control_pc_ip_address << EOSSH
cd $control_pc_robolib_path
cd build
./main_iam_robolib
bash
EOSSH
else
sshpass -p "$control_pc_password" ssh -tt -o StrictHostKeyChecking=no $control_pc_uname@$control_pc_ip_address << EOSSH
cd $control_pc_robolib_path
cd build
./main_iam_robolib
bash
EOSSH
fi
