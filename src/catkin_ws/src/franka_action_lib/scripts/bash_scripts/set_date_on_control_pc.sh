#!/bin/bash

control_pc_uname=$1
control_pc_ip_address=$2
workstation_uname=$3
workstation_ip_address=$4
control_pc_use_passwd=$5

if [ "$control_pc_use_passwd" = "0" ]; then
ssh -T $control_pc_uname@$control_pc_ip_address << EOSSH
export date_string=`ssh -T  $workstation_uname@$workstation_ip_address 'date'`
echo "!@m-guardians" | sudo -S date --set="$date_string"
EOSSH
else
sshpass -p "!@m-guardians" ssh -tt -o StrictHostKeyChecking=no $control_pc_uname@$control_pc_ip_address << EOSSH
export date_string=`ssh -T  $workstation_uname@$workstation_ip_address 'date'`
echo "!@m-guardians" | sudo -S date --set="$date_string"
EOSSH
fi