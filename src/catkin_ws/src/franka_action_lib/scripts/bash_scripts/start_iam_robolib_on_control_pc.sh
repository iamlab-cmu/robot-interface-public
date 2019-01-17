#!/bin/bash

control_pc_ip_address=$1
workstation_ip_address=$2

sshpass -p "!@m-guardians" ssh -tt -o StrictHostKeyChecking=no iam-lab@$control_pc_ip_address << EOSSH
	~/Documents/robot-interface/build/test_iam_robolib
EOSSH
