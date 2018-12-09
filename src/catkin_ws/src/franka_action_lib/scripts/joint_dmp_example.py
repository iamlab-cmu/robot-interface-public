#! /usr/bin/env python

import roslib
roslib.load_manifest('franka_action_lib')
import rospy
import actionlib
import pickle
import argparse

from franka_action_lib.msg import ExecuteSkillAction, ExecuteSkillGoal

from skill_list import BaseSkill
from skill_list import JointPoseWithDefaultSensorSkill

def feedback_callback(feedback):
    print(feedback)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Joint DMP Skill Example')
    parser.add_argument('--filename', required=True, help='filename with dmp weights')
    args = parser.parse_args()

    rospy.init_node('joint_dmp_skill_example_client')
    client = actionlib.SimpleActionClient('/execute_skill_action_server_node/execute_skill', ExecuteSkillAction)
    client.wait_for_server()

    file = open(args.filename,"rb")
    dmp_info = pickle.load(file)

    skill = JointPoseWithDefaultSensorSkill()
    skill.add_initial_sensor_values([dmp_info['phi_j']])  # sensor values

    y0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    print(dmp_info['num_basis'])

    # Run time, tau, alpha, beta, num_basis, num_sensor_values, mu, h, weights
    trajectory_params = [3.6, dmp_info['tau'], dmp_info['alpha'], dmp_info['beta'],\
                         dmp_info['num_basis'], dmp_info['num_sensors']] + dmp_info['mu'].tolist() + \
                         dmp_info['h'].tolist() + y0 + dmp_info['weights'].tolist()
    skill.add_trajectory_params(trajectory_params)  
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))
