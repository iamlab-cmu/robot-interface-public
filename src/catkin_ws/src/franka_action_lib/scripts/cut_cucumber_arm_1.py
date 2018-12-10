#! /usr/bin/env python

import roslib
roslib.load_manifest('franka_action_lib')
import rospy
import actionlib
import pickle
import argparse
import numpy as np

from franka_action_lib.msg import ExecuteSkillAction, ExecuteSkillGoal

from skill_list import BaseSkill
from skill_list import ArmMoveToGoalWithDefaultSensorSkill, GripperWithDefaultSensorSkill, ArmMoveToGoalContactWithDefaultSensorSkill, StayInPositionWithDefaultSensorSkill, JointPoseWithDefaultSensorSkill, ArmRelativeMotionWithDefaultSensorSkill

def feedback_callback(feedback):
    print(feedback)

if __name__ == '__main__':
    rospy.init_node('example_execute_skill_action_client')
    client = actionlib.SimpleActionClient('/execute_skill_action_server_node/execute_skill', ExecuteSkillAction)
    client.wait_for_server()

    parser = argparse.ArgumentParser(description='Joint DMP Skill Example')
    parser.add_argument('--filename', required=True, help='filename with dmp weights')
    args = parser.parse_args()

    file = open(args.filename,"rb")
    dmp_info = pickle.load(file)

    # Move to designated position above the cutting board
    skill = ArmMoveToGoalWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params([3.0, 0.998235,0.0375818,0.0457788,0,0.0378176,-0.999266,-0.00429513,0,0.0455846,0.00601891,-0.998942,0,0.531054,0.0154956,0.301051,1])  # Run Time (1) and Desired End Effector Pose(16)
    skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
    skill.add_termination_params([1.0]) # buffer time
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    print(client.get_result())

    # Move down to contact cutting board
    skill = ArmMoveToGoalContactWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params([3.0, 0.010964,0.99987,0.0110068,0,0.999883,-0.0108553,-0.00988176,0,-0.00976118,0.011114,-0.999891,0,0.627216,-0.0954885,0.0158609,1])  # Run Time (1) and Desired End Effector Pose(16)
    skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
    skill.add_termination_params([1.0]) # buffer time
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    # Move left to contact cucumber
    skill = ArmMoveToGoalContactWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params([3.0, 0.010964,0.99987,0.0110068,0,0.999883,-0.0108553,-0.00988176,0,-0.00976118,0.011114,-0.999891,0,0.627216,-0.0954885,0.0158609,1])  # Run Time (1) and Desired End Effector Pose(16)
    skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
    skill.add_termination_params([1.0]) # buffer time
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    for i in range(5):
        # Move up above the cucumber
        skill = ArmRelativeMotionWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_trajectory_params([3.0, 0.0, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0])  # Run Time (1) and Desired End Effector Pose(16)
        skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
        skill.add_termination_params([1.0]) # buffer time
        goal = skill.create_goal()
        print(goal)
        client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

        while not rospy.is_shutdown() and done != True:
            done = client.wait_for_result(rospy.Duration.from_sec(5.0))

        print(client.get_result())

        # Move left above the cucumber
        skill = ArmRelativeMotionWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_trajectory_params([3.0, 0.0, 0.05, 0.0, 1.0, 0.0, 0.0, 0.0])  # Run Time (1) and Desired End Effector Pose(16)
        skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
        skill.add_termination_params([1.0]) # buffer time
        goal = skill.create_goal()
        print(goal)
        client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

        while not rospy.is_shutdown() and done != True:
            done = client.wait_for_result(rospy.Duration.from_sec(5.0))

        print(client.get_result())

        # Move to contact
        skill = ArmMoveToGoalContactWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_trajectory_params([3.0, 0.010964,0.99987,0.0110068,0,0.999883,-0.0108553,-0.00988176,0,-0.00976118,0.011114,-0.999891,0,0.627216,-0.0954885,0.0158609,1])  # Run Time (1) and Desired End Effector Pose(16)
        skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
        skill.add_termination_params([1.0]) # buffer time
        goal = skill.create_goal()
        print(goal)
        client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

        while not rospy.is_shutdown() and done != True:
            done = client.wait_for_result(rospy.Duration.from_sec(5.0))

        # Start DMP cutting for 5 times
        skill = JointPoseWithDefaultSensorSkill()
        skill.add_initial_sensor_values(dmp_info['phi_j'])  # sensor values

        # y0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        y0 = [-0.282618, -0.18941, 0.0668932, -2.18632, 0.0524845, 1.916, -1.06273]

        print(dmp_info['num_basis'])

        # Run time, tau, alpha, beta, num_basis, num_sensor_values, mu, h, weights
        trajectory_params = [7.0, dmp_info['tau'], dmp_info['alpha'], dmp_info['beta'],\
                             float(dmp_info['num_basis']), float(dmp_info['num_sensors'])] + dmp_info['mu'] + \
                             dmp_info['h'] + y0 + np.array(dmp_info['weights']).reshape(-1).tolist()
        skill.add_trajectory_params(trajectory_params)  
        goal = skill.create_goal()
        print(goal)
        client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

        while not rospy.is_shutdown() and done != True:
            done = client.wait_for_result(rospy.Duration.from_sec(5.0))