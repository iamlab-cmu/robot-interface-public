#! /usr/bin/env python

import roslib
roslib.load_manifest('franka_action_lib')
import rospy
import actionlib

from franka_action_lib.msg import ExecuteSkillAction, ExecuteSkillGoal, RobotState

from frankapy.skill_list import BaseSkill
from frankapy.skill_list import *

import argparse

def feedback_callback(feedback):
    print(feedback)

def load_result_into_robot_state_msg(result):
    robot_state = RobotState()

    current_result_index = 0

    for i in range(16):
        robot_state.pose[i] = result.execution_result[current_result_index]
        current_result_index += 1

    for i in range(7):
        robot_state.joint_torques[i] = result.execution_result[current_result_index]
        current_result_index += 1

    for i in range(7):
        robot_state.joint_torques_derivative[i] = result.execution_result[current_result_index]
        current_result_index += 1

    for i in range(7):
        robot_state.joints[i] = result.execution_result[current_result_index]
        current_result_index += 1

    for i in range(7):
        robot_state.joint_velocities[i] = result.execution_result[current_result_index]
        current_result_index += 1

    return robot_state


if __name__ == '__main__':
    rospy.init_node('example_execute_skill_action_client')
    client = actionlib.SimpleActionClient('/execute_skill_action_server_node/execute_skill', ExecuteSkillAction)
    client.wait_for_server()
    pub = rospy.Publisher('Arm_2_robot_state', RobotState, queue_size=10)
    
    parser = argparse.ArgumentParser(description="Grasp cucumber with tongs.")
    parser.add_argument('--hold_time', type=float, default=1000.0,
                        help='Time to hold the cucumber for cutting (in seconds)')
    parser.add_argument('--grasping_force', type=float, default=10.0,
                        help='Force used to grasp the cucumber')
    parser.add_argument('--use_secondary_intermediate', type=int, default=0,
                        help='Use two intermediate positions to place the ' \
                             'cucumber. Useful when we use boxes.')
    args = parser.parse_args()

    print ('===== ')
    print("Opening the gripper to prepare for grasping.")

    # Open the gripper
    skill = GripperWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params([0.07, 0.025, 1100])  # Gripper Width, Gripper Speed, Wait Time
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    print("====")
    print("Move to inital position")

    # Move to Picking Position
    move_to_initial_position_skill = ArmMoveToGoalWithDefaultSensorSkill()
    move_to_initial_position_skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    move_to_initial_position_skill.add_trajectory_params(
            [3.0, 0.99928,0.0376774,-0.000763022,0,0.0376835,-0.99885,0.0293091,
             0,0.000342153,-0.0293173,-0.99957,0,0.363079,-0.0216016,0.450419,1])
    move_to_initial_position_skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
    move_to_initial_position_skill.add_termination_params([1.0]) # buffer time
    goal = move_to_initial_position_skill.create_goal()
    print(goal)
    client.send_goal(
            goal,
            feedback_cb=lambda x: move_to_initial_position_skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    robot_state = load_result_into_robot_state_msg(client.get_result())
    pub.publish(robot_state)

    print ('===== ')
    print("Moving to the cucumber position.")

    # Move to Picking Position
    skill = ArmMoveToGoalWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params(
            [3.0, 0.942793,-0.0019957,-0.333343,0,
             0.000219568,-0.999969,0.00660774,0,-0.333352,-0.00630305,
             -0.942781,0,0.654899,0.121228,0.214688,1])
    skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
    skill.add_termination_params([1.0]) # buffer time
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    robot_state = load_result_into_robot_state_msg(client.get_result())
    pub.publish(robot_state)

    print ('===== ')
    print("Closing the gripper and grasping.")

    # Close the gripper and grasp
    skill = GripperWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    # Gripper Width, Gripper Speed, Grasping Force, Wait Time
    skill.add_trajectory_params([0.05, 0.025, args.grasping_force, 1100]) 
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))


    print ('===== ')
    print("Moving to intermediate position.")

    # Move to Intermediate Position
    skill = ArmMoveToGoalWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params(
            [3.0, -0.0240269,0.822105,-0.568812,0,0.999031,-0.00109878,
             -0.0437875,0,-0.0366236,-0.569324,-0.821297,0,0.634416,
             -0.0507561,0.304298,1])
    skill.add_feedback_controller_params([600, 50])
    skill.add_termination_params([1.0]) # buffer time
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    robot_state = load_result_into_robot_state_msg(client.get_result())
    pub.publish(robot_state)


    if args.use_secondary_intermediate == 1:
        print ('===== ')
        print("Moving to second intermediate position.")

        # Move to Intermediate Position
        skill = ArmMoveToGoalWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_trajectory_params([3.0, 0.0174699,0.816579,-0.576953,0,0.98671,
                                     -0.107281,-0.121961,0,-0.161491,-0.567166,
                                     -0.807616,0,0.651099,-0.336885,0.161649,1]
        skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
        skill.add_termination_params([1.0]) # buffer time
        goal = skill.create_goal()
        print(goal)
        client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

        while not rospy.is_shutdown() and done != True:
            done = client.wait_for_result(rospy.Duration.from_sec(5.0))

        robot_state = load_result_into_robot_state_msg(client.get_result())
        pub.publish(robot_state)

    print ('===== ')
    print("Moving to contact goal position")

    # Move to contact goal position
    skill = ArmMoveToGoalContactWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params(
        [3.0, 0.00497605,0.754413,-0.656366,0,0.999977,-0.00450945,
         0.00239798,0,-0.0011508,-0.656376,-0.754433,0,0.637414,
         -0.373751,0.185472,1])
    skill.add_feedback_controller_params([600, 50])
    skill.add_termination_params([1.0]) # buffer time
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    robot_state = load_result_into_robot_state_msg(client.get_result())
    pub.publish(robot_state)

    print ('===== ')
    print("Stay in Position")

    # Stay in the position for a certain amount of time
    skill = StayInPositionWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params([args.hold_time])  # Run Time 
    skill.add_feedback_controller_params([800, 50])
    goal = skill.create_goal()

    print(goal)
    
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(1.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(1.0))
        pub.publish(robot_state)

    print ('===== ')
    print("Open the Gripper")

    # Open the gripper
    skill = GripperWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params([0.05, 0.025, 1100])  # Gripper Width, Gripper Speed, Wait Time
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    print ('===== ')
    print("Moving to the original position.")

    # Move to original position
    skill = ArmMoveToGoalWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params([5.0, 0.99747,0.0476472,-0.0525668,0,0.0488561,-0.998555,0.0219546,0,-0.0514457,-0.0244678,-0.998376,0,0.434247,-0.0252676,0.333927,1])  # Run Time (1) and Desired End Effector Pose(16)
    skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
    skill.add_termination_params([1.0]) # buffer time
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    robot_state = load_result_into_robot_state_msg(client.get_result())
    pub.publish(robot_state)
