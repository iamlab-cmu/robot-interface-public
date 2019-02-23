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

class PickAndMoveCucumberSkill(object):

    INITIAL_POSITION = [0.1, 0.0376, -0.0007 , 0, 0.0376 , -1.0, 
                             0.029, 0, 0.00034, -0.0293, -1.0 , 0, 0.363,
                             -0.0216, 0.450, 1]
    INITIAL_CORRECT_POSE = [-0.0790, -0.633, 0.0394, -2.360, -0.0202, 1.726, 0.7104]

    PICKING_POSITION = [0.9427, -0.002, -0.33, 0, 0.0002, -1.0, 0.006, 0, -0.33,
                        -0.0063, -0.9427, 0, 0.6549,0.1212, 0.214688, 1]
    
    FIRST_INTERMEDIATE_POSITION = [-0.024, 0.822,-0.568,0,0.999,-0.001, -0.0437,
                                   0,-0.0366,-0.569, -0.821, 0, 0.634, -0.0507,
                                   0.304, 1]

    CONTACT_GOAL_POSITION = [0.0118, 0.9389,-0.3439, 0, 0.9997, -0.0173, -0.0130,
                             0, -0.0182, -0.3437, -0.9388, 0, 0.6698, -0.417,
                             0.222, 1]

    def __init__(self, args):
        self.grasping_force = args.grasping_force
        self.hold_time = args.hold_time

        self.skill_id = 0

    def execute_skill(self, skill, client):
        goal = skill.create_goal()
        print ("==== Begin goal ====")
        print(goal)
        client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))
        while not rospy.is_shutdown() and done != True:
            print(goal)
            done = client.wait_for_result(rospy.Duration.from_sec(5.0))
        print(client.get_result())
        self.skill_id = self.skill_id + 1

    def create_skill_for_class(self, klass, description):
        skill = klass(
                skill_description=description + 
                ' type: {}, id: {}'.format(klass.__name__, self.skill_id))
        return skill

    def create_open_gripper_skill(self, width, speed, wait_time,
                                  grasping_force=None, desc=''):
        # Open the gripper
        skill = self.create_skill_for_class(GripperWithDefaultSensorSkill, desc)
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        if grasping_force is None:
            skill.add_trajectory_params([width, speed, wait_time])
        else:
            skill.add_trajectory_params([width, speed, grasping_force, wait_time])
        return skill


    def create_move_to_EE_position(self, pos, time, buffer_time=1.0, desc=''):
        skill = self.create_skill_for_class(
                ArmMoveToGoalWithDefaultSensorSkill, desc)
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_trajectory_params([time] + pos)
        skill.add_feedback_controller_params([600, 50]) 
        skill.add_termination_params([buffer_time]) # buffer time
        return skill

    def create_contact_move_to_EE_position(self, pos, time, buffer_time=1.0,
                                           desc=''):
        # Move to contact goal position
        skill = self.create_skill_for_class(
                ArmMoveToGoalContactWithDefaultSensorSkill, desc)
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_trajectory_params([time] + pos)
        skill.add_feedback_controller_params([600, 50])
        skill.add_termination_params([buffer_time]) # buffer time
        return skill

    def create_go_to_pose_skill(self, pose, time, buffer_time=1.0, desc=''):
        skill = self.create_skill_for_class(
            JointPoseMinJerkWithDefaultSensorSkill, desc)

        skill.add_initial_sensor_values([1 , 3, 5, 7, 8])
        skill.add_trajectory_params(pose + [time]) 
        skill.add_feedback_controller_params([600, 50])
        skill.add_termination_params([buffer_time])
        return skill

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
    client = actionlib.SimpleActionClient(
            '/execute_skill_action_server_node/execute_skill',
            ExecuteSkillAction)

    client.wait_for_server()
    
    parser = argparse.ArgumentParser(description="Grasp cucumber with tongs.")
    parser.add_argument('--hold_time', type=float, default=1000.0,
                        help='Time to hold the cucumber for cutting (in seconds)')
    parser.add_argument('--grasping_force', type=float, default=10.0,
                        help='Force used to grasp the cucumber')
    args = parser.parse_args()

    pick_and_move_skill = PickAndMoveCucumberSkill(args)

    print ('===== ')
    print("Opening the gripper to prepare for grasping.")

    # Open the gripper
    skill = pick_and_move_skill.create_open_gripper_skill(0.07, 0.025, 1100)
    pick_and_move_skill.execute_skill(skill, client)

    print("====")
    print("Move to inital position")

    # Move to Initial Position
    move_to_initial_position_skill = pick_and_move_skill.create_move_to_EE_position(
            PickAndMoveCucumberSkill.INITIAL_POSITION, 3.0, 1.0)
    pick_and_move_skill.execute_skill(move_to_initial_position_skill, client)


    # Correct your pose while in the initial position
    correct_pose_skill = pick_and_move_skill.create_go_to_pose_skill(
            PickAndMoveCucumberSkill.INITIAL_CORRECT_POSE, 2.0)
    pick_and_move_skill.execute_skill(correct_pose_skill, client)

    print ('===== ')
    print("Moving to the cucumber position.")

    # Move to Picking Position
    pick_position_skill = pick_and_move_skill.create_move_to_EE_position(
            PickAndMoveCucumberSkill.PICKING_POSITION, 3.0)
    pick_and_move_skill.execute_skill(pick_position_skill, client)

    print ('===== ')
    print("Closing the gripper and grasping.")

    skill = pick_and_move_skill.create_open_gripper_skill(
            0.05, 0.025, 1100, grasping_force=args.grasping_force)
    pick_and_move_skill.execute_skill(skill, client)

    print ('===== ')
    print("Moving to intermediate position.")

    # Move to intermediate Position
    pick_position_skill = pick_and_move_skill.create_move_to_EE_position(
            PickAndMoveCucumberSkill.FIRST_INTERMEDIATE_POSITION, 3.0)
    pick_and_move_skill.execute_skill(pick_position_skill, client)

    print ('===== ')
    print("Moving to contact goal position")

    skill = pick_and_move_skill.create_contact_move_to_EE_position(
            PickAndMoveCucumberSkill.CONTACT_GOAL_POSITION, 3.0)
    pick_and_move_skill.execute_skill(skill, client)

    print ('===== ')
    print("Stay in Position")

    # Stay in the position for a certain amount of time
    skill = StayInPositionWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params([args.hold_time])  # Run Time 
    skill.add_feedback_controller_params([800, 50])
    pick_and_move_skill.execute_skill(skill, client)

    # Now move to another location 
    print ('===== ')
    print("Open the Gripper")

    # Open the gripper
    skill = pick_and_move_skill.create_open_gripper_skill(
            0.05, 0.025, 1100)
    pick_and_move_skill.execute_skill(skill, client)

    print ('===== ')
    print("Moving to the original position.")

    # Move to original position
    move_to_initial_position_skill = pick_and_move_skill.create_move_to_EE_position(
            PickAndMoveCucumberSkill.INITIAL_POSITION, 3.0, 1.0)
    pick_and_move_skill.execute_skill(move_to_initial_position_skill, client)
