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
from skill_list import ArmMoveToGoalWithDefaultSensorSkill
from skill_list import GripperWithDefaultSensorSkill
from skill_list import ArmMoveToGoalContactWithDefaultSensorSkill
from skill_list import StayInPositionWithDefaultSensorSkill
from skill_list import JointPoseWithDefaultSensorSkill
from skill_list import ArmRelativeMotionWithDefaultSensorSkill
from skill_list import ArmRelativeMotionToContactWithDefaultSensorSkill

def feedback_callback(feedback):
    print(feedback)

def execute_skill(skill, client)
    goal = skill.create_goal()
    print(goal)
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))
    done = client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))
    print(client.get_result())

class CutCucumberSkill(object):
    INITIAL_POSITION = [-0.0372113,0.999006,0.0241583,0,0.998733,0.0379922,
                        -0.0327119,0,-0.0335979,0.0229109,-0.999173,0,0.458702,
                        -0.243681,0.245551,1]

    POSITION_ABOVE_CUTTING_BOARD = [0.00428579,0.999112,-0.0416815,0,0.999585,
                                    -0.00545372,-0.0279469,0,-0.02815, -0.0415,
                                    -0.99874,0,0.502556,-0.175819,0.141577,1]

    MOVE_TO_CUTTING_BOARD_POSITION = [0.00338211,0.99924,-0.0385855,0,0.999849,
                                      -0.00401365,-0.0163014,0,-0.0164442,
                                      -0.0385253,-0.999122,0,0.496864,
                                      -0.176568,0.0293225,1]
    MOVE_TO_CUCUMBER_POSITION = [0.00193678,0.999977,0.00475145,0,0.9999,
                                 -0.00199989,0.0133132,0,0.0133227,0.00472528,
                                 -0.9999,0,0.517015,0.04119,0.024956,1]

    SLICE_THICKNESS = 0.0075

    RELATIVE_MOTION_TO_CONTACT_FOR_CUTTING = 0.08

    # Quaternion helpers
    IDENTITY_QUATERNION = [1., 0., 0., 0.]

    # Time helpers
    RANDOM_EXPLORATION_TIME = 1.0

    def __init__(self, cutting_knife_location_x):
        self.cutting_knife_location_x = cutting_knife_location_x

    def get_move_left_skill(self, distance_in_m):
        skill = ArmRelativeMotionWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])
        skill.add_relative_motion_with_quaternion(
                3.0,
                [0., -distance_in_m, 0.],
                CutCucumberSkill.IDENTITY_QUATERNION)
        skill.add_feedback_controller_params([600, 50])
        skill.add_termination_params([1.0])
        return skill

    def create_skill_to_move_to_cucumber(self):
        ''' Move left to contact cucumber '''
        skill = ArmMoveToGoalContactWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])
        skill.add_trajectory_params(
                [3.0] + CutCucumberSkill.MOVE_TO_CUCUMBER_POSITION)
        skill.add_feedback_controller_params([600, 50])
        skill.add_contact_termination_params(1.0, [10.0] * 6, [10.0] * 6)
        return skill

    def add_random_exploration(
            self,
            time,
            position_delta,
            lower_force_thresholds_accel=[10.0] * 6,
            lower_force_thresholds_nominal=[10.0] * 6):
        skill = ArmRelativeMotionToContactWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])
        skill.add_traj_params_with_quaternion(
                time,
                position_delta,
                CutCucumberSkill.IDENTITY_QUATERNION)
        skill.add_controller_stiffness_params(600, 50)
        skill.add_contact_termination_params(
                1.0,
                lower_force_thresholds_accel,
                lower_force_thresholds_nominal)
        return skill

    def add_random_x_exploration(
            self,
            time,
            x_delta,
            lower_force_thresholds_accel=[10.0] * 6,
            lower_force_thresholds_nominal=[10.0] * 6):
        return self.add_random_exploration(
                time,
                [x_delta, 0., 0.],
                lower_force_thresholds_accel,
                lower_force_thresholds_nominal)

    def add_random_y_exploration(
            self,
            time,
            y_delta,
            lower_force_thresholds_accel=[10.0] * 6,
            lower_force_thresholds_nominal=[10.0] * 6):
        return self.add_random_exploration(
                time,
                [0., y_delta, 0.],
                lower_force_thresholds_accel,
                lower_force_thresholds_nominal)

    def add_random_z_exploration(
            self,
            time,
            z_delta,
            lower_force_thresholds_accel=[10.0] * 6,
            lower_force_thresholds_nominal=[10.0] * 6):
        return self.add_random_exploration(
                time,
                [0., 0. z_delta],
                lower_force_thresholds_accel,
                lower_force_thresholds_nominal)


if __name__ == '__main__':
    rospy.init_node('example_execute_skill_action_client')
    client = actionlib.SimpleActionClient(
            '/execute_skill_action_server_node/execute_skill',
            ExecuteSkillAction)
    client.wait_for_server()

    parser = argparse.ArgumentParser(description='Joint DMP Skill Example')
    parser.add_argument('--filename', required=True, help='filename with dmp weights')
    args = parser.parse_args()

    file = open(args.filename,"rb")
    dmp_info = pickle.load(file)
    cutting_knife_location_x = 0.5232
    cut_cucumber_skill = CutCucumberSkill(cutting_knife_location_x)

    skill = ArmMoveToGoalWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    # Run Time (1) and Desired End Effector Pose(16)
    skill.add_trajectory_params([3.0] + CutCucumberSkill.INITIAL_POSITION])
    # translational stiffness, rotational stiffness
    skill.add_feedback_controller_params([600, 50])
    skill.add_buffer_time_for_termination(1.0)
    execute_skill(skill, client)

    # Move to designated position above the cutting board
    skill = ArmMoveToGoalWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params(
            [3.0] + CutCucumberSkill.POSITION_ABOVE_CUTTING_BOARD)
    skill.add_feedback_controller_params([600, 50])
    skill.add_buffer_time_for_termination(1.0)
    execute_skill(skill, client)

    # Move down to contact cutting board
    skill = ArmMoveToGoalContactWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params(
            [3.0] + CutCucumberSkill.MOVE_TO_CUTTING_BOARD_POSITION)
    skill.add_feedback_controller_params([600, 50])
    skill.add_buffer_time_for_termination(1.0)
    execute_skill(skill, client)

    # ==== Begin Random exploration ====
    # Add random exploration to know that you're on the cutting board
    skill = cut_cucumber_skill.add_random_x_exploration(
            CutCucumberSkill.RANDOM_EXPLORATION_TIME, 0.005)
    execute_skill(skill, client)
    skill = cut_cucumber_skill.add_random_y_exploration(
            CutCucumberSkill.RANDOM_EXPLORATION_TIME, 0.005)
    execute_skill(skill, client)
    skill = cut_cucumber_skill.add_random_z_exploration(
            CutCucumberSkill.RANDOM_EXPLORATION_TIME, 0.005)
    execute_skill(skill, client)
    # ==== End ====

    # Move left to contact cucumber
    skill = ArmMoveToGoalContactWithDefaultSensorSkill()
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params(
            [3.0] + CutCucumberSkill.MOVE_TO_CUCUMBER_POSITION)
    skill.add_feedback_controller_params([600, 50])

    skill.add_contact_termination_params(
            1.0,
            [10.0,3.0,10.0,10.0,10.0,10.0],
            [10.0,3.0,10.0,10.0,10.0,10.0])
    execute_skill(skill, client)
    # ==== Begin Random exploration ====
    # Add random exploration to know that you're on the cutting board
    skill = cut_cucumber_skill.add_random_x_exploration(
            CutCucumberSkill.RANDOM_EXPLORATION_TIME, 0.005)
    execute_skill(skill, client)
    skill = cut_cucumber_skill.add_random_y_exploration(
            CutCucumberSkill.RANDOM_EXPLORATION_TIME, 0.005)
    execute_skill(skill, client)
    skill = cut_cucumber_skill.add_random_z_exploration(
            CutCucumberSkill.RANDOM_EXPLORATION_TIME, 0.005)
    execute_skill(skill, client)
    # ==== End ====


    num_slices_to_cut = 4

    for slice_idx in range(num_slices_to_cut):
        # Move up above the cucumber
        skill = ArmRelativeMotionWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_relative_motion_with_quaternion(
                3.0,
                [0., 0., CutCucumberSkill.RELATIVE_MOTION_TO_CONTACT_FOR_CUTTING],
                CutCucumberSkill.IDENTITY_QUATERNION)

        skill.add_feedback_controller_params([600, 50])
        skill.add_termination_params([1.0])
        execute_skill(skill, client)

        # Move left above the cucumber
        skill = ArmRelativeMotionWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        self.add_relative_motion_with_quaternion(
                3.0,
                [0., CutCucumberSkill.SLICE_THICKNESS, 0.],
                CutCucumberSkill.IDENTITY_QUATERNION)

        skill.add_feedback_controller_params([600, 50])
        skill.add_termination_params([1.0])
        execute_skill(skill, client)

        # Move to contact
        skill = ArmRelativeMotionToContactWithDefaultSensorSkill()
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_traj_params_with_quaternion(
                3.0,
                [0., 0., -CutCucumberSkill.RELATIVE_MOTION_TO_CONTACT_FOR_CUTTING],
                CutCucumberSkill.IDENTITY_QUATERNION)
        skill.add_controller_stiffness_params(600, 50)
        skill.add_contact_termination_params(1.0, [10.0] * 6, [10.0] * 6)
        execute_skill(skill, client)

        # Start DMP cutting for 3 times
        skill = JointPoseWithDefaultSensorSkill()
        skill.add_initial_sensor_values(dmp_info['phi_j'])  # sensor values
        # y0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        y0 = [-0.282, -0.189, 0.0668, -2.186, 0.0524, 1.916, -1.06273
        # Run time, tau, alpha, beta, num_basis, num_sensor_values, mu, h, weights
        trajectory_params = [
                4.0, dmp_info['tau'], dmp_info['alpha'], dmp_info['beta'],
                float(dmp_info['num_basis']), float(dmp_info['num_sensors'])] \
                        + dmp_info['mu'] \
                        + dmp_info['h'] \
                        + y0 \
                        + np.array(dmp_info['weights']).reshape(-1).tolist()

        skill.add_trajectory_params(trajectory_params)
        skill.set_meta_skill_id(slice_idx+1)
        skill.set_meta_skill_type(1)
        skill.add_termination_params([1.0])
        num_of_dmps_to_run = 4
        for _ in range(num_of_dmps_to_run):
            execute_skill(skill, client)

        skill = cut_cucumber_skill.get_move_left_skill(0.06)
        execute_skill(skill, client)

        skill = cut_cucumber_skill.create_skill_to_move_to_cucumber()
        execute_skill(skill, client)
