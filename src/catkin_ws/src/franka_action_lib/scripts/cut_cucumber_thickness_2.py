#! /usr/bin/env python

import roslib
roslib.load_manifest('franka_action_lib')
import rospy
import actionlib
import pickle
import argparse
import numpy as np
import os

from franka_action_lib.msg import ExecuteSkillAction, ExecuteSkillGoal
from frankapy.skill_list import *

from autolab_core import transformations

import cv2

def feedback_callback(feedback):
    print(feedback) 


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
                                 -0.9999,0,0.517015,0.1,0.06,1]

    SLICE_THICKNESS = 0.01
    FIRST_SLICE_THICKNESS = 0.02

    RELATIVE_MOTION_TO_CONTACT_FOR_CUTTING = 0.2
    RELATIVE_HEIGHT_ABOVE_CUCUMBER = 0.1

    # Quaternion helpers
    IDENTITY_QUATERNION = [1., 0., 0., 0.]

    # Time helpers
    RANDOM_EXPLORATION_TIME = 1.0

    def __init__(self, cutting_knife_location_x):

        self.cutting_knife_location_x = cutting_knife_location_x
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
        return client.get_result()

    def create_skill_for_class(self, klass, description):
        skill = klass(
                skill_description=description + 
                ' type: {}, id: {}'.format(klass.__name__, self.skill_id))
        return skill

    def get_move_left_skill(self, distance_in_m, time=3.0, desc=''):
        skill = self.create_skill_for_class(
                ArmRelativeMotionWithDefaultSensorSkill,
                desc)
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])
        skill.add_relative_motion_with_quaternion(
                time,
                [0., -distance_in_m, 0.],
                CutCucumberSkill.IDENTITY_QUATERNION)
        skill.add_feedback_controller_params([600, 50])
        skill.add_termination_params([1.0])
        return skill

    def create_skill_to_move_to_cucumber(self, desc=''):
        ''' Move left to contact cucumber '''
        skill = self.create_skill_for_class(
                ArmMoveToGoalContactWithDefaultSensorSkill,
                desc)
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])
        skill.add_trajectory_params(
                [3.0] + CutCucumberSkill.MOVE_TO_CUCUMBER_POSITION)
        skill.add_feedback_controller_params([600, 50])
        skill.add_contact_termination_params(
                1.0,
                [10.0] + [3.0] + [10.0] * 4,
                [10.0] + [3.0] + [10.0] * 4)
        return skill

    def create_skill_to_move_to_air(self, time, dist, quaternion, desc):
        '''Skill that moves the EE pose above in z-axis.'''
        skill = self.move_to_relative_position_and_orientation(
                time,
                [0., 0., dist],
                quaternion,
                [10.0] * 6,
                [10.0] * 6,
                desc)
        return skill

    def move_to_relative_position_and_orientation(
            self,
            time,
            position_delta,
            quaternion,
            lower_force_thresholds_accel=[3.0] * 6,
            lower_force_thresholds_nominal=[3.0] * 6,
            description=''):
        skill = self.create_skill_for_class(
            ArmRelativeMotionToContactWithDefaultSensorSkill,
            description)
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])
        skill.add_traj_params_with_quaternion(
                time,
                position_delta,
                quaternion)
        # assert skill._num_trajectory_generator_params == 8, "WTF"
        skill.add_controller_stiffness_params(600, 50)
        skill.add_contact_termination_params(
                1.0,
                lower_force_thresholds_accel,
                lower_force_thresholds_nominal)
        return skill


if __name__ == '__main__':

    rospy.init_node('example_execute_skill_action_client_2', 
                    log_level=rospy.DEBUG)
    time_now = rospy.Time.now()
    print("Time now: {:.6f}".format(time_now.to_sec()))
    client = actionlib.SimpleActionClient(
            '/execute_skill_action_server_node_2/execute_skill',
            ExecuteSkillAction)
    client.wait_for_server()

    rospy.loginfo("Will start server")

    parser = argparse.ArgumentParser(description='Joint DMP Skill Example')
    parser.add_argument('--filename', required=True, 
                        help='filename with dmp weights')
    parser.add_argument('--thickness', nargs='+', type=float, default=[0.01],
                        help='Thickness for cut slices')
    parser.add_argument('--num_dmps', type=int, default=4,
                        help='Number of DMPs to run continously to cut 1 slice.')
    parser.add_argument('--move_in_air', type=int, default=0,
                        help='Do not slide on the cutting board.')
    parser.add_argument('--move_slices_left', type=int, default=0,
                        help='Move slices left after each cut.')
    parser.add_argument('--move_down_after_slicing', type=int, default=0,
                        help='Move arm down after each cut.')
    parser.add_argument('--objects', nargs='+', default=['cucumber'])
    args = parser.parse_args()

    cutting_knife_location_x = 0.5232
    cut_cucumber_skill = CutCucumberSkill(cutting_knife_location_x)


    file = open(args.filename,"rb")
    dmp_info = pickle.load(file)

    for item in args.objects:

        skill = cut_cucumber_skill.create_skill_for_class(
                ArmMoveToGoalWithDefaultSensorSkill,
                'move_to_initial_position')
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        # Run Time (1) and Desired End Effector Pose(16)
        skill.add_trajectory_params([3.0] + CutCucumberSkill.INITIAL_POSITION)
        # translational stiffness, rotational stiffness
        skill.add_feedback_controller_params([600, 50])
        skill.add_buffer_time_for_termination(1.0)
        cut_cucumber_skill.execute_skill(skill, client)

        # Move to designated position above the cutting board
        skill = cut_cucumber_skill.create_skill_for_class(
                ArmMoveToGoalWithDefaultSensorSkill,
                'move_above_cutting_board')
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_trajectory_params(
                [2.0] + CutCucumberSkill.POSITION_ABOVE_CUTTING_BOARD)
        skill.add_feedback_controller_params([600, 50])
        skill.add_buffer_time_for_termination(1.0)
        cut_cucumber_skill.execute_skill(skill, client)

        # Move down to contact cutting board
        skill = cut_cucumber_skill.create_skill_for_class(
                ArmMoveToGoalContactWithDefaultSensorSkill,
                'move_onto_cutting_board')
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_trajectory_params(
                [2.0] + CutCucumberSkill.MOVE_TO_CUTTING_BOARD_POSITION)
        skill.add_feedback_controller_params([600, 50])
        skill.add_buffer_time_for_termination(1.0)
        cut_cucumber_skill.execute_skill(skill, client)

        orig_quaternion_position = np.array(
            CutCucumberSkill.MOVE_TO_CUTTING_BOARD_POSITION).reshape(4, 4)

        if args.move_in_air:
            move_air_dist = 0.03
            move_to_air_skill = cut_cucumber_skill.create_skill_to_move_to_air(
                    1.0, move_air_dist,
                    CutCucumberSkill.IDENTITY_QUATERNION,
                    "move_to_air_to_avoid_rubbing_board_0")
            cut_cucumber_skill.execute_skill(move_to_air_skill, client)
        
        #Move left to contact cucumber
        skill = cut_cucumber_skill.create_skill_for_class(
                 ArmMoveToGoalContactWithDefaultSensorSkill,
                 'move_left_to_contact_cucumber')
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_trajectory_params(
                [3.0] + CutCucumberSkill.MOVE_TO_CUCUMBER_POSITION)
        skill.add_feedback_controller_params([600, 50])

        skill.add_contact_termination_params(
                1.0,
                [10.0,3.0,10.0,10.0,10.0,3.0],
                [10.0,3.0,10.0,10.0,10.0,3.0])
        cut_cucumber_skill.execute_skill(skill, client)
        # skill = cut_cucumber_skill.create_skill_to_move_to_cucumber(
        #         desc='move_left_to_contact_cucumber_after_slice_{}'.format(slice_idx))
        # cut_cucumber_skill.execute_skill(skill, client)

        num_slices_to_cut = len(args.thickness)

        for slice_idx in range(num_slices_to_cut):
            # Move up above the cucumber
            move_up_above_cucumber_skill = cut_cucumber_skill.create_skill_for_class(
                ArmRelativeMotionWithDefaultSensorSkill,
                'move_up_above_cucumber_{}_above_{:.3f}'.format(
                    slice_idx, CutCucumberSkill.RELATIVE_HEIGHT_ABOVE_CUCUMBER))
            move_up_above_cucumber_skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
            move_up_above_cucumber_skill.add_relative_motion_with_quaternion(
                    2.0,
                    [0., 0., CutCucumberSkill.RELATIVE_HEIGHT_ABOVE_CUCUMBER],
                    CutCucumberSkill.IDENTITY_QUATERNION)

            move_up_above_cucumber_skill.add_feedback_controller_params([600, 50])
            move_up_above_cucumber_skill.add_termination_params([1.0])
            result = cut_cucumber_skill.execute_skill(move_up_above_cucumber_skill, client)

            end_x = result.execution_result[12]
            end_y = result.execution_result[13]
            end_z = result.execution_result[14]

            # correct_pose_skill = cut_cucumber_skill.create_skill_for_class(
            #                                         ArmMoveToGoalWithDefaultSensorSkill,
            #                                         'correct_pose')
            # correct_pose_skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
            # correct_pose_skill.add_trajectory_params([3.0] + [-0.0372113,0.999006,0.0241583,0,
            #                                                   0.998733,0.0379922,-0.0327119,0,
            #                                                   -0.0335979,0.0229109,-0.999173,0,
            #                                                   end_x,end_y,end_z,1])  # Run Time (1) and Desired End Effector Pose(16)
            # correct_pose_skill.add_feedback_controller_params([600, 50]) # translational stiffness, rotational stiffness
            # correct_pose_skill.add_termination_params([1.0]) # buffer time
            # cut_cucumber_skill.execute_skill(correct_pose_skill, client)

            slice_thickness = args.thickness[slice_idx] 
            # Move left above the cucumber
            skill = cut_cucumber_skill.create_skill_for_class(
                ArmRelativeMotionWithDefaultSensorSkill,
                'move_left_to_cut_slice_{}_thick_{:.3f}'.format(
                    slice_idx, slice_thickness))
            skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
            skill.add_relative_motion_with_quaternion(
                    1.0,
                    [0., slice_thickness, 0.],
                    CutCucumberSkill.IDENTITY_QUATERNION)

            skill.add_feedback_controller_params([600, 50])
            skill.add_termination_params([1.0])
            cut_cucumber_skill.execute_skill(skill, client)

            # Move to contact
            move_onto_cucumber_skill = cut_cucumber_skill.create_skill_for_class(
                    ArmRelativeMotionToContactWithDefaultSensorSkill,
                    'move_onto_cucumber_to_cut_{}'.format(slice_idx))
            move_onto_cucumber_skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
            move_onto_cucumber_skill.add_traj_params_with_quaternion(
                    2.0,
                    [0., 0., -CutCucumberSkill.RELATIVE_MOTION_TO_CONTACT_FOR_CUTTING],
                    CutCucumberSkill.IDENTITY_QUATERNION)
            move_onto_cucumber_skill.add_controller_stiffness_params(600, 50)
            move_onto_cucumber_skill.add_contact_termination_params(
                    1.0, 
                    [10., 10., 7., 10., 10., 10.],
                    [10., 10., 7., 10., 10., 10.])
            cut_cucumber_skill.execute_skill(move_onto_cucumber_skill, client)
            
            # Start DMP cutting for n times
            dmp_skill = cut_cucumber_skill.create_skill_for_class(
                    JointPoseDMPWithDefaultSensorSkill,
                    'cut_dmp')
            dmp_skill.add_initial_sensor_values(dmp_info['phi_j'])  # sensor values
            # y0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            y0 = [-0.282, -0.189, 0.0668, -2.186, 0.0524, 1.916, -1.06273]
            # Run time, tau, alpha, beta, num_basis, num_sensor_values, mu, h, weights
            trajectory_params = [
                    4.0, dmp_info['tau'], dmp_info['alpha'], dmp_info['beta'],
                    float(dmp_info['num_basis']), float(dmp_info['num_sensors'])] \
                            + dmp_info['mu'] \
                            + dmp_info['h'] \
                            + y0 \
                            + np.array(dmp_info['weights']).reshape(-1).tolist()

            dmp_skill.add_trajectory_params(trajectory_params)
            dmp_skill.set_meta_skill_id(slice_idx+1)
            dmp_skill.set_meta_skill_type(1)
            dmp_skill.add_termination_params([1.0])
            num_of_dmps_to_run = args.num_dmps

            if(item == 'tomato'):
                num_of_dmps_to_run = 4
            elif(item == 'cucumber'):
                num_of_dmps_to_run = 3
            elif(item == 'cheese'):
                num_of_dmps_to_run = 2

            for dmp_idx in range(num_of_dmps_to_run):
                result = cut_cucumber_skill.execute_skill(dmp_skill, client)

            '''
            if args.move_in_air:
                move_air_dist = 0.1
                skill = cut_cucumber_skill.create_skill_to_move_to_air(
                        1.0, move_air_dist, CutCucumberSkill.IDENTITY_QUATERNION,
                        "move_to_air_to_avoid_rubbing_board_slice_{}".format(
                            slice_idx))
                cut_cucumber_skill.execute_skill(skill, client)
            '''
            if(args.move_down_after_slicing):
                move_onto_cucumber_skill = cut_cucumber_skill.create_skill_for_class(
                        ArmRelativeMotionToContactWithDefaultSensorSkill,
                        'move_onto_cucumber_to_cut_{}'.format(slice_idx))
                move_onto_cucumber_skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
                move_onto_cucumber_skill.add_traj_params_with_quaternion(
                        2.0,
                        [0., 0., -CutCucumberSkill.RELATIVE_MOTION_TO_CONTACT_FOR_CUTTING],
                        CutCucumberSkill.IDENTITY_QUATERNION)
                move_onto_cucumber_skill.add_controller_stiffness_params(600, 50)
                move_onto_cucumber_skill.add_contact_termination_params(
                        1.0, 
                        [10., 10., 10., 10., 10., 10.],
                        [10., 10., 10., 10., 10., 10.])
                result = cut_cucumber_skill.execute_skill(move_onto_cucumber_skill, client)

            end_z = result.execution_result[14]
            print(end_z)

            result = cut_cucumber_skill.execute_skill(dmp_skill, client)
            move_onto_cucumber_skill = cut_cucumber_skill.create_skill_for_class(
                    ArmRelativeMotionToContactWithDefaultSensorSkill,
                    'move_onto_cucumber_to_cut_{}'.format(slice_idx))
            move_onto_cucumber_skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
            move_onto_cucumber_skill.add_traj_params_with_quaternion(
                    2.0,
                    [0., 0., -CutCucumberSkill.RELATIVE_MOTION_TO_CONTACT_FOR_CUTTING],
                    CutCucumberSkill.IDENTITY_QUATERNION)
            move_onto_cucumber_skill.add_controller_stiffness_params(600, 50)
            move_onto_cucumber_skill.add_contact_termination_params(
                    1.0, 
                    [10., 10., 10., 10., 10., 10.],
                    [10., 10., 10., 10., 10., 10.])
            result = cut_cucumber_skill.execute_skill(move_onto_cucumber_skill, client)

            if(args.move_slices_left):

                move_air_dist = 0.033
                move_to_air_skill = cut_cucumber_skill.create_skill_to_move_to_air(
                                    1.0, move_air_dist,
                                    CutCucumberSkill.IDENTITY_QUATERNION,
                                    "move_to_air_to_avoid_rubbing_board_0")
                cut_cucumber_skill.execute_skill(move_to_air_skill, client)

                #Move cut cucumber piece away from the main cucumber (Stay in place)
                move_cut_piece_away_dist = 0.04
                skill = cut_cucumber_skill.get_move_left_skill(
                        move_cut_piece_away_dist, 
                        time=2.0,
                        desc='move_to_separate_cut_slice_{}_dist_{:.3f}'.format(
                            slice_idx, move_cut_piece_away_dist))
                cut_cucumber_skill.execute_skill(skill, client)

                # Move back to cucumber
                skill = cut_cucumber_skill.create_skill_to_move_to_cucumber(
                        desc='move_left_to_contact_cucumber_after_slice_{}'.format(slice_idx))
                cut_cucumber_skill.execute_skill(skill, client)

        if(not args.move_slices_left):
            move_air_dist = 0.033
            move_to_air_skill = cut_cucumber_skill.create_skill_to_move_to_air(
                                1.0, move_air_dist,
                                CutCucumberSkill.IDENTITY_QUATERNION,
                                "move_to_air_to_avoid_rubbing_board_0")
            cut_cucumber_skill.execute_skill(move_to_air_skill, client)

            move_cut_piece_away_dist = 0.08
            skill = cut_cucumber_skill.get_move_left_skill(
                    move_cut_piece_away_dist, 
                    time=2.0,
                    desc='move_to_separate_cut_slice_{}_dist_{:.3f}'.format(
                        slice_idx, move_cut_piece_away_dist))
            cut_cucumber_skill.execute_skill(skill, client)

        # Finally move to position above cutting board
        # Move to designated position above the cutting board
        skill = cut_cucumber_skill.create_skill_for_class(
                ArmMoveToGoalWithDefaultSensorSkill,
                'move_above_cutting_board_final')
        skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
        skill.add_trajectory_params(
                [3.0] + CutCucumberSkill.POSITION_ABOVE_CUTTING_BOARD)
        skill.add_feedback_controller_params([600, 50])
        skill.add_buffer_time_for_termination(1.0)
        cut_cucumber_skill.execute_skill(skill, client)
