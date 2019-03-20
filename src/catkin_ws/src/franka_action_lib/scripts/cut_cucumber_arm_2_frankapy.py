import numpy as np
import math
import rospy
import argparse
import pickle
import os
from autolab_core import RigidTransform, Point
from frankapy import FrankaArm

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--objects', nargs='+', default=['cucumber'])
    parser.add_argument('--thicknesses', nargs='+', type=float, default=[0.01],
                        help='Thickness for cut slices')
    parser.add_argument('--dmp_weights_file_path', type=str, default='')
    args = parser.parse_args()

    print(os.path.dirname(os.path.realpath(__file__)))

    print('Starting robot')
    fa = FrankaArm(rosnode_name='franka_arm_client_2',
                   execute_skill_action_server_name='/execute_skill_action_server_node_2/execute_skill',
                   robot_state_server_name='/get_current_robot_state_server_node_2/get_current_robot_state_server',
                   robolib_status_server_name='/get_current_robolib_status_server_node_2/get_current_robolib_status_server');

    if not args.dmp_weights_file_path:
        args.dmp_weights_file_path = os.path.dirname(os.path.realpath(__file__)) + '/../../../../../trained_weights/weights_tau_0.75_alpha_5.00_basis_5_sensors_2.pkl'

    file = open(args.dmp_weights_file_path,"rb")
    dmp_info = pickle.load(file)

    for item in args.objects:
        print('Reset with pose')
        initial_position = RigidTransform(rotation=np.array([[0,1,0],[1,0,0],[0,0,-1]]), translation=np.array([0.5,-0.2,0.2]), from_frame='franka_tool')
        fa.goto_pose(initial_position)

        print('Reset with joints')
        initial_joints = [-0.3267232182193221, 0.1908027676045647, -0.05570736447318772, -2.2245624597826876, 0.024569029312127157, 2.3955545955234103, -1.1778217117166347]
        fa.goto_joints(initial_joints, duration=3)
        
        print('Going down to cutting board')    
        cutting_board_location = RigidTransform(rotation=np.array([[0,1,0],[1,0,0],[0,0,-1]]), translation=np.array([0.5,-0.2,0.03]), from_frame='franka_tool')
        cutting_board_vertical_contact_forces = [10.0, 10.0, 7.0, 10.0, 10.0, 10.0]
        fa.goto_pose(cutting_board_location, stop_on_contact_forces=cutting_board_vertical_contact_forces)

        print('Move up a little bit to avoid scraping the cutting board')    
        relative_up_3cm = RigidTransform(rotation=np.array([[1,0,0],[0,1,0],[0,0,1]]), translation=np.array([0.0,0.0,0.03]), from_frame='franka_tool', to_frame='franka_tool')
        fa.goto_pose_delta(relative_up_3cm, duration=3.0)

        print('Move left to avoid scraping the cutting board')  
        horizontal_contact_forces = [10.0, 5.0, 10.0, 10.0, 10.0, 10.0]  
        relative_left_33cm = RigidTransform(rotation=np.array([[1,0,0],[0,1,0],[0,0,1]]), translation=np.array([0.0,0.33,0.02]), from_frame='franka_tool', to_frame='franka_tool')
        fa.goto_pose_delta(relative_left_33cm, stop_on_contact_forces=horizontal_contact_forces)

        num_slices_to_cut = len(args.thicknesses)
        for slice_idx in range(num_slices_to_cut):

            if(item == "cucumber"):
                relative_vertical_height = 0.1
                object_vertical_contact_forces = [10.0, 10.0, 7.0, 10.0, 10.0, 10.0]
                num_dmps = 3
            elif(item == "tomato"):
                relative_vertical_height = 0.1
                object_vertical_contact_forces = [10.0, 10.0, 7.0, 10.0, 10.0, 10.0]
                num_dmps = 4
            else:
                relative_vertical_height = 0.1
                object_vertical_contact_forces = [10.0, 10.0, 7.0, 10.0, 10.0, 10.0]
                num_dmps = 3
            
            print('Move up above ' + item)
            relative_move_up = RigidTransform(rotation=np.array([[1,0,0],[0,1,0],[0,0,1]]), translation=np.array([0.0,0.0,relative_vertical_height]), from_frame='franka_tool', to_frame='franka_tool')
            fa.goto_pose_delta(relative_move_up)

            print('Move left ' + str(args.thicknesses[slice_idx]) + 'm')
            relative_move_left = RigidTransform(rotation=np.array([[1,0,0],[0,1,0],[0,0,1]]), translation=np.array([0.0,args.thicknesses[slice_idx],0.0]), from_frame='franka_tool', to_frame='franka_tool')
            fa.goto_pose_delta(relative_move_left)

            print('Move down to contact ' + item)
            relative_move_down = RigidTransform(rotation=np.array([[1,0,0],[0,1,0],[0,0,1]]), translation=np.array([0.0,0.0,-relative_vertical_height]), from_frame='franka_tool', to_frame='franka_tool')
            fa.goto_pose_delta(relative_move_down, stop_on_contact_forces=object_vertical_contact_forces)

            for i in range(num_dmps):
                fa.execute_dmp(dmp_info, slice_idx+1, duration=4.0)

            print('Move up a little bit to avoid scraping the cutting board')    
            relative_up_3cm = RigidTransform(rotation=np.array([[1,0,0],[0,1,0],[0,0,1]]), translation=np.array([0.0,0.0,0.03]), from_frame='franka_tool', to_frame='franka_tool')
            fa.goto_pose_delta(relative_up_3cm, duration=3.0)

            interactive_perception_distance = 0.1

            print('Move right ' + str(interactive_perception_distance) + 'm for interactive perception')
            relative_move_right = RigidTransform(rotation=np.array([[1,0,0],[0,1,0],[0,0,1]]), translation=np.array([0.0,-interactive_perception_distance,0.0]), from_frame='franka_tool', to_frame='franka_tool')
            fa.goto_pose_delta(relative_move_right, duration=3.0)            

            print('Move left to avoid scraping the cutting board')  
            relative_left_to_contact = RigidTransform(rotation=np.array([[1,0,0],[0,1,0],[0,0,1]]), translation=np.array([0.0,interactive_perception_distance + 0.03,0.01]), from_frame='franka_tool', to_frame='franka_tool')
            fa.goto_pose_delta(relative_left_to_contact, stop_on_contact_forces=horizontal_contact_forces, duration=3.0)