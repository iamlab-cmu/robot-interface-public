import numpy as np

def truncate_expert_data(expert_data):

    for trajectory_num in expert_data.keys():
        start_trajectory_index = 0
        end_trajectory_index = 180
        q = expert_data[trajectory_num]['q']
        num_saved_data_points = q.shape[0]
        previous_joint_positions = q[0,:]
        for i in range(1,num_saved_data_points):
            current_joint_positions = q[i,:]
            if(np.linalg.norm(previous_joint_positions - current_joint_positions) > 0.001):
                start_trajectory_index = i - 1
                break
                
        previous_joint_positions = q[179,:]
        for i in reversed(range(0,num_saved_data_points-1)):
            current_joint_positions = q[i,:]
            if(np.linalg.norm(previous_joint_positions - current_joint_positions) > 0.001):
                end_trajectory_index = i + 2
                break

        print(start_trajectory_index)
        print(end_trajectory_index)

        expert_data[trajectory_num]['time'] = expert_data[trajectory_num]['time'][start_trajectory_index:end_trajectory_index]
        expert_data[trajectory_num]['pose_desired'] = expert_data[trajectory_num]['pose_desired'][start_trajectory_index:end_trajectory_index]
        expert_data[trajectory_num]['robot_state'] = expert_data[trajectory_num]['robot_state'][start_trajectory_index:end_trajectory_index]
        expert_data[trajectory_num]['tau_j'] = expert_data[trajectory_num]['tau_j'][start_trajectory_index:end_trajectory_index]
        expert_data[trajectory_num]['d_tau_j'] = expert_data[trajectory_num]['d_tau_j'][start_trajectory_index:end_trajectory_index]
        expert_data[trajectory_num]['q'] = expert_data[trajectory_num]['q'][start_trajectory_index:end_trajectory_index]
        expert_data[trajectory_num]['dq'] = expert_data[trajectory_num]['dq'][start_trajectory_index:end_trajectory_index]
        
    return expert_data