1. Start roscore in a new terminal
2. Start kinect using command 
`roslaunch kinect2_bridge kinect2_bridge.launch`
3. Start darknet_ros on raspberry pi vnc
`roslaunch darknet_ros yolo_v3.launch`
4. Unlock both arms
`ssh -X iam-lab@iam-space
 firefox`
`ssh -X iam-lab@iam-zotac
 google-chrome`
5. Start main iam robolib on iam-space and iam-zotac
`ssh iam-lab@iam-space
 cdbuild
 ./main_iam_robolib`
`ssh iam-lab@iam-zotac
 cdbuild
 ./main_iam_robolib`
6. Start execute skill action server on iam-space and iam-zotac
`cdbash_scripts
./start_control_pc_no_delay.sh -a iam-zotac -p 1 -c 0`
`cdbash_scripts
./start_control_pc_2_no_delay.sh -a iam-space -p 1 -c 0`

Cheese
python ../pick_and_place_cucumber_arm_with_tongs_and_vision.py --objects cheese

rosrun franka_action_lib cut_cucumber_thickness_2.py --filename ../../../../../../trained_weights/weights_tau_0.75_alpha_5.00_basis_5_sensors_2.pkl --move_in_air 1 --thickness 0.01 0.008 --num_dmps 3 --move_slices_left 1 --move_down_after_slicing 1 --objects cheese

Tomato
python ../pick_and_place_cucumber_arm_with_tongs_and_vision.py --objects tomato

rosrun franka_action_lib cut_cucumber_thickness_2.py --filename ../../../../../../trained_weights/weights_tau_0.75_alpha_5.00_basis_5_sensors_2.pkl --move_in_air 1 --thickness 0.012 0.01 --num_dmps 4 --move_slices_left 1 --move_down_after_slicing 1 --objects tomato

Cucumber
python ../pick_and_place_cucumber_arm_with_tongs_and_vision.py --objects cucumber

rosrun franka_action_lib cut_cucumber_thickness_2.py --filename ../../../../../../trained_weights/weights_tau_0.75_alpha_5.00_basis_5_sensors_2.pkl --move_in_air 1 --thickness 0.015 0.013 0.01 0.02 --num_dmps 3 --move_slices_left 1 --move_down_after_slicing 1 --objects cucumber