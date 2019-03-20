import numpy as np
import math
import rospy
import argparse
from frankapy import FrankaArm
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from autolab_core import RigidTransform, Point
from perception import CameraIntrinsics

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--objects', nargs='+', default=['cucumber'])
    parser.add_argument('--intrinsics_file_path', type=str, default='/home/klz1/Documents/kinect_registration/calib/kinect2_overhead/kinect_hd_intrinsics.intr')
    parser.add_argument('--transform_file_path', type=str, default='/home/klz1/Documents/kinect_registration/calib/kinect2_overhead/kinect2_overhead_to_world.tf')
    args = parser.parse_args()

    cv_bridge = CvBridge()
    ir_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)

    kinect2_overhead_to_world_transform = RigidTransform.load(args.transform_file_path)

    print('Starting robot')
    fa = FrankaArm()

    print('Opening Grippers')
    fa.open_gripper()

    print('Reset with pose')
    fa.reset_pose()

    print('Reset with joints')
    fa.reset_joints()
        
    for item in args.objects:
        item_max_probability = 0.0
        item_xmin = -1
        item_ymin = -1
        item_xmax = -1
        item_ymax = -1

        width_direction = "x"
        item_width = 0
        item_height = 0

        while item_max_probability < 0.3:
            bounding_boxes_msg = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes)
            
            for bounding_box in bounding_boxes_msg.bounding_boxes:
                if bounding_box.Class == item:
                    if(bounding_box.probability > item_max_probability):
                        item_max_probability = bounding_box.probability
                        item_xmin = bounding_box.xmin
                        item_ymin = bounding_box.ymin
                        item_xmax = bounding_box.xmax
                        item_ymax = bounding_box.ymax

                        if ((item_ymax - item_ymin) < (item_xmax - item_xmin)):
                            width_direction = "y"
                            item_width = item_ymax - item_ymin
                            item_height = item_xmax - item_xmin
                        else:
                            width_direction = "x"
                            item_width = item_xmax - item_xmin
                            item_height = item_ymax - item_ymin

        depth_image_msg = rospy.wait_for_message('/kinect2/hd/image_depth_rect', Image)
        try:
            cv_image = cv_bridge.imgmsg_to_cv2(depth_image_msg)
        except CvBridgeError as e:
            print(e)
        
        item_xcenter = int(item_xmin + item_xmax) / 2 + 30
        item_ycenter = int(item_ymin + item_ymax) / 2

        item_center = Point(np.array([item_xcenter, item_ycenter]), 'kinect2_overhead')
        item_depth = cv_image[item_ycenter, item_xcenter] / 1000.0 + 0.03
        print(item_xcenter)
        print(item_ycenter)
        print(item_depth)

        if(item_depth < 0.5):
            total_item_depth = 0.0
            for i in range(-1,3):
                for j in range(-1,3):
                    total_item_depth += cv_image[item_ycenter + i, item_xcenter + j] / 1000.0 + 0.03
            item_depth = total_item_depth / 9

        item_center_point_in_world = kinect2_overhead_to_world_transform * ir_intrinsics.deproject_pixel(item_depth, item_center)
        
        print(item_center_point_in_world)

        item_center_xmax = Point(np.array([item_xmax, item_ycenter]), 'kinect2_overhead')
        item_center_xmin = Point(np.array([item_xmin, item_ycenter]), 'kinect2_overhead')

        item_center_xmax_point_in_world = kinect2_overhead_to_world_transform * ir_intrinsics.deproject_pixel(item_depth, item_center_xmax)
        item_center_xmin_point_in_world = kinect2_overhead_to_world_transform * ir_intrinsics.deproject_pixel(item_depth, item_center_xmin)

        item_width = item_center_xmax_point_in_world.y - item_center_xmin_point_in_world.y

        print(item_width)

        fa.goto_gripper(item_width + 0.03)

        desired_item_pre_grasp_pose = RigidTransform(rotation=np.array([[1,0,0],[0,-1,0],[0,0,-1]]), translation=np.array([item_center_point_in_world.x, item_center_point_in_world.y, item_center_point_in_world.z + 0.3]), from_frame='franka_tool')
        fa.goto_pose(desired_item_pre_grasp_pose)
        desired_item_grasp_pose = RigidTransform(rotation=np.array([[1,0,0],[0,-1,0],[0,0,-1]]), translation=np.array([item_center_point_in_world.x, item_center_point_in_world.y, item_center_point_in_world.z + 0.2]), from_frame='franka_tool')
        fa.goto_pose(desired_item_grasp_pose)

        if(item == 'tomato' or item =='cheese'):
            fa.goto_gripper(item_width - 0.06, force=1)
        else:
            fa.goto_gripper(item_width - 0.06, force=5)

        fa.goto_pose(desired_item_pre_grasp_pose)

        above_desired_cutting_board_location = RigidTransform(rotation=np.array([[0,1,0],[1,0,0],[0,0,-1]]), translation=np.array([0.654,-0.42,0.3]), from_frame='franka_tool')

        fa.goto_pose(above_desired_cutting_board_location)

        desired_cutting_board_location = RigidTransform(rotation=np.array([[0,1,0],[1,0,0],[0,0,-1]]), translation=np.array([0.654,-0.42,0.24]), from_frame='franka_tool')

        fa.goto_pose(desired_cutting_board_location)

        fa.open_gripper()

        fa.goto_pose(above_desired_cutting_board_location)

        desired_cutting_board_grasping_location = RigidTransform(rotation=np.array([[0,1,0],[math.sin(2 * math.pi / 9),0,-math.cos(2 * math.pi / 9)],[-math.cos(2 * math.pi / 9),0,-math.sin(2 * math.pi / 9)]]), translation=np.array([0.654,-0.24,0.19]), from_frame='franka_tool')

        if(item == 'tomato'):
            desired_cutting_board_grasping_location = RigidTransform(rotation=np.array([[0,1,0],[math.sin(2 * math.pi / 9),0,-math.cos(2 * math.pi / 9)],[-math.cos(2 * math.pi / 9),0,-math.sin(2 * math.pi / 9)]]), translation=np.array([0.654,-0.23,0.19]), from_frame='franka_tool')

        fa.goto_pose(desired_cutting_board_grasping_location)

        fa.goto_gripper(item_width-0.06, force=5)

