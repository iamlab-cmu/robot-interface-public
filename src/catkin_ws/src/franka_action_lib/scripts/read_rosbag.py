import sys
import rosbag
import time
import subprocess
import yaml
import rospy
import os
import argparse
import math
from shutil import move

import pprint

import pdb


IMAGE_TOPICS_LIST = ['/camera1/color/image_raw']
DEPTH_TOPICS_LIST = []
# DEPTH_TOPICS_LIST = ['/camera1/depth/image_rect_raw']

ROBOT_STATE_TOPIC = '/robot_state_publisher_node/robot_state'
CLOCK_TOPIC = '/clock'

TARGET_SKILL_DESC = 'random_exploration_after_cut_slice_2_z_2_pos_0.010_time_0.200 type: ArmRelativeMotionToContactWithDefaultSensorSkill,  id: 129'

def status(length, percent):
  sys.stdout.write('\x1B[2K') # Erase entire current line
  sys.stdout.write('\x1B[0E') # Move to the beginning of the current line
  progress = "Progress: ["
  for i in range(0, length):
    if i < length * percent:
      progress += '='
    else:
      progress += ' '
  progress += "] " + str(round(percent * 100.0, 2)) + "%"
  sys.stdout.write(progress)
  sys.stdout.flush()

class RosbagUtils(object):
    def __init__(self, rosbag_path):
        assert os.path.exists(rosbag_path), "Rosbag does not exist"
        self._rosbag_path = rosbag_path

    def get_rosbag_info(self):
        '''Run rosbag info on the rosbag.'''
        info_dict = yaml.load(subprocess.Popen(
            ['rosbag', 'info', '--yaml', self._rosbag_path],
            stdout=subprocess.PIPE).communicate()[0])
        return info_dict

    @staticmethod
    def get_robot_state_as_dict_from_message(msg):
        '''Get robot state as a dictionary from rosbag message.'''
        robot_state_dict = {}
        robot_state_dict['pose_desired'] = msg.pose_desired
        robot_state_dict['pose'] =  msg.pose
        robot_state_dict['joint_torques'] = msg.joint_torques
        robot_state_dict['joint_torques_derivative'] = msg.joint_torques_derivative
        robot_state_dict['joints'] = msg.joints
        robot_state_dict['joints_desired'] = msg.joints_desired
        robot_state_dict['joint_velocities'] = msg.joint_velocities
        robot_state_dict['time_since_skill_started'] = msg.time_since_skill_started
        robot_state_dict['gripper_width'] = msg.gripper_width
        robot_state_dict['gripper_is_grasped'] = msg.gripper_is_grasped

        return robot_state_dict

    def get_robot_state_indexed_by_skill(self):
        '''Get the robot state indexed by skill.'''
        robot_state_by_skill_dict = {}
        for topic, msg, t in rosbag.Bag(self._rosbag_path).read_messages(
                topics=[ROBOT_STATE_TOPIC]):
            skill_desc = msg.skill_description
            assert type(skill_desc) is str, "Incorrect skill description type"
            if robot_state_by_skill_dict.get(skill_desc) is None:
                robot_state_by_skill_dict[skill_desc] = {}

            if skill_desc == TARGET_SKILL_DESC:
                print("Got target skill: {}".format(robot_state['time_since_skill_started']))

            # Append robot_state
            robot_state = RosbagUtils.get_robot_state_as_dict_from_message(msg)
            '''
            if len(skill_desc) > 5:
                if robot_state_by_skill_dict[skill_desc].get(
                        robot_state['time_since_skill_started']) is not None:
                    print("Found state for skill: {} at same time: {}".format(
                        skill_desc, 
                        robot_state['time_since_skill_started']))
                    continue
            '''

            skill_dict = robot_state_by_skill_dict[skill_desc]
            skill_dict[robot_state['time_since_skill_started']] = {}
            skill_at_t_dict = skill_dict[robot_state['time_since_skill_started']]
            skill_at_t_dict['robot_state'] = robot_state
            skill_at_t_dict['header'] = {
                    'seq': msg.header.seq,
                    'secs': msg.header.stamp.secs,
                    'nsecs': msg.header.stamp.nsecs
            }
        return robot_state_by_skill_dict


def main(args):
    parser = argparse.ArgumentParser(
            description='Get Robot state and images from bagfile.')
    parser.add_argument('--bagfile', type=str, required=True,
                        help='input bag file')
    args = parser.parse_args()

    # Get bag duration
    bagfile = args.bagfile

    rosbag_utils = RosbagUtils(bagfile)
    rosbag_info_dict = rosbag_utils.get_rosbag_info()
    pprint.pprint(rosbag_info_dict)

    robot_state_by_skill_dict = rosbag_utils.get_robot_state_indexed_by_skill()

    pdb.set_trace()

    images_list, images_time__list = [], []
    for topic, msg, t in rosbag.Bag(bagfile).read_messages(
            topics=['/camera1/color/image_raw',
                    '/robot_state_publisher_node/robot_state']):
        if topic == '/camera1/color/image_raw':
            pdb.set_trace()
        elif topic == '/robot_state_publisher_node/robot_state':
            pdb.set_trace()


if __name__ == "__main__":
    main(sys.argv[1:])

