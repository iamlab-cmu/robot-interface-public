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

import pdb


IMAGE_TOPICS_LIST = ['/camera1/color/image_raw']
DEPTH_TOPICS_LIST = []
# DEPTH_TOPICS_LIST = ['/camera1/depth/image_rect_raw']

ROBOT_STATE_TOPIC = '/robot_state_publisher_node/robot_state'
CLOCK_TOPIC = '/clock'

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


def main_old(args):
    parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
    parser.add_argument('--bagfile', nargs=1, help='input bag file')
    args = parser.parse_args()

    # Get bag duration

    bagfile = args.bagfile[0]

    info_dict = yaml.load(subprocess.Popen(
        ['rosbag', 'info', '--yaml', bagfile], 
        stdout=subprocess.PIPE).communicate()[0])
    duration = info_dict['duration']
    start_time = info_dict['start']

    orig = os.path.splitext(bagfile)[0] + ".orig.bag"

    move(bagfile, orig)

    last_time = time.clock()
    for topic, msg, t in rosbag.Bag(bagfile).read_messages():

      if time.clock() - last_time > .1:
          percent = (t.to_sec() - start_time) / duration
          status(40, percent)
          last_time = time.clock()

      # This also replaces tf timestamps under the assumption
      # that all transforms in the message share the same timestamp
      if topic == "/tf" and msg.transforms:
        # Writing transforms to bag file 1 second ahead of time to ensure availability
        diff = math.fabs(msg.transforms[0].header.stamp.to_sec() - t.to_sec())
        outbag.write(
                topic,
                msg,
                msg.transforms[0].header.stamp - rospy.Duration(1) \
                        if diff < args.max_offset else t)
      elif msg._has_header:
        diff = math.fabs(msg.header.stamp.to_sec() - t.to_sec())
        outbag.write(topic, msg, msg.header.stamp if diff < args.max_offset else t)
      else:
        outbag.write(topic, msg, t)
    print "\ndone"

class RosbagUtils(object):
    def __init__(self, rosbag_path):
        assert os.path.exists(rosbag_path), "Rosbag does not exist"
        self._rosbag_path = rosbag_path

    def get_rosbag_info(self):
        '''Run rosbag info on the rosbag.'''
        info_dict = yaml.load(subprocess.Popen(
            ['rosbag', 'info', '--yaml', bagfile],
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
        for topic, msg, t in rosbag.Bag(bagfile).read_messages(
                topics=[ROBOT_STATE_TOPIC]):
            skill_desc = msg.skill_description
            assert type(skill_desc) is str, "Incorrect skill description type"
            if robot_state_by_skill_dict.get(skill_desc) is None:
                robot_state_by_skill_dict[skill_desc] = []

            # Append robot_state
            robot_state = RosbagUtils.get_robot_state_as_dict_from_message(msg)
            assert robot_state_by_skill_dict[skill_desc].get(
                    robot_state['time_since_skill_started']) is None, \
                            "Cannot have two states at same time"

            skill_dict = robot_state_by_skill_dict[skill_desc]
            skill_dict[robot_state['time_since_skill_started']] = {}
            skill_at_t_dict = skill_dict[robot_state['time_since_skill_started']]
            skill_at_t_dict['robot_state'] = robot_state
            skill_at_t_dict['header'] = {
                    'seq': msg.header.seq,
                    'secs': msg.header.stamp.secs,
                    'nsecs': msg.header.stamp.nsecs
            }



def main(args):
    parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
    parser.add_argument('--bagfile', nargs=1, help='input bag file')
    args = parser.parse_args()

    # Get bag duration

    bagfile = args.bagfile[0]

    info_dict = yaml.load(subprocess.Popen(
        ['rosbag', 'info', '--yaml', bagfile],
        stdout=subprocess.PIPE).communicate()[0])
    duration = info_dict['duration']
    start_time = info_dict['start']

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

