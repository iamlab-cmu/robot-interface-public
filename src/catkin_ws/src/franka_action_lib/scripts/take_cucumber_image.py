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

import pyrealsense2 as rs
import cv2

def feedback_callback(feedback):
    print(feedback) 

def enumerate_connected_devices(context):
    """
    Enumerate the connected Intel RealSense devices
    Parameters:
    -----------
    context               : rs.context()
                         The context created for using the realsense library
    Return:
    -----------
    connect_device : array
                       Array of enumerated devices which are connected to the PC
    """
    connect_device = []
    for d in context.devices:
        if d.get_info(rs.camera_info.name).lower() != 'platform camera':
            connect_device.append(d.get_info(rs.camera_info.serial_number))
    return connect_device

class Device:
    def __init__(self, pipeline, pipeline_profile):
        self.pipeline = pipeline
        self.pipeline_profile = pipeline_profile


class CutCucumberSkill(object):

    def __init__(self, cutting_knife_location_x, image_dir, save_depth=False):
        self.image_dir = image_dir
        self.camera_dirs_to_device = {
                'camera1_color_image_raw': 0,
                'camera2_color_image_raw': 1,
        }
        self.save_depth = save_depth
        if save_depth:
            self.camera_dirs_to_device['camera1_depth_image'] = 0
            self.camera_dirs_to_device['camera2_depth_image'] = 1

        for camera_dir in self.camera_dirs_to_device.keys():
            if not os.path.exists(os.path.join(image_dir, camera_dir)):
                os.makedirs(os.path.join(image_dir, camera_dir))

        self.images_by_camera, self.image_idx_by_camera = {}, {}
        for k in self.camera_dirs_to_device.keys():
            self.images_by_camera[k] = {}
            self.image_idx_by_camera[k] = 0

        # Configure depth and color streams
        self._config = rs.config()
        self._config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 6)
        if self.save_depth:
            self._config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        self._context = rs.context()
        # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self._config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        if self.save_depth:
            self._config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)

        self.available_devices = enumerate_connected_devices(self._context)
        print("Available devices: {}".format(self.available_devices))
        self._enabled_devices = {}

    def enable_device(self, device_serial, enable_ir_emitter):
        """
        Enable an Intel RealSense Device
        Parameters:
        -----------
        device_serial        : string
                             Serial number of the realsense device
        enable_ir_emitter : bool
                            Enable/Disable the IR-Emitter of the device
        """
        pipeline = rs.pipeline()

        # Enable the device
        self._config.enable_device(device_serial)
        pipeline_profile = pipeline.start(self._config)

        # Set the acquisition parameters
        # sensor = pipeline_profile.get_device().first_depth_sensor()
        # sensor.set_option(rs.option.emitter_enabled, 1 if enable_ir_emitter else 0)

        self._enabled_devices[device_serial] = (Device(pipeline, pipeline_profile))

    def save_current_images(self, img_count, dmp_idx=0, before_dmp=True):
        frames = self.ms_get_frames()
        for camera_dir in self.camera_dirs_to_device.keys():
            device_idx = self.camera_dirs_to_device[camera_dir]
            img_idx = self.image_idx_by_camera[camera_dir]
            img_path = os.path.join(self.image_dir, camera_dir,
                                    'frame_{:05d}.jpg'.format(img_idx))
            print("keys: {}".format(frames.keys()))
            if 'depth' in camera_dir and \
                frames[self.available_devices[device_idx]]['depth'] is not None:
                cv2.imwrite(img_path, frames[self.available_devices[device_idx]]['depth'])
            else:
                cv2.imwrite(img_path, frames[self.available_devices[device_idx]]['color'])

            print("Did save image: {}".format(img_path))

            if self.images_by_camera[camera_dir].get(dmp_idx) is None:
                assert before_dmp, "Cannot have after dmp image without before"
                self.images_by_camera[camera_dir][dmp_idx] = {
                        'before': [], 'after': []
                        }
            
            self.images_by_camera[camera_dir][dmp_idx]['before'].append(img_idx)
            self.image_idx_by_camera[camera_dir] += 1 

        # Now save the pickle file
        pkl_path = os.path.join(self.image_dir, 'images_info.pkl')
        with open(pkl_path, 'wb') as pkl_f:
            pickle.dump((self.images_by_camera), pkl_f, protocol=2)
            print("Did save image info pickle: {}".format(pkl_path))

    def ms_get_frames(self):
        image_by_device_dict = {}
        for (serial, device) in self._enabled_devices.items():
            frames = device.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if self.save_depth:
                depth_frame = frames.get_depth_frame()
            color_image = np.asanyarray(color_frame.get_data())

            if self.save_depth: 
                # Colorize depth frame to jet colormap
                depth_color_frame = rs.colorizer().colorize(depth_frame)
                # Convert depth_frame to numpy array to render image in opencv
                depth_color_image = np.asanyarray(depth_color_frame.get_data())
            else:
                depth_color_image = None

            image_by_device_dict[serial] = {
                    'color': color_image,
                    'depth': depth_color_image,
            }
        return image_by_device_dict

    def poll_frames(self):
        """
        Poll for frames from the enabled Intel RealSense devices.  If temporal
        post processing is enabled, the depth stream is averaged over a certain
        amount of frames Parameters: -----------
        """
        frames = {}
        for (serial, device) in self._enabled_devices.items():
            streams = device.pipeline_profile.get_streams()
            print(streams)
            # frameset = rs.composite_frame(rs.frame())
            frameset = rs.frame()
            device.pipeline.poll_for_frames(frameset)
            if frameset.size() == len(streams):
                frames[serial] = {}
                for stream in streams:
                    if (rs.stream.infrared == stream.stream_type()):
                        frame = frameset.get_infrared_frame(stream.stream_index())
                        key_ = (stream.stream_type(), stream.stream_index())
                    else:
                        frame = frameset.first_or_default(stream.stream_type())
                        key_ = stream.stream_type()
                    frames[serial][key_] = frame

        return frames


if __name__ == '__main__':

    rospy.init_node('example_execute_skill_action_client', 
                    log_level=rospy.DEBUG)
    time_now = rospy.Time.now()
    print("Time now: {:.6f}".format(time_now.to_sec()))

    rospy.loginfo("Will start taking images")
    parser = argparse.ArgumentParser(description='Joint DMP Skill Example')
    parser.add_argument('--num_cameras', type=int, default=3,
                        help='Number of cameras to record')
    parser.add_argument('--save_depth', type=int, default=0,
                        help='Save depth images as well')
    parser.add_argument('--image_dir', type=str, 
                        default='/tmp/cut_cucumber_thickness',
                        help='Directory to store h5 file with images.')
    args = parser.parse_args()

    if not os.path.exists(args.image_dir):
        os.makedirs(args.image_dir)

    cutting_knife_location_x = 0.5232
    cut_cucumber_skill = CutCucumberSkill(cutting_knife_location_x, 
                                          args.image_dir,
                                          args.save_depth)
    # Enable camera devices
    for i in range(args.num_cameras):
        device_id = cut_cucumber_skill.available_devices[i]
        cut_cucumber_skill.enable_device(device_id, False)
        print("Did enable device: {}".format(device_id))

    # Buffer some images initially since realsense changes lighting
    for k in range(100):
        frames = cut_cucumber_skill.ms_get_frames()
        assert len(frames.keys()) != 0, "Did not get any camera images" 

    img_count = 0
    while True:
        cut_cucumber_skill.save_current_images(img_count)
        rospy.sleep(0.1)
        img_count = img_count + 1
        if img_count % 10 == 0:
            print("Did save {} images".format(img_count))

