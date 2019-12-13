import os
import logging
import argparse
from time import sleep

import numpy as np

from autolab_core import RigidTransform, YamlConfig
from visualization import Visualizer3D as vis3d

from perception_utils.apriltags import AprilTagDetector
from perception_utils.realsense import get_first_realsense_sensor

from frankapy import FrankaArm


def subsample(data, rate=0.1):
    idx = np.random.choice(np.arange(len(data)), size=int(rate * len(data)))
    return data[idx]


def make_det_one(R):
    U, _, Vt = np.linalg.svd(R)
    return U @ np.eye(len(R)) @ Vt


def get_closest_grasp_pose(T_tag_world, T_ee_world):
    tag_axes = [
        T_tag_world.rotation[:,0], -T_tag_world.rotation[:,0],
        T_tag_world.rotation[:,1], -T_tag_world.rotation[:,1]
    ]
    x_axis_ee = T_ee_world.rotation[:,0]
    dots = [axis @ x_axis_ee for axis in tag_axes]
    grasp_x_axis = tag_axes[np.argmax(dots)]
    grasp_z_axis = np.array([0, 0, -1])
    grasp_y_axis = np.cross(grasp_z_axis, grasp_x_axis)
    grasp_R = make_det_one(np.c_[grasp_x_axis, grasp_y_axis, grasp_z_axis])
    grasp_translation = T_tag_world.translation + np.array([0, 0, -cfg['cube_size'] / 2])
    return RigidTransform(
        rotation=grasp_R,
        translation=grasp_translation,
        from_frame=T_ee_world.from_frame, to_frame=T_ee_world.to_frame
    )


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.INFO)
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', '-c', type=str, default='cfg/examples/april_tag_pick_place_cfg.yaml')
    parser.add_argument('--no_grasp', '-ng', action='store_true')
    args = parser.parse_args()
    cfg = YamlConfig(args.cfg)

    logging.info('Starting robot')
    fa = FrankaArm()
    fa.reset_joints()
    fa.open_gripper()

    T_ready_world = fa.get_pose()
    T_ready_world.translation[0] += 0.25
    T_ready_world.translation[2] = 0.4

    ret = fa.goto_pose(T_ready_world)

    logging.info('Init camera')
    sensor = get_first_realsense_sensor(cfg['rs'])
    sensor.start()

    logging.info('Detecting April Tags')
    april = AprilTagDetector(cfg['april_tag'])
    intr = sensor.color_intrinsics
    T_tag_camera = april.detect(sensor, intr, vis=cfg['vis_detect'])[0]
    T_camera_ee = RigidTransform(
        translation=np.array(cfg['camera_ee_tra']),
        from_frame=sensor.frame, to_frame=T_ready_world.from_frame
    )
    T_camera_world = T_ready_world * T_camera_ee
    T_tag_world = T_camera_world * T_tag_camera
    logging.info('Tag has translation {}'.format(T_tag_world.translation))

    logging.info('Finding closest orthogonal grasp')
    T_grasp_world = get_closest_grasp_pose(T_tag_world, T_ready_world)
    T_lift = RigidTransform(translation=[0, 0, 0.2], from_frame=T_ready_world.to_frame, to_frame=T_ready_world.to_frame)
    T_lift_world = T_lift * T_grasp_world

    logging.info('Visualizing poses')
    _, depth_im, _ = sensor.frames()
    points_world = T_camera_world * intr.deproject(depth_im)

    vis3d.figure()
    vis3d.points(subsample(points_world.data.T), color=(0,1,0), scale=0.002)
    vis3d.pose(T_ready_world, length=0.05)
    vis3d.pose(T_camera_world, length=0.1)
    vis3d.pose(T_tag_world)
    vis3d.pose(T_grasp_world)
    vis3d.pose(T_lift_world)
    vis3d.show()

    if not args.no_grasp:
        logging.info('Commanding robot')
        fa.goto_pose(T_lift_world)
        fa.goto_pose(T_grasp_world)
        fa.close_gripper()
        fa.goto_pose(T_lift_world)
        sleep(3)
        fa.goto_pose(T_grasp_world)
        fa.open_gripper()
        fa.goto_pose(T_lift_world)
        fa.goto_pose(T_ready_world)

    import IPython; IPython.embed(); exit(0)
