from frankapy import FrankaArm

if __name__ == '__main__':
    print('Starting robot')
    fa = FrankaArm()

    fa.reset_joints()

    pose = fa.get_pose()
    pose.translation[0] = 0.5

    fa.goto_pose(pose)
    