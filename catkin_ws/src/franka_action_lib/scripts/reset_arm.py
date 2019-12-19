import argparse
from frankapy import FrankaArm

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_joints', '-u', action='store_true')
    parser.add_argument('--close_gripers', '-c', action='store_true')
    args = parser.parse_args()

    print('Starting robot')
    fa = FrankaArm()

    if args.use_joints:
        print('Reset with joints')
        fa.reset_joints()
    else:
        print('Reset with pose')
        fa.reset_pose()
    
    if args.close_gripers:
        print('Closing Grippers')
        fa.close_gripper()
    else:
        print('Opening Grippers')
        fa.open_gripper()