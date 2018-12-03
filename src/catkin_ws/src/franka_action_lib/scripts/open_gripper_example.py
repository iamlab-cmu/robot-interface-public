#! /usr/bin/env python

import roslib
roslib.load_manifest('franka_action_lib')
import rospy
import actionlib

from franka_action_lib.msg import ExecuteSkillAction, ExecuteSkillGoal

from skill_list import BaseSkill
from skill_list import OpenGripperWithDefaultSensorSkill

def feedback_callback(feedback):
    print(feedback)

if __name__ == '__main__':
    rospy.init_node('example_execute_skill_action_client')
    client = actionlib.SimpleActionClient('/execute_skill_action_server_node/execute_skill', ExecuteSkillAction)
    client.wait_for_server()

    skill = OpenGripperWithDefaultSensorSkill(1)
    skill.add_initial_sensor_values([1, 3, 5, 7, 8])  # random
    skill.add_trajectory_params([0.05, 0.025, 2100])
    goal = client.create_goal()
    client.send_goal(goal, feedback_cb=lambda x: skill.feedback_callback(x))


    while not rospy.is_shutdown() and done != True:
        done = client.wait_for_result(rospy.Duration.from_sec(5.0))