#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from franka_action_lib.msg import SensorData
from proto import sensor_msg_pb2
import math

def create_sensor_data_message():
    rospy.init_node('talker', anonymous=True)

    HOME_JOINTS = [-0.0231353,-0.823394,0.132174,-2.60391,0.00749405,1.77283,0.804603]

    sensor_msg = sensor_msg_pb2.JointSensorInfo()   
    sensor_msg.id = 1
    sensor_msg.q1 = HOME_JOINTS[0]
    sensor_msg.q2 = HOME_JOINTS[1]
    sensor_msg.q3 = HOME_JOINTS[2]
    sensor_msg.q4 = HOME_JOINTS[3]
    sensor_msg.q5 = HOME_JOINTS[4]
    sensor_msg.q6 = HOME_JOINTS[5]
    sensor_msg.q7 = HOME_JOINTS[6]
    begin=rospy.Time.now()
    sensor_msg.timestamp=int(begin.to_sec())
    waypoint1_duration = rospy.Duration(5) #30 sec
    end_waypoint1 = begin + waypoint1_duration
    sensor_data_bytes = sensor_msg.SerializeToString()

    f_data = SensorData()
    f_data.sensorDataInfo = "JointPositions"   
    f_data.size = len(sensor_data_bytes)
    f_data.sensorData = sensor_data_bytes
    print('data_in_bytes: type: {}, data_len: {}'.format(
    type(sensor_data_bytes), len(sensor_data_bytes)))

    pub = rospy.Publisher('dummy_sensor', SensorData, queue_size=1000)
    rate = rospy.Rate(1)
    print('updated joint positions 1 - robot should not change!')
    while(rospy.Time.now() < end_waypoint1):
        rospy.loginfo("will send message 1")
        pub.publish(f_data)
        rate.sleep()

def talker():
    HOME_JOINTS = [0, -math.pi / 4, 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]


    rospy.init_node('talker', anonymous=True)
    sensor_msg = sensor_msg_pb2.JointPositions()   
    sensor_msg.id = 1
    sensor_msg.q1 = HOME_JOINTS[0]
    sensor_msg.q2 = HOME_JOINTS[1]
    sensor_msg.q3 = HOME_JOINTS[2]
    sensor_msg.q4 = HOME_JOINTS[3]
    sensor_msg.q5 = HOME_JOINTS[4]
    sensor_msg.q6 = HOME_JOINTS[5]
    sensor_msg.q7 = HOME_JOINTS[6]
    begin=rospy.Time.now()
    sensor_msg.timestamp=begin.to_sec()
    waypoint1_duration = rospy.Duration(5) #30 sec
    end_waypoint1 = begin + waypoint1_duration
    sensor_data_bytes = sensor_msg.SerializeToString()
    

    f_data = SensorData()
    f_data.sensorDataInfo = "JointPositions"   
    f_data.size = len(sensor_data_bytes)
    f_data.sensorData = sensor_data_bytes
    print('data_in_bytes: type: {}, data: {}'.format(
    type(sensor_data_bytes), sensor_data_bytes))
    
    pub = rospy.Publisher('dummy_sensor', SensorData, queue_size=1000)
    rate = rospy.Rate(1)
    print('updated joint positions 1 - robot should not change!')
    while(rospy.Time.now() < end_waypoint1):
        rospy.loginfo("will send message 1")
        pub.publish(f_data)
        rate.sleep()
    
    sensor_msg.id = 2
    sensor_msg.q1 = HOME_JOINTS[0] + 0.2
    sensor_msg.q2 = HOME_JOINTS[1] + 0.2
    sensor_msg.q3 = HOME_JOINTS[2] + 0.2
    sensor_msg.q4 = HOME_JOINTS[3] + 0.2
    sensor_msg.q5 = HOME_JOINTS[4] + 0.2
    sensor_msg.q6 = HOME_JOINTS[5] + 0.2
    sensor_msg.q7 = HOME_JOINTS[6] + 0.2
    sensor_data_bytes = sensor_msg.SerializeToString()
    f_data = SensorData()
    f_data.sensorDataInfo = "JointPositions"   
    f_data.size = len(sensor_data_bytes)
    f_data.sensorData = sensor_data_bytes
    
    print('data_in_bytes: type: {}, data: {}'.format(
    type(sensor_data_bytes), sensor_data_bytes))

    print(" updated joint positions 2")
    while not rospy.is_shutdown():
        rospy.loginfo("will send message 2")
        pub.publish(f_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        create_sensor_data_message()
    except rospy.ROSInterruptException:
        pass
