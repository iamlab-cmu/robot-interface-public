#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from franka_action_lib.msg import SensorData
from proto import sensor_msg_pb2


def talker():
    rospy.init_node('talker', anonymous=True)
    sensor_msg = sensor_msg_pb2.JointPositions()   
    sensor_msg.id = 1
    sensor_msg.q1 = 0.5
    sensor_msg.q2 = 0.5
    sensor_msg.q3 = 0.5
    sensor_msg.q4 = 0.5
    sensor_msg.q5 = 0.5
    sensor_msg.q6 = 0.5
    sensor_msg.q7 = 0.5
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

    while(rospy.Time.now() < end_waypoint1):
        rospy.loginfo("will send message 1")
        pub.publish(f_data)
        rate.sleep()
    
    sensor_msg.id = 2
    sensor_msg.q1 = 1
    sensor_msg.q2 = 1
    sensor_msg.q3 = 1
    sensor_msg.q4 = 1
    sensor_msg.q5 = 1
    sensor_msg.q6 = 1
    sensor_msg.q7 = 1
    sensor_data_bytes = sensor_msg.SerializeToString()
    f_data = SensorData()
    f_data.sensorDataInfo = "JointPositions"   
    f_data.size = len(sensor_data_bytes)
    f_data.sensorData = sensor_data_bytes
    
    print('data_in_bytes: type: {}, data: {}'.format(
    type(sensor_data_bytes), sensor_data_bytes))

    while not rospy.is_shutdown():
        rospy.loginfo("will send message 2")
        pub.publish(f_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass