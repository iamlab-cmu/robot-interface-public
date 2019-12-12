import rospy
from std_msgs.msg import String
from franka_action_lib.msg import SensorData
from proto import sensor_msg_pb2

def talker():
    sensor_msg = sensor_msg_pb2.BoundingBox()
    data_in_bytes=sensor_msg.SerializeToString()

    print('data_in_bytes',data_in_bytes)
    print(type(data_in_bytes)

    pub = rospy.Publisher('dummy_time', SensorData, queue_size=1000)
    f_data = SensorData()
    f_data.sensorDataInfo = "BoundingBox"   

    f_data.SensorData = data_in_bytes
    #f_data.size =len10

    # rospy.init_node('dummy_sensor_bytes_publisher', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
       

    #     print('data_in_bytes',data_in_bytes)

    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(f_data)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass