#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include "franka_action_lib/SensorData.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_sensor_publisher");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<franka_action_lib::SensorData>("dummy_time", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    //std_msgs::Float64 f_data;
   franka_action_lib::SensorData f_data;

//    std::stringstream ss;
//    ss << "hello world " << count;
    //f_data.data = ros::Time::now().toSec();


    f_data.sensorDataInfo = "Torque";
    f_data.size =10;
    f_data.sensorData = {0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0};

//    ROS_INFO("%s", msg.data.c_str());
      ROS_INFO("%f", f_data.sensorData[0]);

      /**
       * The publish() function is how you send messages. The parameter
       * is the message object. The type of this object must agree with the type
       * given as a template parameter to the advertise<>() call, as was done
       * in the constructor above.
       */
    chatter_pub.publish(f_data);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}