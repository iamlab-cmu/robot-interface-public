#ifndef IAM_ROBOLIB_ROBOTS_UR5E_ROBOT_H_
#define IAM_ROBOLIB_ROBOTS_UR5E_ROBOT_H_

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <iostream>

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/factory.h"
#include "ur_modern_driver/ur/messages.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/producer.h"
#include "ur_modern_driver/ur/rt_state.h"
#include "ur_modern_driver/ur/rt_consumer.h"
#include "ur_modern_driver/ur/state.h"

#include "iam_robolib/robots/robot.h"

class IgnorePipelineStoppedNotifier : public INotifier
{
public:
    void started(std::string name){
        LOG_INFO("Starting pipeline %s", name.c_str());
    }
    void stopped(std::string name){
        LOG_INFO("Stopping pipeline %s", name.c_str());
    }
};

class ShutdownOnPipelineStoppedNotifier : public INotifier
{
public:
    void started(std::string name){
        LOG_INFO("Starting pipeline %s", name.c_str());
    }
    void stopped(std::string name){
        LOG_INFO("Shutting down on stopped pipeline %s", name.c_str());
        exit(1);
    }
};

/*std::string getLocalIPAccessibleFromHost(std::string &host, int transmit_port)
{
  URStream stream(host, transmit_port);
  return stream.connect() ? stream.getIP() : std::string();
}*/

class UR5ERobot : public Robot
{
 public:
  UR5ERobot(std::string &robot_ip, RobotType robot_type) : Robot(robot_ip, robot_type),
                                                           factory_(robot_ip),
                                                           rt_receive_stream_(robot_ip, UR_RT_RECEIVE_PORT_),
                                                           rt_transmit_stream_(robot_ip, UR_RT_TRANSMIT_PORT_)

  {
    // std::string local_ip(getLocalIPAccessibleFromHost(robot_ip, UR_RT_TRANSMIT_PORT_));

    // RT packets
    auto rt_parser = factory_.getRTParser();
    URProducer<RTPacket> rt_prod(rt_receive_stream_, *rt_parser);

    RTConsumer rt_consumer(false);

    INotifier *notifier(nullptr);
    if (true)
    {
      LOG_INFO("Notifier: Pipeline disconnect will shutdown the node");
      notifier = new ShutdownOnPipelineStoppedNotifier();
    }
    else
    {
      LOG_INFO("Notifier: Pipeline disconnect will be ignored.");
      notifier = new IgnorePipelineStoppedNotifier();
    }

    vector<IConsumer<RTPacket> *> rt_vec{ &rt_consumer };
    MultiConsumer<RTPacket> rt_cons(rt_vec);
    Pipeline<RTPacket> rt_pl(rt_prod, rt_cons, "RTPacket", *notifier);

    //RTPublisher rt_pub(args.prefix, args.base_frame, args.tool_frame, args.use_ros_control);
    
    auto rt_commander = factory_.getCommander(rt_transmit_stream_);

    LOG_INFO("Starting main loop");

    rt_pl.run();

    rt_transmit_stream_.connect();

    std::array<double, 6> pose = {0.13339,-0.49242,0.48877,0.0,3.136,0.0};
    //std::array<double, 6> joints = {1.57,-1.57,1.57,-1.57,-1.57,0};
    double tool_acceleration = 1.0;
    //double gain = 300.0;

    int i = 0;

    while(i < 10000)
    {
      //joints[5] += 0.0003;
      pose[2] -= 0.00001;
      //rt_commander->servoj(joints, gain);

      rt_commander->movel(pose, tool_acceleration);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      i++;
      //std::cout << i << std::endl;
    }

    rt_commander->stopl();

    LOG_INFO("Stopping, shutting down pipelines");

    rt_pl.stop();
  }

 private:
  const int UR_RT_TRANSMIT_PORT_ = 30003;
  const int UR_RT_RECEIVE_PORT_ = 30013;

  URFactory factory_;
  URStream rt_receive_stream_;
  URStream rt_transmit_stream_;

};

#endif  // IAM_ROBOLIB_ROBOTS_UR5E_ROBOT_H_