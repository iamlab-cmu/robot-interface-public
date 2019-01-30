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

static const int UR_RT_TRANSMIT_PORT = 30003;
static const int UR_RT_RECEIVE_PORT = 30013;
static std::string ROBOT_IP("192.168.1.10");

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

std::string getLocalIPAccessibleFromHost(std::string &host)
{
  URStream stream(host, UR_RT_TRANSMIT_PORT);
  return stream.connect() ? stream.getIP() : std::string();
}

int main(int argc, char **argv)
{
  std::string local_ip(getLocalIPAccessibleFromHost(ROBOT_IP));

  URFactory factory(ROBOT_IP);
  //vector<Service *> services;

  // RT packets
  auto rt_parser = factory.getRTParser();
  URStream rt_receive_stream(ROBOT_IP, UR_RT_RECEIVE_PORT);
  URProducer<RTPacket> rt_prod(rt_receive_stream, *rt_parser);

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
  URStream rt_transmit_stream(ROBOT_IP, UR_RT_TRANSMIT_PORT);
  auto rt_commander = factory.getCommander(rt_transmit_stream);

  LOG_INFO("Starting main loop");

  rt_pl.run();

  rt_transmit_stream.connect();

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

  
  // vector<IConsumer<RTPacket> *> rt_vec{ &rt_pub };

  /*
  
  URScriptHandler urscript_handler(*rt_commander);
  services.push_back(&urscript_handler);
  if (args.shutdown_on_disconnect)
  {
    LOG_INFO("Notifier: Pipeline disconnect will shutdown the node");
    notifier = new ShutdownOnPipelineStoppedNotifier();
  }
  else
  {
    LOG_INFO("Notifier: Pipeline disconnect will be ignored.");
    notifier = new IgnorePipelineStoppedNotifier();
  }

  MultiConsumer<RTPacket> rt_cons(rt_vec);
  Pipeline<RTPacket> rt_pl(rt_prod, rt_cons, "RTPacket", *notifier); 
  

  // Message packets
  auto state_parser = factory.getStateParser();
  URStream state_stream(ROBOT_IP, UR_RT_RECEIVE_PORT);
  URProducer<StatePacket> state_prod(state_stream, *state_parser);
  
  MBPublisher state_pub;

  ServiceStopper service_stopper(services);

  vector<IConsumer<StatePacket> *> state_vec{ &state_pub, &service_stopper };
  MultiConsumer<StatePacket> state_cons(state_vec);
  Pipeline<StatePacket> state_pl(state_prod, state_cons, "StatePacket", *notifier);

  LOG_INFO("Starting main loop");

  rt_pl.run();
  state_pl.run();

  auto state_commander = factory.getCommander(state_stream);
  IOService io_service(*state_commander);

  if (action_server)
    action_server->start();

  ros::spin();

  LOG_INFO("ROS stopping, shutting down pipelines");

  rt_pl.stop();
  state_pl.stop();

  LOG_INFO("Pipelines shutdown complete");
  */

  return EXIT_SUCCESS;
}
