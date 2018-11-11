#pragma once

#include <ros/node_handle.h>
#include <ros/time.h>

// TODO(Mohit): Add a namespace to these declarations.
// TODO(Mohit): Need to make this an interface.
class RunLoop {
 public:
  // TODO(Mohit): Maybe we should pass in a pointer to the main loop interface?
  bool init(ros::NodeHandle& node_handle);
  void starting(const ros::Time&);
  void update(const ros::Time&, const ros::Duration& period);
  void run();

 private:
  ros::Duration elapsed_time_;
  ros::NodeHandle& node_handle;
};
