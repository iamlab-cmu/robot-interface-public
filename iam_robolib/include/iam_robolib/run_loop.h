#pragma once

#include <cmath>
#include <functional>
#include <boost/interprocess/managed_shared_memory.hpp>

// #include <libiam/motion_generator.h>

void setCurrentThreadToRealtime(bool throw_on_error);

// TODO(Mohit): Add a namespace to these declarations.
// TODO(Mohit): Need to make this an interface.
class RunLoop {
 public: // TODO(Mohit): Maybe we should pass in a pointer to the main loop interface?
  RunLoop() : limit_rate_(false), cutoff_frequency_(0.0), elapsed_time_(0.0) {}
  // ~RunLoop();

  bool init();
  void start();
  void stop();
  void update();
  void run();

 private:

  // MotionGenerator motion_generator;

  const bool limit_rate_;  // NOLINT(readability-identifier-naming)
  const double cutoff_frequency_; // NOLINT(readability-identifier-naming)
  uint32_t elapsed_time_;
  boost::interprocess::managed_shared_memory managed_shared_memory_1_{} ;
  boost::interprocess::managed_shared_memory managed_shared_memory_2_();

};
