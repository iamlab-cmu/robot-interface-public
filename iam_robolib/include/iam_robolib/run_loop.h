#pragma once

#include <cmath>
#include <functional>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <iam_robolib/run_loop_process_info.h>

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

  RunLoopProcessInfo *run_loop_info_=NULL;

  const bool limit_rate_;  // NOLINT(readability-identifier-naming)
  const double cutoff_frequency_; // NOLINT(readability-identifier-naming)
  uint32_t elapsed_time_;

  // Managed memory segments
  boost::interprocess::managed_shared_memory managed_shared_memory_1_{};
  boost::interprocess::managed_shared_memory managed_shared_memory_2_{};

  // Managed memory objects
  boost::interprocess::shared_memory_object shared_memory_object_1_{};
  boost::interprocess::shared_memory_object shared_memory_object_2_{};

  boost::interprocess::mapped_region region_1_{};
  boost::interprocess::mapped_region region_2_{};

};
