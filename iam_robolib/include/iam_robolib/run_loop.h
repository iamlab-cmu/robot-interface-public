#pragma once

#include <chrono>
#include <cmath>
#include <functional>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <iam_robolib/run_loop_process_info.h>

// TODO(Mohit): Fix this, CANNOT do private imports in public headers. FML.
#include "../../src/skill_info_manager.h"

void setCurrentThreadToRealtime(bool throw_on_error);

// TODO(Mohit): Add a namespace to these declarations.
// TODO(Mohit): Need to make this an interface.
class RunLoop {
 public: // TODO(Mohit): Maybe we should pass in a pointer to the main loop interface?
  RunLoop() : limit_rate_(false), cutoff_frequency_(0.0), elapsed_time_(0.0),
              process_info_requires_update_(false) {}

  // Todo(Mohit): Implement this!!! We should free up the shared memory correctly.
  // ~RunLoop();

  bool init();
  /**
   *  Start the RunLoop.
   *
   *  This will allocate the shared memory buffers i.e., shared memory object and
   *  shared memory segment used to communicate between the actionlib interface and
   *  the real time loop.
   */
  void start();

  void stop();

  /**
   *  Update the currently executing task. Maybe we should pass in the TaskInfo or
   *  it should return some task info from it.
   */
  bool update();

  /**
   *  Start running the real time loop.
   */
  void run();

 private:

  // MotionGenerator motion_generator;
  SkillInfoManager skill_manager_{};

  // If this flag is true at every loop we will try to get the lock and update
  // process info.
  bool process_info_requires_update_;
  boost::interprocess::interprocess_mutex *run_loop_info_mutex_=NULL;
  RunLoopProcessInfo *run_loop_info_=NULL; const bool limit_rate_;  // NOLINT(readability-identifier-naming)


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

  /**
   *  Start executing new task.
   */
  void start_new_task();

  /**
   *  Finish current executing task.
   *
   * TODO(Mohit): Pass in the current Task Info object to it or return from it?
   */
  void finish_current_task();

  /**
   * Update process info in the shared memory to reflect run-loop's
   * current status.
   */
  void update_process_info();

};
