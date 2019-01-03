#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <iam_robolib_common/run_loop_process_info.h>
#include <iam_robolib_common/SharedMemoryInfo.h>

#include <franka/robot.h>
#include <franka/gripper.h>

// TODO(Mohit): Fix this, CANNOT do private imports in public headers. FML.
#include "../../src/skill_info_manager.h"
#include "../../src/run_loop_logger.h"
#include "../../src/control_loop_data.h"
#include "../../src/run_loop_shared_memory_handler.h"
#include "../../src/trajectory_generator_factory.h"
#include "../../src/feedback_controller_factory.h"
#include "../../src/termination_handler_factory.h"
#include "../../src/definitions.h"

class base_skill;

// Set thread to real time priority.
void setCurrentThreadToRealtime(bool throw_on_error);

// TODO(Mohit): Add a namespace to these declarations.

class run_loop {
 public:
  run_loop(std::mutex& logger_mutex,
          std::mutex& control_loop_data_mutex) : limit_rate_(false),
                                                 cutoff_frequency_(0.0),
                                                 logger_(logger_mutex),
                                                 elapsed_time_(0.0),
                                                 process_info_requires_update_(false),
                                                 control_loop_data_(control_loop_data_mutex),
                                                 robot_("172.16.0.2"),
                                                 gripper_("172.16.0.2") {};

  // Todo(Mohit): Implement this!!! We should free up the shared memory correctly.
  // ~run_loop();

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

  /**
   *  Start running the real time loop on franka
   */
  void run_on_franka();

  /**
   * Get SkillInfo manager.
   */
  SkillInfoManager* getSkillInfoManager();

  /**
   * Did finish skill in meta skill.
   * @param skill
   */
  void didFinishSkillInMetaSkill(base_skill* skill);

  /**
   * Start executing new skill.
   * @param new_skill New skill to start.
   */
  void start_new_skill(base_skill* new_skill);

  /**
   *  Finish current executing skill.
   */
  void finish_current_skill(base_skill* skill);

  static std::atomic<bool> running_skills_;

 private:

  franka::Robot robot_;
  franka::Gripper gripper_;

  std::thread print_thread_{};

  run_loop_shared_memory_handler* shared_memory_handler_ = nullptr;
  SkillInfoManager skill_manager_{};
  run_loop_logger logger_;
  control_loop_data control_loop_data_;

  // If this flag is true at every loop we will try to get the lock and update
  // process info.
  bool process_info_requires_update_;
  const bool limit_rate_;  // NOLINT(readability-identifier-naming)

  const double cutoff_frequency_; // NOLINT(readability-identifier-naming)
  uint32_t elapsed_time_;

  trajectory_generator_factory traj_gen_factory_={};
  feedback_controller_factory feedback_controller_factory_={};
  termination_handler_factory termination_handler_factory_={};

  /**
   * Check if new skill should be started or not. Starting a new skill
   * initializes it's trajectory generator, feedback controller and other
   * associated things.
   *
   * @param old_skill
   * @param new_skill
   * @return True if new skill should be started else false.
   */
  bool should_start_new_skill(base_skill* old_skill, base_skill* new_skill);

  /**
   * Update process info in the shared memory to reflect run-loop's
   * current status.
   */
  void update_process_info();

  /**
   * Setup thread to print data from the real time control loop thread.
   */
  void setup_print_thread();

  /**
   * Setup default collision behavior for robot.
   */
  void setup_robot_default_behavior();
};
