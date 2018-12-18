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
#include "../../src/BaseSkill.h"
#include "../../src/skill_info_manager.h"
#include "../../src/TerminationHandler.h"
#include "../../src/RunLoopLogger.h"
#include "../../src/ControlLoopData.h"
#include "../../src/SharedMemoryHandler.h"
#include "../../src/TrajectoryGeneratorFactory.h"
#include "../../src/FeedbackControllerFactory.h"


// SharedBuffer type to share memory (Change size later)
// using SharedBuffer = std::array<float, 1024>;
typedef float *SharedBuffer;

// Set thread to real time priority.
void setCurrentThreadToRealtime(bool throw_on_error);

// TODO(Mohit): Add a namespace to these declarations.

class RunLoop {
 public:
  RunLoop(std::mutex& logger_mutex,
          std::mutex& control_loop_data_mutex) : limit_rate_(false),
                                                 cutoff_frequency_(0.0),
                                                 logger_(logger_mutex),
                                                 elapsed_time_(0.0),
                                                 process_info_requires_update_(false),
                                                 control_loop_data_(control_loop_data_mutex),
                                                 robot_("172.16.0.2"),
                                                 gripper_("172.16.0.2") {};

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
  void didFinishSkillInMetaSkill(BaseSkill* skill);

  /**
   * Start executing new skill.
   * @param new_skill New skill to start.
   */
  void start_new_skill(BaseSkill* new_skill);

  /**
   *  Finish current executing skill.
   */
  void finish_current_skill(BaseSkill* skill);

  static std::atomic<bool> running_skills_;

 private:

  franka::Robot robot_;
  franka::Gripper gripper_;

  std::thread print_thread_{};

  SharedMemoryHandler* shared_memory_handler_ = nullptr;
  SkillInfoManager skill_manager_{};
  RunLoopLogger logger_;
  ControlLoopData control_loop_data_;

  // If this flag is true at every loop we will try to get the lock and update
  // process info.
  bool process_info_requires_update_;
  const bool limit_rate_;  // NOLINT(readability-identifier-naming)

  const double cutoff_frequency_; // NOLINT(readability-identifier-naming)
  uint32_t elapsed_time_;

  TrajectoryGeneratorFactory traj_gen_factory_={};
  FeedbackControllerFactory feedback_controller_factory_={};

  /**
   * Check if new skill should be started or not. Starting a new skill
   * initializes it's trajectory generator, feedback controller and other
   * associated things.
   *
   * @param old_skill
   * @param new_skill
   * @return True if new skill should be started else false.
   */
  bool should_start_new_skill(BaseSkill* old_skill, BaseSkill* new_skill);

  /**
   * Update process info in the shared memory to reflect run-loop's
   * current status.
   */
  void update_process_info();

  /**
   * Get termination handler for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   * @return TermatinationHanndler instance for this skill
   */
  TerminationHandler* get_termination_handler_for_skill(int memory_region);


  /**
   * Setup thread to print data from the real time control loop thread.
   */
  void setup_print_thread();

  /**
   * Setup default collision behavior for robot.
   */
  void setup_robot_default_behavior();
};
