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
#include "../../src/trajectory_generator.h"
#include "../../src/FeedbackController.h"
#include "../../src/TerminationHandler.h"
#include "../../src/RunLoopLogger.h"
#include "../../src/ControlLoopData.h"

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

  static std::atomic<bool> running_skills_;

 private:

  SharedMemoryInfo shared_memory_info_=SharedMemoryInfo();

  franka::Robot robot_;
  franka::Gripper gripper_;

  std::thread print_thread_{};

  SkillInfoManager skill_manager_{};
  RunLoopLogger logger_;
  ControlLoopData control_loop_data_;

  // If this flag is true at every loop we will try to get the lock and update
  // process info.
  bool process_info_requires_update_;
  boost::interprocess::interprocess_mutex *run_loop_info_mutex_=NULL;
  RunLoopProcessInfo *run_loop_info_=NULL;
  const bool limit_rate_;  // NOLINT(readability-identifier-naming)

  const double cutoff_frequency_; // NOLINT(readability-identifier-naming)
  uint32_t elapsed_time_;


  // Managed memory segments
  boost::interprocess::managed_shared_memory managed_shared_memory_{};

  // Managed memory objects
  boost::interprocess::shared_memory_object shared_memory_object_0_{};
  boost::interprocess::shared_memory_object shared_memory_object_1_{};
  boost::interprocess::interprocess_mutex *shared_memory_mutex_0_= nullptr;
  boost::interprocess::interprocess_mutex *shared_memory_mutex_1_= nullptr;


  boost::interprocess::mapped_region region_traj_params_0_{};
  boost::interprocess::mapped_region region_feedback_controller_params_0_{};
  boost::interprocess::mapped_region region_termination_params_0_{};
  boost::interprocess::mapped_region region_timer_params_0_{};

  boost::interprocess::mapped_region region_traj_params_1_{};
  boost::interprocess::mapped_region region_feedback_controller_params_1_{};
  boost::interprocess::mapped_region region_termination_params_1_{};
  boost::interprocess::mapped_region region_timer_params_1_{};

  SharedBuffer traj_gen_buffer_0_=0;
  SharedBuffer feedback_controller_buffer_0_=0;
  SharedBuffer termination_buffer_0_=0;
  SharedBuffer timer_buffer_0_=0;

  SharedBuffer traj_gen_buffer_1_=0;
  SharedBuffer feedback_controller_buffer_1_=0;
  SharedBuffer termination_buffer_1_=0;
  SharedBuffer timer_buffer_1_=0;

  boost::interprocess::shared_memory_object shared_sensor_data_0_{};
  boost::interprocess::shared_memory_object shared_sensor_data_1_{};
  boost::interprocess::interprocess_mutex *shared_sensor_data_mutex_0_= nullptr;
  boost::interprocess::interprocess_mutex *shared_sensor_data_mutex_1_= nullptr;

  boost::interprocess::mapped_region region_traj_sensor_data_0_{};
  boost::interprocess::mapped_region region_feedback_controller_sensor_data_0_{};
  boost::interprocess::mapped_region region_termination_sensor_data_0_{};
  boost::interprocess::mapped_region region_timer_sensor_data_0_{};

  boost::interprocess::mapped_region region_traj_sensor_data_1_{};
  boost::interprocess::mapped_region region_feedback_controller_sensor_data_1_{};
  boost::interprocess::mapped_region region_termination_sensor_data_1_{};
  boost::interprocess::mapped_region region_timer_sensor_data_1_{};

  SharedBuffer traj_gen_sensor_buffer_0_=0;
  SharedBuffer feedback_controller_sensor_buffer_0_=0;
  SharedBuffer termination_sensor_buffer_0_=0;
  SharedBuffer timer_sensor_buffer_0_=0;

  SharedBuffer traj_gen_sensor_buffer_1_=0;
  SharedBuffer feedback_controller_sensor_buffer_1_=0;
  SharedBuffer termination_sensor_buffer_1_=0;
  SharedBuffer timer_sensor_buffer_1_=0;

  boost::interprocess::shared_memory_object shared_execution_result_0_{};
  boost::interprocess::shared_memory_object shared_execution_result_1_{};
  boost::interprocess::interprocess_mutex *shared_execution_result_mutex_0_= nullptr;
  boost::interprocess::interprocess_mutex *shared_execution_result_mutex_1_= nullptr;

  boost::interprocess::mapped_region region_execution_feedback_buffer_0{};
  boost::interprocess::mapped_region region_execution_result_buffer_0_{};
  boost::interprocess::mapped_region region_execution_feedback_buffer_1_{};
  boost::interprocess::mapped_region region_execution_result_buffer_1_{};

  SharedBuffer execution_feedback_buffer_0_=0;
  SharedBuffer execution_result_buffer_0_=0;
  SharedBuffer execution_feedback_buffer_1_=0;
  SharedBuffer execution_result_buffer_1_=0;

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
   *  Finish current executing skill.
   */
  void finish_current_skill(BaseSkill* skill);

  /**
   * Update process info in the shared memory to reflect run-loop's
   * current status.
   */
  void update_process_info();

  /**
   * Get trajectory generator for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   */
  TrajectoryGenerator* get_trajectory_generator_for_skill(int memory_region);

  /**
   * Get feedback controller for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   * @return FeedbackController instance for this skill
   */
  FeedbackController* get_feedback_controller_for_skill(int memory_region);

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
