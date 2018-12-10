#include <iam_robolib/run_loop.h>
#include <iam_robolib/duration.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <pthread.h>

#include <cerrno>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <cassert>

#include <franka/exception.h>

#include "counter_trajectory_generator.h"
#include "goal_linear_trajectory_generator.h"
#include "linear_trajectory_generator.h"
#include "linear_trajectory_generator_with_time_and_goal.h"
#include "relative_linear_trajectory_generator.h"
#include "stay_in_initial_position_trajectory_generator.h"
#include "LinearJointTrajectoryController.h"
#include "NoopFeedbackController.h"
#include "torque_feedback_controller.h"

#include "BaseMetaSkill.h"
#include "BaseSkill.h"
#include "ControlLoopData.h"
#include "CustomGainTorqueController.h"
#include "DMPTrajectoryGenerator.h"
#include "FileStreamLogger.h"
#include "FinalJointTerminationHandler.h"
#include "FinalPoseTerminationHandler.h"
#include "GripperOpenSkill.h"
#include "GripperOpenTrajectoryGenerator.h"
#include "NoopTerminationHandler.h"
#include "contact_termination_handler.h"
#include "linear_trajectory_generator_with_time_and_goal_termination_handler.h"
#include "time_termination_handler.h"
#include "JointPoseSkill.h"
#include "JointPoseContinuousSkill.h"
#include "SaveTrajectorySkill.h"

std::atomic<bool> RunLoop::running_skills_{false};

template<typename ... Args>
std::string string_format(const std::string& format, Args ... args )
{
  size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  std::unique_ptr<char[]> buf( new char[ size ] );
  snprintf( buf.get(), size, format.c_str(), args ... );
  return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

void setCurrentThreadToRealtime(bool throw_on_error) {
  // Change prints to exceptions.
  const int thread_priority = sched_get_priority_max(SCHED_FIFO);
  if (thread_priority == -1) {
    std::cout << std::string("libfranka: unable to get maximum possible thread priority: ") +
        std::strerror(errno);
  }
  sched_param thread_param{};
  thread_param.sched_priority = thread_priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
    if (throw_on_error) {
      std::cout << std::string("libfranka: unable to set realtime scheduling: ") +
          std::strerror(errno);
    }
  }
}

bool RunLoop::init() {
  // TODO(Mohit): Initialize memory and stuff.
  bool throw_on_error;
  setCurrentThreadToRealtime(throw_on_error);
}

void RunLoop::start() {
  // Start processing, might want to do some pre-processing 
  std::cout << "start run loop.\n";

  // Create managed shared memory (segments) here.
  boost::interprocess::shared_memory_object::remove(shared_memory_info_.getSharedMemoryNameForObjects().c_str());
  managed_shared_memory_ = boost::interprocess::managed_shared_memory(
          boost::interprocess::create_only,
          shared_memory_info_.getSharedMemoryNameForObjects().c_str(),
          shared_memory_info_.getObjectMemorySize());

  // Add run loop process info to the main loop.
  run_loop_info_ = managed_shared_memory_.construct<RunLoopProcessInfo>
          (shared_memory_info_.getRunLoopInfoObjectName().c_str())
          (1);

  // Add the inter-process mutex into memory. We will grab this each
  // time we want to update anything in the memory.
  run_loop_info_mutex_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
          (shared_memory_info_.getRunLoopInfoMutexName().c_str())
          ();


  /**
   * Create shared memory region for buffer 0.
   */
  const char *shm_name_0 = shared_memory_info_.getSharedMemoryNameForParameters(0).c_str();
  boost::interprocess::shared_memory_object::remove(shm_name_0);
  shared_memory_object_0_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForParameters(0).c_str(),
      boost::interprocess::read_write
  );
  shared_memory_object_0_.truncate(shared_memory_info_.getParameterMemorySize(0));

  region_traj_params_0_=  boost::interprocess::mapped_region(
      shared_memory_object_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTrajectoryParameters(),
      shared_memory_info_.getSizeForTrajectoryParameters()
      );
  traj_gen_buffer_0_ = reinterpret_cast<SharedBuffer>(region_traj_params_0_.get_address());
  region_feedback_controller_params_0_ = boost::interprocess::mapped_region(
      shared_memory_object_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForFeedbackControllerParameters(),
      shared_memory_info_.getSizeForFeedbackControllerParameters()
      );
  feedback_controller_buffer_0_ = reinterpret_cast<SharedBuffer>
      (region_feedback_controller_params_0_.get_address());
  region_termination_params_0_ = boost::interprocess::mapped_region(
      shared_memory_object_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTerminationParameters(),
      shared_memory_info_.getSizeForTerminationParameters()
  );
  termination_buffer_0_ = reinterpret_cast<SharedBuffer>(
      region_termination_params_0_.get_address());
  region_timer_params_0_ = boost::interprocess::mapped_region(
      shared_memory_object_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTimerParameters(),
      shared_memory_info_.getSizeForTimerParameters()
  );
  timer_buffer_0_ = reinterpret_cast<SharedBuffer>(region_timer_params_0_.get_address());


  /**
   * Create shared memory region for buffer 1.
   */

  // Create shared memory objects. for different parameters
  const char *shm_name_1 = shared_memory_info_.getSharedMemoryNameForParameters(1).c_str();
  boost::interprocess::shared_memory_object::remove(shm_name_1);
  shared_memory_object_1_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForParameters(1).c_str(),
      boost::interprocess::read_write
      );

  // Allocate memory
  shared_memory_object_1_.truncate(shared_memory_info_.getParameterMemorySize(1));

  // Allocate regions for each parameter array
  region_traj_params_1_ =  boost::interprocess::mapped_region(
      shared_memory_object_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTrajectoryParameters(),
      shared_memory_info_.getSizeForTrajectoryParameters()
      );
  traj_gen_buffer_1_ = reinterpret_cast<SharedBuffer>(region_traj_params_1_.get_address());
  region_feedback_controller_params_1_ = boost::interprocess::mapped_region(
      shared_memory_object_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForFeedbackControllerParameters(),
      shared_memory_info_.getSizeForFeedbackControllerParameters()
      );
  feedback_controller_buffer_1_ = reinterpret_cast<SharedBuffer>
      (region_feedback_controller_params_1_.get_address());
  region_termination_params_1_ = boost::interprocess::mapped_region(
      shared_memory_object_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTerminationParameters(),
      shared_memory_info_.getSizeForTerminationParameters()
      );
  termination_buffer_1_ = reinterpret_cast<SharedBuffer>(
      region_termination_params_1_.get_address());
  region_timer_params_1_ = boost::interprocess::mapped_region(
      shared_memory_object_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTimerParameters(),
      shared_memory_info_.getSizeForTimerParameters()
      );
  timer_buffer_1_ = reinterpret_cast<SharedBuffer>(region_timer_params_1_.get_address());

  /**
   * Create mutexes for parameter buffers.
   */
  shared_memory_mutex_0_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getParameterMemoryMutexName(0).c_str())
      ();
  shared_memory_mutex_1_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getParameterMemoryMutexName(1).c_str())
      ();

  /**
   * Create shared memory region for sensor data buffer 0.
   */
  // Create shared memory objects. for different parameters
  const char *sensor_name_0 = shared_memory_info_.getSharedMemoryNameForSensorData(0).c_str();
  boost::interprocess::shared_memory_object::remove(sensor_name_0);
  shared_sensor_data_0_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForSensorData(0).c_str(),
      boost::interprocess::read_write
  );
  shared_sensor_data_0_.truncate(shared_memory_info_.getSensorDataMemorySize());
  region_traj_sensor_data_0_ =  boost::interprocess::mapped_region(
      shared_sensor_data_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTrajectorySensorData(),
      shared_memory_info_.getSizeForTrajectorySensorData()
  );
  traj_gen_sensor_buffer_0_ = reinterpret_cast<SharedBuffer>(
      region_traj_sensor_data_0_.get_address());
  region_feedback_controller_sensor_data_0_= boost::interprocess::mapped_region(
      shared_sensor_data_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForFeedbackControllerSensorData(),
      shared_memory_info_.getSizeForFeedbackControllerSensorData()
  );
  feedback_controller_sensor_buffer_0_ = reinterpret_cast<SharedBuffer>
    (region_feedback_controller_sensor_data_0_.get_address());
  region_termination_sensor_data_0_ = boost::interprocess::mapped_region(
      shared_sensor_data_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTerminationSensorData(),
      shared_memory_info_.getSizeForTerminationSensorData()
  );
  termination_sensor_buffer_0_ = reinterpret_cast<SharedBuffer>(
      region_termination_sensor_data_0_.get_address());
  region_timer_sensor_data_0_= boost::interprocess::mapped_region(
      shared_sensor_data_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTimerParameters(),
      shared_memory_info_.getSizeForTimerParameters()
  );
  timer_sensor_buffer_0_ = reinterpret_cast<SharedBuffer>(
      region_timer_sensor_data_0_.get_address());

  /**
   * Create shared memory region for sensor data buffer 1.
   */
  // Create shared memory objects. for different parameters
  const char *sensor_name_1 = shared_memory_info_.getSharedMemoryNameForSensorData(1).c_str();
  boost::interprocess::shared_memory_object::remove(sensor_name_1);
  shared_sensor_data_1_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForSensorData(1).c_str(),
      boost::interprocess::read_write
  );
  shared_sensor_data_1_.truncate(shared_memory_info_.getSensorDataMemorySize());
  region_traj_sensor_data_1_ =  boost::interprocess::mapped_region(
      shared_sensor_data_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTrajectorySensorData(),
      shared_memory_info_.getSizeForTrajectorySensorData()
  );
  traj_gen_sensor_buffer_1_ = reinterpret_cast<SharedBuffer>(
      region_traj_sensor_data_1_.get_address());
  region_feedback_controller_sensor_data_1_= boost::interprocess::mapped_region(
      shared_sensor_data_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForFeedbackControllerSensorData(),
      shared_memory_info_.getSizeForFeedbackControllerSensorData()
  );
  feedback_controller_sensor_buffer_1_ = reinterpret_cast<SharedBuffer>
    (region_feedback_controller_sensor_data_1_.get_address());
  region_termination_sensor_data_1_ = boost::interprocess::mapped_region(
      shared_sensor_data_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTerminationSensorData(),
      shared_memory_info_.getSizeForTerminationSensorData()
  );
  termination_sensor_buffer_1_ = reinterpret_cast<SharedBuffer>(
      region_termination_sensor_data_1_.get_address());
  region_timer_sensor_data_1_= boost::interprocess::mapped_region(
      shared_sensor_data_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTimerParameters(),
      shared_memory_info_.getSizeForTimerParameters()
  );
  timer_sensor_buffer_1_ = reinterpret_cast<SharedBuffer>(
      region_timer_sensor_data_1_.get_address());

  /**
   * Create mutexes for sensor data.
   */
  shared_sensor_data_mutex_0_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getSensorDataMutexName(0).c_str())
      ();
  shared_sensor_data_mutex_1_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getSensorDataMutexName(1).c_str())
      ();


  /**
   * Create memory 0 for execution response.
   */
  const char *results_0 = shared_memory_info_.getSharedMemoryNameForResults(0).c_str();
  boost::interprocess::shared_memory_object::remove(results_0);
  shared_execution_result_0_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForResults(0).c_str(),
      boost::interprocess::read_write
  );
  shared_execution_result_0_.truncate(shared_memory_info_.getExecutionResponseMemorySize());
  region_execution_feedback_buffer_0 =  boost::interprocess::mapped_region(
      shared_execution_result_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForExecutionFeedbackData(),
      shared_memory_info_.getSizeForExecutionFeedbackData()
  );
  execution_feedback_buffer_0_ = reinterpret_cast<SharedBuffer>(region_execution_feedback_buffer_0.get_address());
  region_execution_result_buffer_0_ = boost::interprocess::mapped_region(
      shared_execution_result_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForExecutionReturnData(),
      shared_memory_info_.getSizeForExecutionReturnData()
      );
  execution_result_buffer_0_ = reinterpret_cast<SharedBuffer>(region_execution_result_buffer_0_.get_address());

  /**
   * Create memory 1 for execution response.
   */
  const char *results_1 = shared_memory_info_.getSharedMemoryNameForResults(1).c_str();
  boost::interprocess::shared_memory_object::remove(results_1);
  shared_execution_result_1_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForResults(1).c_str(),
      boost::interprocess::read_write
  );
  shared_execution_result_1_.truncate(shared_memory_info_.getExecutionResponseMemorySize());
  region_execution_feedback_buffer_1_ =  boost::interprocess::mapped_region(
      shared_execution_result_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForExecutionFeedbackData(),
      shared_memory_info_.getSizeForExecutionFeedbackData()
  );
  execution_feedback_buffer_1_ = reinterpret_cast<SharedBuffer>(region_execution_feedback_buffer_1_.get_address());
  region_execution_result_buffer_1_ = boost::interprocess::mapped_region(
      shared_execution_result_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForExecutionReturnData(),
      shared_memory_info_.getSizeForExecutionReturnData()
  );
  execution_result_buffer_1_ = reinterpret_cast<SharedBuffer>(region_execution_result_buffer_1_.get_address());

  /**
   * Create mutexes for execution response.
   */
  shared_execution_result_mutex_0_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getExecutionResponseMutexName(0).c_str())
      ();
  shared_execution_result_mutex_1_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getExecutionResponseMutexName(1).c_str())
      ();

  std::cout << "Did create all shared memory buffers." << std::endl;
}

TrajectoryGenerator* RunLoop::get_trajectory_generator_for_skill(int memory_region) {
  SharedBuffer buffer = traj_gen_buffer_0_;
  if (memory_region == 1) {
    buffer = traj_gen_buffer_1_;
  }
  int traj_gen_id = static_cast<int>(buffer[0]);

  TrajectoryGenerator *traj_generator = nullptr;

  if (traj_gen_id == 1) {
    // Create Counter based trajectory.
    traj_generator = new CounterTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 2) {
    traj_generator = new LinearTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 3) {
    traj_generator = new LinearJointTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 4) {
    traj_generator = new LinearTrajectoryGeneratorWithTimeAndGoal(buffer);
  } else if (traj_gen_id == 5){
    traj_generator = new GripperOpenTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 6) {
    traj_generator = new StayInInitialPositionTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 7) {
    traj_generator = new DMPTrajectoryGenerator(buffer);
  } else if (traj_gen_id == 8) {
    traj_generator = new RelativeLinearTrajectoryGenerator(buffer);
  } else {
    // Cannot create Trajectory generator for this skill. Throw error
    logger_.add_error_log(string_format(
        "Cannot create TrajectoryGenerator with class_id: %d\n", traj_gen_id));
    return nullptr;
  }
  traj_generator->parse_parameters();
  return traj_generator;
}

FeedbackController* RunLoop::get_feedback_controller_for_skill(int memory_region) {
  SharedBuffer buffer = feedback_controller_buffer_0_;
  if (memory_region == 1) {
    buffer = feedback_controller_buffer_1_;
  }
  int feedback_controller_id = static_cast<int>(buffer[0]);

  FeedbackController* feedback_controller = nullptr;
  if (feedback_controller_id == 1) {
    // Create Counter based trajectory.
    feedback_controller = new NoopFeedbackController(buffer);
  } else if (feedback_controller_id == 2) {
    // Create Counter based trajectory.
    feedback_controller = new TorqueFeedbackController(buffer);
  } else if (feedback_controller_id == 3) {
    feedback_controller = new CustomGainTorqueController(buffer);
  } else {
    logger_.add_error_log(string_format(
        "Cannot create FeedbackController with class_id: %d\n", feedback_controller_id));
    return nullptr;
  }
  feedback_controller->parse_parameters();
  return feedback_controller;
}

TerminationHandler* RunLoop::get_termination_handler_for_skill(int memory_region) {
  SharedBuffer buffer = termination_buffer_0_;
  if (memory_region == 1) {
    buffer = termination_buffer_1_;
  }
  int termination_handler_id = static_cast<int>(buffer[0]);

  if (termination_handler_id == 1) {
    // Create Counter based trajectory.
    NoopTerminationHandler *termination_handler = new NoopTerminationHandler(buffer);
    termination_handler->parse_parameters();
    return termination_handler;
  } else if (termination_handler_id == 2) {
    FinalPoseTerminationHandler *termination_handler = new FinalPoseTerminationHandler(buffer);
    termination_handler->parse_parameters();
    return termination_handler;
  } else if (termination_handler_id == 3) {
    FinalJointTerminationHandler *termination_handler = new FinalJointTerminationHandler(buffer);
    termination_handler->parse_parameters();
    return termination_handler;
  } else if (termination_handler_id == 4) {
    LinearTrajectoryGeneratorWithTimeAndGoalTerminationHandler *termination_handler = new LinearTrajectoryGeneratorWithTimeAndGoalTerminationHandler(buffer);
    termination_handler->parse_parameters();
    return termination_handler;
  } else if (termination_handler_id == 5) {
    ContactTerminationHandler *termination_handler = new ContactTerminationHandler(buffer);
    termination_handler->parse_parameters();
    return termination_handler;
  } else if (termination_handler_id == 6) {
    TimeTerminationHandler *termination_handler = new TimeTerminationHandler(buffer);
    termination_handler->parse_parameters();
    return termination_handler;
  } else {
      // Cannot create Trajectory generator for this skill. Throw error
      logger_.add_error_log(string_format(
          "Cannot create TerminationHandler with class_id: %d\n", termination_handler_id));
      return nullptr;
  }
}


void RunLoop::stop() {
  // Maybe call this after exceptions or SIGINT or any Interrupt.
  // Stop the interface gracefully.
}

bool RunLoop::should_start_new_skill(BaseSkill* old_skill, BaseSkill* new_skill) {
  // No new skill to start.
  if (new_skill == nullptr) {
    return false;
  }
  // Old  skill was null, new skill is not null. should start it.
  if (old_skill == nullptr) {
    return true;
  }
  // If new skill is different than old skill, we should start it.
  if (new_skill->get_skill_id() != old_skill->get_skill_id()) {
    return true;
  }

  return false;
}

void RunLoop::start_new_skill(BaseSkill* new_skill) {
  // Generate things that are required here.
  int memory_index = run_loop_info_->get_current_shared_memory_index();
  logger_.add_info_log(string_format("Create skill from memory index: %d\n", memory_index));

  TrajectoryGenerator *traj_generator =
      get_trajectory_generator_for_skill(memory_index);
  std::cout << "Did get traj generator\n";
  FeedbackController *feedback_controller =
      get_feedback_controller_for_skill(memory_index);
  TerminationHandler* termination_handler =
      get_termination_handler_for_skill(memory_index);
  std::cout << "Did get termination_handler\n";
  // Start skill, does any pre-processing if required.
  new_skill->start_skill(&robot_, traj_generator, feedback_controller, termination_handler);

  /**
   * HACK - Used for counter based testing only.
  CounterTrajectoryGenerator *ctg = static_cast<CounterTrajectoryGenerator *>(traj_generator);
  NoopFeedbackController *noopfbc = static_cast<NoopFeedbackController *>(feedback_controller);
  ctg->delta_ = noopfbc->delta_;
  assert(noopfbc->delta_ >= 0.0001);
  assert(ctg->delta_ >= 0.0001);
  */
}


void RunLoop::finish_current_skill(BaseSkill* skill) {
  SkillStatus status = skill->get_current_skill_status();

  if (skill->should_terminate()) {
    skill->set_skill_status(SkillStatus::FINISHED);

    // Write results to memory
    int memory_index = skill->get_skill_id() % 2;

    std::cout << "Writing to execution result buffer number: " << memory_index << std::endl;

    SharedBuffer buffer = execution_result_buffer_0_;
    if (memory_index == 1) {
      buffer = execution_result_buffer_1_;
    }
    skill->write_result_to_shared_memory(buffer, &robot_);
  }

  if (status == SkillStatus::FINISHED) {
    process_info_requires_update_ = true;
  }
  // TODO(Mohit): Do any other-preprocessing if required
}

void RunLoop::update_process_info() {
  BaseSkill* skill = skill_manager_.get_current_skill();
  int current_skill_id = -1;
  if (skill != nullptr) {
    current_skill_id = skill->get_skill_id();
  }
  bool is_executing_skill = skill_manager_.is_currently_executing_skill();

  // Grab the lock and update process info.
  {
    boost::interprocess::scoped_lock<
            boost::interprocess::interprocess_mutex> lock(
                *run_loop_info_mutex_,
                boost::interprocess::defer_lock);
    try {
      if (lock.try_lock()) {
        run_loop_info_->set_is_running_skill(is_executing_skill);

        // We have a skill that we have finished. Make sure we update this in RunLoopProcessInfo.
        if (skill != nullptr && !is_executing_skill) {
          if (run_loop_info_->get_done_skill_id() > current_skill_id) {
            // Make sure get done skill id is not ahead of us.
            logger_.add_error_log(string_format("INVALID: RunLoopProcInfo has done skill id %d "
                                                " greater than current skill id %d\n",
                                                run_loop_info_->get_done_skill_id(),
                                                current_skill_id));
          } else if (run_loop_info_->get_result_skill_id() + 2 <= current_skill_id) {
            // Make sure that ActionLib has read the skill results before we overwrite them.
            logger_.add_error_log(
                string_format("ActionLib server has not read previous result %d. "
                              "Cannot write new result %d\n",
                              run_loop_info_->get_result_skill_id(),
                              current_skill_id));
          } else if (run_loop_info_->get_done_skill_id() != current_skill_id - 1) {
            // Make sure we are only updating skill sequentially.
            logger_.add_info_log(
                string_format("RunLoopProcInfo done skill id: %d current skill id: %d\n",
                    run_loop_info_->get_done_skill_id(), current_skill_id));
          } else {
            run_loop_info_->set_done_skill_id(current_skill_id);
            logger_.add_info_log(string_format("Did set done_skill_id %d\n", current_skill_id));
          }
        }
        process_info_requires_update_ = false;

        // Check if new skill is available only if no current skill is being
        // currently executed.
        if (!is_executing_skill && run_loop_info_->get_new_skill_available()) {

          std::cout << "Did get new skill";
          // Create new task Skill
          int new_skill_id = run_loop_info_->get_new_skill_id();
          int new_skill_type = run_loop_info_->get_new_skill_type();
          int new_meta_skill_id = run_loop_info_->get_new_meta_skill_id();
          int new_meta_skill_type = run_loop_info_->get_new_meta_skill_type();
          std::cout << string_format("Did find new skill id: %d, type: %d meta skill: %d, type: %d\n",
              new_skill_id, new_skill_type, new_meta_skill_id, new_meta_skill_type);

          // Add new skill
          run_loop_info_->set_current_skill_id(new_skill_id);
          BaseSkill *new_skill;
          if (new_skill_type == 0) {
            new_skill = new SkillInfo(new_skill_id, new_meta_skill_id);
          } else if (new_skill_type == 1) {
            new_skill = new GripperOpenSkill(new_skill_id, new_meta_skill_id);
          } else if (new_skill_type == 2) {
            new_skill = new JointPoseSkill(new_skill_id, new_meta_skill_id);
          } else if (new_skill_type == 3) {
            new_skill = new SaveTrajectorySkill(new_skill_id, new_meta_skill_id);
          } else {
              std::cout << "Incorrect skill type: " << new_skill_type << "\n";
              assert(false);
          }
          skill_manager_.add_skill(new_skill);

          // Get Meta-skill
          BaseMetaSkill* new_meta_skill = skill_manager_.get_meta_skill_with_id(new_meta_skill_id);
          if (new_meta_skill == nullptr) {
            if (new_meta_skill_type == 0) {
              new_meta_skill = new BaseMetaSkill(new_meta_skill_id);
            } else if (new_meta_skill_type == 1) {
              new_meta_skill = new JointPoseContinuousSkill(new_meta_skill_id);
            } else {
                std::cout << "Incorrect meta skill type: " << new_skill_type << "\n";
                assert(false);
            }
            skill_manager_.add_meta_skill(new_meta_skill);
          }

          // Update the shared memory region. This means that the actionlib service will now write
          // to the other memory region, i.e. not the current memory region.
          // TODO(Mohit): We should lock the other memory so that ActionLibServer cannot modify it?
          run_loop_info_->update_shared_memory_region();
          run_loop_info_->set_new_skill_available(false);
        }
      }
    } catch (boost::interprocess::lock_exception) {
      // TODO(Mohit): Do something better here.
      logger_.add_info_log("Cannot acquire lock for run loop info");
    }
  }

}

void RunLoop::run() {
  // Wait for sometime to let the client add data to the buffer
  std::this_thread::sleep_for(std::chrono::seconds(10));

  std::chrono::time_point<std::chrono::high_resolution_clock> start;
  auto milli = std::chrono::milliseconds(1);

  while (1) {
    start = std::chrono::high_resolution_clock::now();

    // Execute the current skill (traj_generator, FBC are here)
    BaseSkill *skill = skill_manager_.get_current_skill();

    // NOTE: We keep on running the last skill even if it is finished!!
    if (skill != 0) {
      // Execute skill.
      skill->execute_skill();

      int memory_index = run_loop_info_->get_current_shared_memory_index();
      SharedBuffer buffer = execution_feedback_buffer_0_;
      if (memory_index == 1) {
        buffer = execution_feedback_buffer_1_;
      }
      skill->write_feedback_to_shared_memory(buffer);

      // Finish skill if possible.
      finish_current_skill(skill);
    }

    // Complete old skills and acquire new skills
    update_process_info();

    // Start new skill, if possible
    BaseSkill *new_skill = skill_manager_.get_current_skill();
    if (should_start_new_skill(skill, new_skill)) {
      start_new_skill(new_skill);
    }

    // Sleep to maintain 1Khz frequency, not sure if this is required or not.
    auto finish = std::chrono::high_resolution_clock::now();
    // Wait for start + milli - finish
    auto elapsed = start + milli - finish;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void RunLoop::setup_print_thread() {
  int print_rate = 10;
  print_thread_ = std::thread([&, print_rate]() {
      // Sleep to achieve the desired print rate.
      while (running_skills_) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
        std::cout << "WTF, will wait for " << static_cast<int>(1.0 / print_rate * 1000.0) << "\n";
        // Try to lock data to avoid read write collisions.
        if (control_loop_data_.mutex_.try_lock()) {
          if (control_loop_data_.has_data_) {
            control_loop_data_.has_data_ = false;
            std::cout << "Count: " << control_loop_data_.counter_ << " time: " <<
                      control_loop_data_.time_ << std::endl;
          }
          control_loop_data_.mutex_.unlock();
        }
      }
  });
}

void RunLoop::setup_robot_default_behavior() {
  // Set additional parameters always before the control loop, NEVER in the control loop!
  // Set collision behavior.
  /*robot_.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});*/
  robot_.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{120.0, 120.0, 118.0, 118.0, 116.0, 114.0, 112.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{120.0, 120.0, 118.0, 118.0, 116.0, 114.0, 112.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{120.0, 120.0, 120.0, 125.0, 125.0, 125.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{120.0, 120.0, 120.0, 125.0, 125.0, 125.0}});

  robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

void RunLoop::didFinishSkillInMetaSkill(BaseSkill* skill) {
  // Finish skill if possible.
  finish_current_skill(skill);
  // Complete old skills and acquire new skills
  update_process_info();
}

void RunLoop::run_on_franka() {
  init();

  setup_print_thread();

  // Wait for sometime to let the client add data to the buffer
  std::this_thread::sleep_for(std::chrono::seconds(10));

  std::chrono::time_point<std::chrono::high_resolution_clock> start;
  auto milli = std::chrono::milliseconds(1);

  setup_robot_default_behavior();

  try {
    running_skills_ = true;
    FileStreamLogger* logger = new FileStreamLogger();
    control_loop_data_.setFileStreamLogger(logger);
    control_loop_data_.startFileLoggerThread();
    while (1) {
      start = std::chrono::high_resolution_clock::now();

      // Execute the current skill (traj_generator, FBC are here)
      BaseSkill* skill = skill_manager_.get_current_skill();
      BaseMetaSkill *meta_skill = skill_manager_.get_current_meta_skill();

      // NOTE: We keep on running the last skill even if it is finished!!
      if (skill != nullptr && meta_skill != nullptr) {
        // Execute skill.
        std::cout << "Will execute skill: " << skill->get_skill_id() << ", meta skill: " <<
          meta_skill->getMetaSkillId() << "\n" << std::endl;
        meta_skill->execute_skill_on_franka(this, &robot_, &gripper_, &control_loop_data_);
      }

      // Complete old skills and acquire new skills
      update_process_info();

      // Start new skill, if possible
      BaseSkill* new_skill = skill_manager_.get_current_skill();
      if (should_start_new_skill(skill, new_skill)) {
        std::cout << "Will start skill\n";
        start_new_skill(new_skill);
      }

      // Sleep to maintain 1Khz frequency, not sure if this is required or not.
      auto finish = std::chrono::high_resolution_clock::now();
      // Wait for start + milli - finish
      auto elapsed = start + milli - finish;
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (const franka::Exception& ex) {

    std::cerr << ex.what() << std::endl;
    logger_.print_error_log();
    logger_.print_warning_log();
    logger_.print_info_log();

  }

  if (print_thread_.joinable()) {
    print_thread_.join();
  }
  if (control_loop_data_.file_logger_thread_.joinable()) {
    control_loop_data_.file_logger_thread_.join();
  }
}

SkillInfoManager* RunLoop::getSkillInfoManager() {
  return &skill_manager_;
}
