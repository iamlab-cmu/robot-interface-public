#include "franka_action_lib/shared_memory_handler.h"

namespace franka_action_lib
{

  SharedMemoryHandler::SharedMemoryHandler() : shared_memory_info_()
  {
    managed_shared_memory_ = boost::interprocess::managed_shared_memory(
        boost::interprocess::open_only, shared_memory_info_.getSharedMemoryNameForObjects().c_str());

    // Get RunLoopProcessInfo from the the shared memory segment.
    std::pair<RunLoopProcessInfo*, std::size_t> run_loop_process_info_pair = \
        managed_shared_memory_.find<RunLoopProcessInfo> (shared_memory_info_.getRunLoopInfoObjectName().c_str());
    run_loop_process_info_ = run_loop_process_info_pair.first;

    // Make sure the process info object can be found in memory.
    assert(run_loop_process_info_ != 0);

    // Get mutex for ProcessInfo from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> run_loop_info_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getRunLoopInfoMutexName().c_str());
    run_loop_info_mutex_ = run_loop_info_mutex_pair.first;
    assert(run_loop_info_mutex_ != 0);

    // Get IAMRobolibStateInfo from the the shared memory segment.
    std::pair<IAMRobolibStateInfo*, std::size_t> iam_robolib_state_info_pair = \
        managed_shared_memory_.find<IAMRobolibStateInfo> (shared_memory_info_.getIAMRobolibStateInfoObjectName().c_str());
    iam_robolib_state_info_ = iam_robolib_state_info_pair.first;

    // Make sure the process info object can be found in memory.
    assert(iam_robolib_state_info_ != 0);

    // Get mutex for ProcessInfo from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> iam_robolib_state_info_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getIAMRobolibStateInfoMutexName().c_str());
    iam_robolib_state_info_mutex_ = iam_robolib_state_info_mutex_pair.first;
    assert(iam_robolib_state_info_mutex_ != 0);

    // Get mutex for buffer 0 from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_memory_object_0_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getParameterMemoryMutexName(0).c_str());
    shared_memory_object_0_mutex_ = shared_memory_object_0_mutex_pair.first;
    assert(shared_memory_object_0_mutex_ != 0);

    /**
     * Open shared memory region for parameter buffer 0.
     */
    shared_memory_object_0_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only,
        shared_memory_info_.getSharedMemoryNameForParameters(0).c_str(),
        boost::interprocess::read_write
    );
    std::cout << "line 56 " << std::endl;

    // Allocate regions for each parameter array
    region_traj_params_0_ = boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTrajectoryParameters(),
        shared_memory_info_.getSizeForTrajectoryParameters()
        );
    traj_gen_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_traj_params_0_.get_address());
    region_feedback_controller_params_0_ = boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForFeedbackControllerParameters(),
        shared_memory_info_.getSizeForFeedbackControllerParameters()
        );
    feedback_controller_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_feedback_controller_params_0_.get_address());
    region_termination_params_0_ = boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTerminationParameters(),
        shared_memory_info_.getSizeForTerminationParameters()
    );

    std::cout << "line 80 " << std::endl;
    termination_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_termination_params_0_.get_address());
    region_timer_params_0_ = boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTimerParameters(),
        shared_memory_info_.getSizeForTimerParameters()
    );
    timer_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_timer_params_0_.get_address());
    region_sensor_data_0_ =  boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForSensorData(),
        shared_memory_info_.getSizeForSensorData()
    );
    std::cout << region_sensor_data_0_.get_address() << std::endl;
    sensor_data_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_sensor_data_0_.get_address());
    std::cout << "line 97 " << std::endl;
    // Get mutex for buffer 1 from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_memory_object_1_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getParameterMemoryMutexName(1).c_str());
    shared_memory_object_1_mutex_ = shared_memory_object_1_mutex_pair.first;
    assert(shared_memory_object_1_mutex_ != 0);

    /**
     * Open shared memory region for parameter buffer 1.
     */
    shared_memory_object_1_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only,
        shared_memory_info_.getSharedMemoryNameForParameters(1).c_str(),
        boost::interprocess::read_write
        );
    std::cout << "line 113 " << std::endl;
    // Allocate regions for each parameter array
    region_traj_params_1_ =  boost::interprocess::mapped_region(
        shared_memory_object_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTrajectoryParameters(),
        sizeof(SharedBufferType) * shared_memory_info_.getSizeForTrajectoryParameters()
        );
    traj_gen_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_traj_params_1_.get_address());
    region_feedback_controller_params_1_ = boost::interprocess::mapped_region(
        shared_memory_object_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForFeedbackControllerParameters(),
        shared_memory_info_.getSizeForFeedbackControllerParameters()
        );
    feedback_controller_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_feedback_controller_params_1_.get_address());
    region_termination_params_1_ = boost::interprocess::mapped_region(
        shared_memory_object_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTerminationParameters(),
        shared_memory_info_.getSizeForTerminationParameters()
        );
    termination_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_termination_params_1_.get_address());
    region_timer_params_1_ = boost::interprocess::mapped_region(
        shared_memory_object_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTimerParameters(),
        shared_memory_info_.getSizeForTimerParameters()
        );
    timer_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_timer_params_1_.get_address());
    std::cout << "line 143 " << std::endl;


    // Get mutex for sensor data buffer 0 from the shared memory segment.
//    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_sensor_data_0_mutex_pair = \
//                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
//                                (shared_memory_info_.getSensorDataMutexName(0).c_str());
//    shared_sensor_data_0_mutex_ = shared_sensor_data_0_mutex_pair.first;
//    assert(shared_sensor_data_0_mutex_ != 0);


      std::pair<boost::interprocess::interprocess_mutex *, std::size_t> sensor_data_0_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
              (shared_memory_info_.getSensorDataMutexName(0).c_str());
      sensor_data_0_mutex_ = sensor_data_0_mutex_pair.first;
      assert(sensor_data_0_mutex_ != 0);


      std::cout << "line 150 " << std::endl;
    /**
     * Open shared memory region for sensor data buffer 0.
     */
//    shared_sensor_data_0_ = boost::interprocess::shared_memory_object(
//        boost::interprocess::open_only,
//        shared_memory_info_.getSharedMemoryNameForSensorData(0).c_str(),
//        boost::interprocess::read_write
//    );
      std::cout << "line 159 " << std::endl;
//    region_traj_sensor_data_0_ =  boost::interprocess::mapped_region(
//        shared_sensor_data_0_,
//        boost::interprocess::read_write,
//        shared_memory_info_.getOffsetForTrajectorySensorData(),
//        shared_memory_info_.getSizeForTrajectorySensorData()
//    );
//    traj_gen_sensor_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_traj_sensor_data_0_.get_address());
//    region_feedback_controller_sensor_data_0_= boost::interprocess::mapped_region(
//        shared_sensor_data_0_,
//        boost::interprocess::read_write,
//        shared_memory_info_.getOffsetForFeedbackControllerSensorData(),
//        shared_memory_info_.getSizeForFeedbackControllerSensorData()
//    );
//    feedback_controller_sensor_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_feedback_controller_sensor_data_0_.get_address());
//    region_termination_sensor_data_0_ = boost::interprocess::mapped_region(
//        shared_sensor_data_0_,
//        boost::interprocess::read_write,
//        shared_memory_info_.getOffsetForTerminationSensorData(),
//        shared_memory_info_.getSizeForTerminationSensorData()
//    );
//    termination_sensor_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_termination_sensor_data_0_.get_address());
//    region_timer_sensor_data_0_= boost::interprocess::mapped_region(
//        shared_sensor_data_0_,
//        boost::interprocess::read_write,
//        shared_memory_info_.getOffsetForTimerParameters(),
//        shared_memory_info_.getSizeForTimerParameters()
//    );
//    timer_sensor_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_timer_sensor_data_0_.get_address());

    // Get mutex for sensor data buffer 0 from the shared memory segment.
//    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_sensor_data_1_mutex_pair = \
//                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
//                                (shared_memory_info_.getSensorDataMutexName(1).c_str());
//    shared_sensor_data_1_mutex_ = shared_sensor_data_1_mutex_pair.first;
//    assert(shared_sensor_data_1_mutex_ != 0);
//    std::cout << "line 195 " << std::endl;
    /**
     * Open shared memory region for sensor data buffer 1.
     */
//    shared_sensor_data_1_ = boost::interprocess::shared_memory_object(
//        boost::interprocess::open_only,
//        shared_memory_info_.getSharedMemoryNameForSensorData(1).c_str(),
//        boost::interprocess::read_write
//    );

//    region_traj_sensor_data_1_ =  boost::interprocess::mapped_region(
//        shared_sensor_data_1_,
//        boost::interprocess::read_write,
//        shared_memory_info_.getOffsetForTrajectorySensorData(),
//        shared_memory_info_.getSizeForTrajectorySensorData()
//    );
//    traj_gen_sensor_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_traj_sensor_data_1_.get_address());
//    region_feedback_controller_sensor_data_1_= boost::interprocess::mapped_region(
//        shared_sensor_data_1_,
//        boost::interprocess::read_write,
//        shared_memory_info_.getOffsetForFeedbackControllerSensorData(),
//        shared_memory_info_.getSizeForFeedbackControllerSensorData()
//    );
//    feedback_controller_sensor_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_feedback_controller_sensor_data_1_.get_address());
//    region_termination_sensor_data_1_ = boost::interprocess::mapped_region(
//        shared_sensor_data_1_,
//        boost::interprocess::read_write,
//        shared_memory_info_.getOffsetForTerminationSensorData(),
//        shared_memory_info_.getSizeForTerminationSensorData()
//    );
//    termination_sensor_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_termination_sensor_data_1_.get_address());
//    region_timer_sensor_data_1_= boost::interprocess::mapped_region(
//        shared_sensor_data_1_,
//        boost::interprocess::read_write,
//        shared_memory_info_.getOffsetForTimerParameters(),
//        shared_memory_info_.getSizeForTimerParameters()
//    );
//    timer_sensor_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_timer_sensor_data_1_.get_address());

    // Get mutex for execution response buffer 0 from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_execution_response_0_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getExecutionResponseMutexName(0).c_str());
    shared_execution_response_0_mutex_ = shared_execution_response_0_mutex_pair.first;
    assert(shared_execution_response_0_mutex_ != 0);
    std::cout << "line 240 " << std::endl;
    /**
     * Open shared memory region for execution response buffer 0.
     */
    shared_execution_result_0_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only,
        shared_memory_info_.getSharedMemoryNameForResults(0).c_str(),
        boost::interprocess::read_write
    );

    execution_feedback_region_0_ =  boost::interprocess::mapped_region(
        shared_execution_result_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForExecutionFeedbackData(),
        shared_memory_info_.getSizeForExecutionFeedbackData()
    );
    execution_feedback_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(execution_feedback_region_0_.get_address());
    execution_result_region_0_ = boost::interprocess::mapped_region(
        shared_execution_result_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForExecutionResultData(),
        shared_memory_info_.getSizeForExecutionResultData()
        );
    execution_result_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(execution_result_region_0_.get_address());

    // Get mutex for execution response buffer 1 from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_execution_response_1_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getExecutionResponseMutexName(1).c_str());
    shared_execution_response_1_mutex_ = shared_execution_response_1_mutex_pair.first;
    assert(shared_execution_response_1_mutex_ != 0);

    /**
     * Open shared memory region for execution response buffer 1.
     */
    shared_execution_result_1_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only,
        shared_memory_info_.getSharedMemoryNameForResults(1).c_str(),
        boost::interprocess::read_write
    );

    execution_feedback_region_1_ =  boost::interprocess::mapped_region(
        shared_execution_result_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForExecutionFeedbackData(),
        shared_memory_info_.getSizeForExecutionFeedbackData()
    );
    execution_feedback_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(execution_feedback_region_1_.get_address());
    execution_result_region_1_ = boost::interprocess::mapped_region(
        shared_execution_result_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForExecutionResultData(),
        shared_memory_info_.getSizeForExecutionResultData()
    );
    execution_result_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(execution_result_region_1_.get_address());

    // Get mutex for current robot state buffer
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_current_robot_state_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getCurrentRobotStateMutexName().c_str());
    shared_current_robot_state_mutex_ = shared_current_robot_state_mutex_pair.first;
    assert(shared_current_robot_state_mutex_ != 0);

    /**
     * Open shared memory region for current robot state buffer
     */
    shared_current_robot_state_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only,
        shared_memory_info_.getSharedMemoryNameForCurrentRobotState().c_str(),
        boost::interprocess::read_only
    );

    shared_current_robot_region_ =  boost::interprocess::mapped_region(
        shared_current_robot_state_,
        boost::interprocess::read_only,
        shared_memory_info_.getOffsetForExecutionFeedbackData(),
        shared_memory_info_.getSizeForExecutionFeedbackData()
    );
    current_robot_state_buffer_ = reinterpret_cast<SharedBufferTypePtr>(shared_current_robot_region_.get_address());

  }

  int SharedMemoryHandler::loadSkillParametersIntoSharedMemory(const franka_action_lib::ExecuteSkillGoalConstPtr &goal)
  {
    // Grab lock of run_loop_info_mutex_ to see if we can load the new skill parameters into the shared memory
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    bool new_skill_flag = getNewSkillAvailableFlagInSharedMemoryUnprotected();

    // Return -1 if the new_skill_flag is already set to true, which means that another new skill has already been loaded
    // into the shared memory.
    if(new_skill_flag)
    {
      return -1;
    }

    int current_skill_id = getCurrentSkillIdInSharedMemoryUnprotected();
    int new_skill_id = getNewSkillIdInSharedMemoryUnprotected();

    ROS_DEBUG("Current skill id: %d", current_skill_id);
    ROS_DEBUG("New skill id: %d", new_skill_id);

    if(current_skill_id != new_skill_id)
    {
      ROS_ERROR("Error with the current_skill_id and new_skill_id. current_skill_id = %d, new_skill_id = %d", current_skill_id, new_skill_id);
      return -1;
    }

    new_skill_id += 1;

    int current_free_shared_memory_index = getCurrentFreeSharedMemoryIndexInSharedMemoryUnprotected();

    // Grab lock of the free shared memory
    if(current_free_shared_memory_index == 0)
    {
      // Grab lock of shared_memory_object_0_mutex_ to make sure no one else can modify shared_memory_0_
      boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_memory_object_0_lock(*shared_memory_object_0_mutex_);

      // Load all of the data into shared_memory_0_
      loadSensorDataUnprotected(goal, 0);


      loadTrajGenParamsUnprotected(goal, 0);
      loadFeedbackControllerParamsUnprotected(goal, 0);
      loadTerminationParamsUnprotected(goal, 0);
      loadTimerParamsUnprotected(goal, 0);

      // The lock of the shared_memory_object_0_mutex_ should be released automatically
    }
    else if (current_free_shared_memory_index == 1)
    {
      // Grab lock of shared_memory_object_1_mutex_ to make sure no one else can modify shared_memory_1_
      boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_memory_object_1_lock(*shared_memory_object_1_mutex_);

      // Load all of the data into shared_memory_1_
      loadSensorDataUnprotected(goal, 1);
      loadTrajGenParamsUnprotected(goal, 1);
      loadFeedbackControllerParamsUnprotected(goal, 1);
      loadTerminationParamsUnprotected(goal, 1);
      loadTimerParamsUnprotected(goal, 1);

      // The lock of the shared_memory_object_1_mutex_ should be released automatically
    }

    setNewSkillIdInSharedMemoryUnprotected(new_skill_id);
    setNewSkillTypeInSharedMemoryUnprotected(goal->skill_type);
    setNewSkillDescriptionInSharedMemoryUnprotected(goal->skill_description);
    setNewMetaSkillTypeInSharedMemoryUnprotected(goal->meta_skill_type);
    setNewMetaSkillIdInSharedMemoryUnprotected(goal->meta_skill_id);

    // Set the new skill flag in shared memory to true to signal that a new skill has been loaded into the current free shared memory.
    setNewSkillFlagInSharedMemoryUnprotected(true);

    // Return the skill_id of the current skill
    return new_skill_id;

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  bool SharedMemoryHandler::getSkillRunningFlagInSharedMemory()
  {
    bool skill_running_flag;
    {
      // Grab the lock of the run_loop_info_mutex_
      boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

      skill_running_flag = run_loop_process_info_->get_is_running_skill();

      // The lock of the run_loop_info_mutex_ should be released automatically
    }

    // Return the skill_running_flag
    return skill_running_flag;
  }

  int SharedMemoryHandler::getDoneSkillIdInSharedMemory()
  {
    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_, boost::interprocess::defer_lock);
    if (run_loop_info_lock.try_lock()) {
      // Return the done_skill_id
      return getDoneSkillIdInSharedMemoryUnprotected();
    } else {
      return -1;
    }

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  void SharedMemoryHandler::setSkillPreemptedFlagInSharedMemory(bool skill_preempted_flag)
  {
    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    // Set skill_preempted_ in run_loop_process_info_ to the input skill_preempted_flag
    run_loop_process_info_->set_skill_preempted(skill_preempted_flag);

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  void SharedMemoryHandler::setNewSkillDescriptionInSharedMemory(std::string description)
  {
    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    setNewSkillDescriptionInSharedMemoryUnprotected(description);

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  franka_action_lib::ExecuteSkillFeedback SharedMemoryHandler::getSkillFeedback()
  {
    franka_action_lib::ExecuteSkillFeedback feedback;

    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_, boost::interprocess::defer_lock);

    if (run_loop_info_lock.try_lock()) {

      int current_free_shared_feedback_index = run_loop_process_info_->get_current_free_shared_feedback_index();

      if(current_free_shared_feedback_index == 0)
      {
        // Grab lock of shared_execution_response_0_mutex_ to make sure no one else can modify 0
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_execution_response_0_lock(*shared_execution_response_0_mutex_);

        int num_execution_feedback = static_cast<int>(execution_feedback_buffer_0_[0]);

        std::vector<SharedBufferType> execution_feedback(execution_feedback_buffer_0_ + 1, execution_feedback_buffer_0_ + num_execution_feedback + 1);

        feedback.num_execution_feedback = num_execution_feedback;
        feedback.execution_feedback = execution_feedback;

        // The lock of the shared_execution_response_0_mutex_ should be released automatically
      }
      else if(current_free_shared_feedback_index == 1)
      {
        // Grab lock of shared_execution_response_1_mutex_ to make sure no one else can modify 1
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_execution_response_1_lock(*shared_execution_response_1_mutex_);

        int num_execution_feedback = static_cast<int>(execution_feedback_buffer_1_[0]);

        std::vector<SharedBufferType> execution_feedback(execution_feedback_buffer_1_ + 1, execution_feedback_buffer_1_ + num_execution_feedback + 1);

        feedback.num_execution_feedback = num_execution_feedback;
        feedback.execution_feedback = execution_feedback;

        // The lock of the shared_execution_response_1_mutex_ should be released automatically
      }
    } else {
      feedback.num_execution_feedback = -1;
    }

    return feedback;

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  franka_action_lib::ExecuteSkillResult SharedMemoryHandler::getSkillResult(int skill_id)
  {
    franka_action_lib::ExecuteSkillResult result;

    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    int result_memory_index = skill_id % 2;

    if(result_memory_index == 0)
    {
      int num_execution_result = static_cast<int>(execution_result_buffer_0_[0]);

      std::vector<SharedBufferType> execution_result(execution_result_buffer_0_ + 1, execution_result_buffer_0_ + num_execution_result + 1);

      result.num_execution_result = num_execution_result;
      result.execution_result = execution_result;

      // The lock of the shared_execution_response_0_mutex_ should be released automatically
    }
    else if(result_memory_index == 1)
    {
      int num_execution_result = static_cast<int>(execution_result_buffer_1_[0]);

      std::vector<SharedBufferType> execution_result(execution_result_buffer_1_ + 1, execution_result_buffer_1_ + num_execution_result + 1);

      result.num_execution_result = num_execution_result;
      result.execution_result = execution_result;

      // The lock of the shared_execution_response_1_mutex_ should be released automatically
    }

    setResultSkillIdInSharedMemoryUnprotected(skill_id);

    return result;

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  franka_action_lib::RobotState SharedMemoryHandler::getRobotState(std::array<double, 144> &robot_frames) {
    franka_action_lib::RobotState robot_state;

    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> current_robot_state_lock(*shared_current_robot_state_mutex_, boost::interprocess::defer_lock);
    if (current_robot_state_lock.try_lock()) {
      robot_state.header.stamp = ros::Time::now();

      size_t offset = 0;

      int num_elements = static_cast<int>(current_robot_state_buffer_[offset++]);

      memcpy(&robot_state.pose_desired, &current_robot_state_buffer_[offset], robot_state.pose_desired.size() * sizeof(SharedBufferType));
      offset += robot_state.pose_desired.size();

      memcpy(&robot_state.O_T_EE, &current_robot_state_buffer_[offset], robot_state.O_T_EE.size() * sizeof(SharedBufferType));
      offset += robot_state.O_T_EE.size();

      memcpy(&robot_state.O_T_EE_d, &current_robot_state_buffer_[offset], robot_state.O_T_EE_d.size() * sizeof(SharedBufferType));
      offset += robot_state.O_T_EE_d.size();
      
      memcpy(&robot_state.F_T_EE, &current_robot_state_buffer_[offset], robot_state.F_T_EE.size() * sizeof(SharedBufferType));
      offset += robot_state.F_T_EE.size();

      memcpy(&robot_state.EE_T_K, &current_robot_state_buffer_[offset], robot_state.EE_T_K.size() * sizeof(SharedBufferType));
      offset += robot_state.EE_T_K.size();
      
      robot_state.m_ee = current_robot_state_buffer_[offset++];

      memcpy(&robot_state.I_ee, &current_robot_state_buffer_[offset], robot_state.I_ee.size() * sizeof(SharedBufferType));
      offset += robot_state.I_ee.size();

      memcpy(&robot_state.F_x_Cee, &current_robot_state_buffer_[offset], robot_state.F_x_Cee.size() * sizeof(SharedBufferType));
      offset += robot_state.F_x_Cee.size();

      robot_state.m_load = current_robot_state_buffer_[offset++];

      memcpy(&robot_state.I_load, &current_robot_state_buffer_[offset], robot_state.I_load.size() * sizeof(SharedBufferType));
      offset += robot_state.I_load.size();

      memcpy(&robot_state.F_x_Cload, &current_robot_state_buffer_[offset], robot_state.F_x_Cload.size() * sizeof(SharedBufferType));
      offset += robot_state.F_x_Cload.size();

      robot_state.m_total = current_robot_state_buffer_[offset++];

      memcpy(&robot_state.I_total, &current_robot_state_buffer_[offset], robot_state.I_total.size() * sizeof(SharedBufferType));
      offset += robot_state.I_total.size();

      memcpy(&robot_state.F_x_Ctotal, &current_robot_state_buffer_[offset], robot_state.F_x_Ctotal.size() * sizeof(SharedBufferType));
      offset += robot_state.F_x_Ctotal.size();
      
      memcpy(&robot_state.elbow, &current_robot_state_buffer_[offset], robot_state.elbow.size() * sizeof(SharedBufferType));
      offset += robot_state.elbow.size();

      memcpy(&robot_state.elbow_d, &current_robot_state_buffer_[offset], robot_state.elbow_d.size() * sizeof(SharedBufferType));
      offset += robot_state.elbow_d.size();

      memcpy(&robot_state.elbow_c, &current_robot_state_buffer_[offset], robot_state.elbow_c.size() * sizeof(SharedBufferType));
      offset += robot_state.elbow_c.size();

      memcpy(&robot_state.delbow_c, &current_robot_state_buffer_[offset], robot_state.delbow_c.size() * sizeof(SharedBufferType));
      offset += robot_state.delbow_c.size();

      memcpy(&robot_state.ddelbow_c, &current_robot_state_buffer_[offset], robot_state.ddelbow_c.size() * sizeof(SharedBufferType));
      offset += robot_state.ddelbow_c.size();
      
      memcpy(&robot_state.tau_J, &current_robot_state_buffer_[offset], robot_state.tau_J.size() * sizeof(SharedBufferType));
      offset += robot_state.tau_J.size();

      memcpy(&robot_state.tau_J_d, &current_robot_state_buffer_[offset], robot_state.tau_J_d.size() * sizeof(SharedBufferType));
      offset += robot_state.tau_J_d.size();

      memcpy(&robot_state.dtau_J, &current_robot_state_buffer_[offset], robot_state.dtau_J.size() * sizeof(SharedBufferType));
      offset += robot_state.dtau_J.size();

      memcpy(&robot_state.q, &current_robot_state_buffer_[offset], robot_state.q.size() * sizeof(SharedBufferType));
      offset += robot_state.q.size();

      memcpy(&robot_state.q_d, &current_robot_state_buffer_[offset], robot_state.q_d.size() * sizeof(SharedBufferType));
      offset += robot_state.q_d.size();

      memcpy(&robot_state.dq, &current_robot_state_buffer_[offset], robot_state.dq.size() * sizeof(SharedBufferType));
      offset += robot_state.dq.size();

      memcpy(&robot_state.dq_d, &current_robot_state_buffer_[offset], robot_state.dq_d.size() * sizeof(SharedBufferType));
      offset += robot_state.dq_d.size();

      memcpy(&robot_state.ddq_d, &current_robot_state_buffer_[offset], robot_state.ddq_d.size() * sizeof(SharedBufferType));
      offset += robot_state.ddq_d.size();

      memcpy(&robot_state.joint_contact, &current_robot_state_buffer_[offset], robot_state.joint_contact.size() * sizeof(SharedBufferType));
      offset += robot_state.joint_contact.size();

      memcpy(&robot_state.cartesian_contact, &current_robot_state_buffer_[offset], robot_state.cartesian_contact.size() * sizeof(SharedBufferType));
      offset += robot_state.cartesian_contact.size();

      memcpy(&robot_state.joint_collision, &current_robot_state_buffer_[offset], robot_state.joint_collision.size() * sizeof(SharedBufferType));
      offset += robot_state.joint_collision.size();

      memcpy(&robot_state.cartesian_collision, &current_robot_state_buffer_[offset], robot_state.cartesian_collision.size() * sizeof(SharedBufferType));
      offset += robot_state.cartesian_collision.size();

      memcpy(&robot_state.tau_ext_hat_filtered, &current_robot_state_buffer_[offset], robot_state.tau_ext_hat_filtered.size() * sizeof(SharedBufferType));
      offset += robot_state.tau_ext_hat_filtered.size();

      memcpy(&robot_state.O_F_ext_hat_K, &current_robot_state_buffer_[offset], robot_state.O_F_ext_hat_K.size() * sizeof(SharedBufferType));
      offset += robot_state.O_F_ext_hat_K.size();

      memcpy(&robot_state.K_F_ext_hat_K, &current_robot_state_buffer_[offset], robot_state.K_F_ext_hat_K.size() * sizeof(SharedBufferType));
      offset += robot_state.K_F_ext_hat_K.size();

      memcpy(&robot_state.O_dP_EE_d, &current_robot_state_buffer_[offset], robot_state.O_dP_EE_d.size() * sizeof(SharedBufferType));
      offset += robot_state.O_dP_EE_d.size();

      memcpy(&robot_state.O_T_EE_c, &current_robot_state_buffer_[offset], robot_state.O_T_EE_c.size() * sizeof(SharedBufferType));
      offset += robot_state.O_T_EE_c.size();

      memcpy(&robot_state.O_dP_EE_c, &current_robot_state_buffer_[offset], robot_state.O_dP_EE_c.size() * sizeof(SharedBufferType));
      offset += robot_state.O_dP_EE_c.size();

      memcpy(&robot_state.O_ddP_EE_c, &current_robot_state_buffer_[offset], robot_state.O_ddP_EE_c.size() * sizeof(SharedBufferType));
      offset += robot_state.O_ddP_EE_c.size();

      memcpy(&robot_state.theta, &current_robot_state_buffer_[offset], robot_state.theta.size() * sizeof(SharedBufferType));
      offset += robot_state.theta.size();

      memcpy(&robot_state.dtheta, &current_robot_state_buffer_[offset], robot_state.dtheta.size() * sizeof(SharedBufferType));
      offset += robot_state.dtheta.size();

      memcpy(&robot_frames, &current_robot_state_buffer_[offset], robot_frames.size() * sizeof(SharedBufferType));
      offset += robot_frames.size();

      robot_state.current_errors.joint_position_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_position_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.self_collision_avoidance_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.joint_velocity_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_velocity_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.force_control_safety_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.joint_reflex = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_reflex = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.max_goal_pose_deviation_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.max_path_pose_deviation_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_velocity_profile_safety_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.joint_position_motion_generator_start_pose_invalid = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.joint_motion_generator_position_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.joint_motion_generator_velocity_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.joint_motion_generator_velocity_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.joint_motion_generator_acceleration_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_position_motion_generator_start_pose_invalid = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_motion_generator_elbow_limit_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_motion_generator_velocity_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_motion_generator_velocity_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_motion_generator_acceleration_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_motion_generator_elbow_sign_inconsistent = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_motion_generator_start_elbow_invalid = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_motion_generator_joint_position_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_motion_generator_joint_velocity_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_motion_generator_joint_velocity_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_motion_generator_joint_acceleration_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.cartesian_position_motion_generator_invalid_frame = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.force_controller_desired_force_tolerance_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.controller_torque_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.start_elbow_sign_inconsistent = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.communication_constraints_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.power_limit_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.joint_p2p_insufficient_torque_for_planning = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.tau_j_range_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.instability_detected = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.current_errors.joint_move_in_wrong_direction = current_robot_state_buffer_[offset++] == 1 ? true : false;

      robot_state.last_motion_errors.joint_position_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_position_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.self_collision_avoidance_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.joint_velocity_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_velocity_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.force_control_safety_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.joint_reflex = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_reflex = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.max_goal_pose_deviation_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.max_path_pose_deviation_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_velocity_profile_safety_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.joint_position_motion_generator_start_pose_invalid = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.joint_motion_generator_position_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.joint_motion_generator_velocity_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.joint_motion_generator_velocity_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.joint_motion_generator_acceleration_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_position_motion_generator_start_pose_invalid = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_motion_generator_elbow_limit_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_motion_generator_velocity_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_motion_generator_velocity_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_motion_generator_acceleration_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_motion_generator_elbow_sign_inconsistent = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_motion_generator_start_elbow_invalid = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_motion_generator_joint_position_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_motion_generator_joint_velocity_limits_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_motion_generator_joint_velocity_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_motion_generator_joint_acceleration_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.cartesian_position_motion_generator_invalid_frame = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.force_controller_desired_force_tolerance_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.controller_torque_discontinuity = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.start_elbow_sign_inconsistent = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.communication_constraints_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.power_limit_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.joint_p2p_insufficient_torque_for_planning = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.tau_j_range_violation = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.instability_detected = current_robot_state_buffer_[offset++] == 1 ? true : false;
      robot_state.last_motion_errors.joint_move_in_wrong_direction = current_robot_state_buffer_[offset++] == 1 ? true : false;
      
      robot_state.control_command_success_rate = current_robot_state_buffer_[offset++];

      robot_state.robot_mode = static_cast<uint8_t>(current_robot_state_buffer_[offset++]);

      robot_state.robot_time = current_robot_state_buffer_[offset++];

      robot_state.gripper_width = current_robot_state_buffer_[offset++];

      robot_state.gripper_max_width = current_robot_state_buffer_[offset++];

      robot_state.gripper_is_grasped = current_robot_state_buffer_[offset++] == 1 ? true : false;

      robot_state.gripper_temperature = static_cast<uint16_t>(current_robot_state_buffer_[offset++]);

      robot_state.gripper_time = current_robot_state_buffer_[offset++];

      robot_state.is_fresh = true;
    } else {
      robot_state.is_fresh = false;
    }

    return robot_state;
  }

  franka_action_lib::RobolibStatus SharedMemoryHandler::getRobolibStatus()
  {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> iam_robolib_state_info_lock(*iam_robolib_state_info_mutex_, boost::interprocess::defer_lock);

    franka_action_lib::RobolibStatus robolib_status;
    robolib_status.header.stamp = ros::Time::now();

    if (iam_robolib_state_info_lock.try_lock()) {
      // TODO(jacky): 25 roughly equates to 500ms of disconnect from the robolib. This should be placed in a global config file.
      if (iam_robolib_state_info_->get_watchdog_counter() > 25) {
        robolib_status.is_ready = false;
      } else {
        robolib_status.is_ready = iam_robolib_state_info_->get_is_ready();
      }
      robolib_status.error_description = iam_robolib_state_info_->get_error_description();
      robolib_status.is_fresh = true;
    } else {
      robolib_status.is_fresh = false;
    }
    
    return robolib_status;
  }

  franka_action_lib::RunLoopProcessInfoState SharedMemoryHandler::getRunLoopProcessInfoState()
  {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_, boost::interprocess::defer_lock);

    franka_action_lib::RunLoopProcessInfoState run_loop_process_info_state;
    run_loop_process_info_state.header.stamp = ros::Time::now();

    if (run_loop_info_lock.try_lock()) {
      run_loop_process_info_state.current_memory_region = run_loop_process_info_->get_current_memory_region();
      run_loop_process_info_state.current_sensor_region = run_loop_process_info_->get_current_sensor_region();
      run_loop_process_info_state.current_feedback_region = run_loop_process_info_->get_current_feedback_region();
      run_loop_process_info_state.current_skill_id = run_loop_process_info_->get_current_skill_id();
      run_loop_process_info_state.current_skill_type = run_loop_process_info_->get_current_skill_type();
      run_loop_process_info_state.current_meta_skill_id = run_loop_process_info_->get_current_meta_skill_id();
      run_loop_process_info_state.current_meta_skill_type = run_loop_process_info_->get_current_meta_skill_type();
      run_loop_process_info_state.current_skill_description = run_loop_process_info_->get_current_skill_description();
      run_loop_process_info_state.new_skill_available = run_loop_process_info_->get_new_skill_available();
      run_loop_process_info_state.new_skill_id = run_loop_process_info_->get_new_skill_id();
      run_loop_process_info_state.new_skill_type = run_loop_process_info_->get_new_skill_type();
      run_loop_process_info_state.new_meta_skill_id = run_loop_process_info_->get_new_meta_skill_id();
      run_loop_process_info_state.new_meta_skill_type = run_loop_process_info_->get_new_meta_skill_type();
      run_loop_process_info_state.new_skill_description = run_loop_process_info_->get_new_skill_description();
      run_loop_process_info_state.is_running_skill = run_loop_process_info_->get_is_running_skill();
      run_loop_process_info_state.skill_preempted = run_loop_process_info_->get_skill_preempted();
      run_loop_process_info_state.done_skill_id = run_loop_process_info_->get_done_skill_id();
      run_loop_process_info_state.result_skill_id = run_loop_process_info_->get_result_skill_id();
      run_loop_process_info_state.time_skill_started_in_robot_time = run_loop_process_info_->get_time_skill_started_in_robot_time();
      run_loop_process_info_state.time_skill_finished_in_robot_time = run_loop_process_info_->get_time_skill_finished_in_robot_time();
      
      run_loop_process_info_state.is_fresh = true;
    } else {
      run_loop_process_info_state.is_fresh = false;
    }
    
    return run_loop_process_info_state;
  }

  void SharedMemoryHandler::incrementWatchdogCounter()
  {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> iam_robolib_state_info_lock(*iam_robolib_state_info_mutex_, boost::interprocess::defer_lock);
    if (iam_robolib_state_info_lock.try_lock()) {
      iam_robolib_state_info_->increment_watchdog_counter();
    }
  }
  
  bool SharedMemoryHandler::getNewSkillAvailableFlagInSharedMemory()
  {
    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    // Return the done_skill_id
    return getNewSkillAvailableFlagInSharedMemoryUnprotected();

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  int SharedMemoryHandler::getNewSkillIdInSharedMemory()
  {
    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    // Return the done_skill_id
    return getNewSkillIdInSharedMemoryUnprotected();

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  // ALL UNPROTECTED FUNCTIONS BELOW REQUIRE A MUTEX OVER THE RUN_LOOP_INFO

  int SharedMemoryHandler::getCurrentFreeSharedMemoryIndexInSharedMemoryUnprotected()
  {
    // Return the current_free_shared_memory_index from run_loop_process_info_
    return run_loop_process_info_->get_current_free_shared_memory_index();
  }

  int SharedMemoryHandler::getCurrentSkillIdInSharedMemoryUnprotected()
  {
    // Return the current_skill_id from run_loop_process_info_
    return run_loop_process_info_->get_current_skill_id();
  }

  int SharedMemoryHandler::getDoneSkillIdInSharedMemoryUnprotected()
  {
    // Return the done_skill_id from run_loop_process_info_
    return run_loop_process_info_->get_done_skill_id();
  }

  bool SharedMemoryHandler::getNewSkillAvailableFlagInSharedMemoryUnprotected()
  {
    // Return the new_skill_available_flag
    return run_loop_process_info_->get_new_skill_available();
  }

  void SharedMemoryHandler::setNewSkillFlagInSharedMemoryUnprotected(bool new_skill_flag)
  {
    // Set new_skill_available_ in run_loop_process_info_ to the input new_skill_flag
    run_loop_process_info_->set_new_skill_available(new_skill_flag);
  }

  int SharedMemoryHandler::getNewSkillIdInSharedMemoryUnprotected()
  {
    // Return the new_skill_id from run_loop_process_info_
    return run_loop_process_info_->get_new_skill_id();
  }

  void SharedMemoryHandler::setNewSkillIdInSharedMemoryUnprotected(int new_skill_id)
  {
    // Set new_skill_id_ in run_loop_process_info_ to the input new_skill_id
    run_loop_process_info_->set_new_skill_id(new_skill_id);
  }

  void SharedMemoryHandler::setNewSkillDescriptionInSharedMemoryUnprotected(std::string description)
  {
    // Set new_skill_id_ in run_loop_process_info_ to the input new_skill_id
    run_loop_process_info_->set_new_skill_description(description);
  }

  void SharedMemoryHandler::setNewSkillTypeInSharedMemoryUnprotected(int new_skill_type)
  {
    // Set new_skill_type_ in run_loop_process_info_ to the input new_skill_type
    run_loop_process_info_->set_new_skill_type(new_skill_type);
  }

  void SharedMemoryHandler::setNewMetaSkillIdInSharedMemoryUnprotected(int new_meta_skill_id)
  {
    // Set new_meta_skill_id_ in run_loop_process_info_ to the input new_meta_skill_id
    run_loop_process_info_->set_new_meta_skill_id(new_meta_skill_id);
  }

  void SharedMemoryHandler::setNewMetaSkillTypeInSharedMemoryUnprotected(int new_meta_skill_type)
  {
    // Set new_meta_skill_type_ in run_loop_process_info_ to the input new_meta_skill_type
    run_loop_process_info_->set_new_meta_skill_type(new_meta_skill_type);
  }

  void SharedMemoryHandler::setResultSkillIdInSharedMemoryUnprotected(int result_skill_id)
  {
    // Set result_skill_id_ in run_loop_process_info_ to the input result_skill_id
    run_loop_process_info_->set_result_skill_id(result_skill_id);
  }

  // Loads sensor data into the designated sensor memory buffer
  // Requires a lock on the mutex of the designated sensor memory buffer
  void SharedMemoryHandler::loadSensorDataUnprotected(const franka_action_lib::ExecuteSkillGoalConstPtr &goal,
                                                      int current_free_shared_memory_index)
  {
//    if(current_free_shared_memory_index == 0)
//    {
//      // Currently ignoring sensor names and putting everything into the traj_gen_sensor_buffer
//      traj_gen_sensor_buffer_0_[0] = static_cast<SharedBufferType>(goal->sensor_value_sizes[0]);
//      memcpy(traj_gen_sensor_buffer_0_ + 1, &goal->initial_sensor_values[0], goal->sensor_value_sizes[0] * sizeof(SharedBufferType));
//    }
//    else if(current_free_shared_memory_index == 1)
//    {
//      // Currently ignoring sensor names and putting everything into the traj_gen_sensor_buffer
//      traj_gen_sensor_buffer_1_[0] = static_cast<SharedBufferType>(goal->sensor_value_sizes[0]);
//      memcpy(traj_gen_sensor_buffer_1_ + 1, &goal->initial_sensor_values[0], goal->sensor_value_sizes[0] * sizeof(SharedBufferType));
//    }
  }

    //Adding new function to load sensor data into sensor memory buffer
void SharedMemoryHandler::loadSensorData_dummy_Unprotected(const franka_action_lib::SensorData::ConstPtr &ptr,
                                                           int current_free_shared_memory_index)
{
    if(current_free_shared_memory_index == 0)
    {
        //std::cout << std::setprecision(10) <<ptr->data << std::endl;
        //std::cout << sensor_data_buffer_0_ << std::endl;

        // if(sensor_data_0_mutex_.try_lock()) {
            sensor_data_buffer_0_[0] = static_cast<SharedBufferType>(ptr->size);
            memcpy(sensor_data_buffer_0_ + 2, &ptr->sensorData,
                   ptr->size * sizeof(SharedBufferType));  //TODO change size of parameters
            // sensor_data_0_mutex_.unlock();
        // }

    }
//    else if(current_free_shared_memory_index == 1)
//    {
//        sensor_data_buffer_1_[0] = static_cast<SharedBufferType>(goal->termination_type);
//        sensor_data_buffer_1_[1] = static_cast<SharedBufferType>(goal->num_termination_params);
//        memcpy(sensor_data_buffer_1_ + 2, &goal->termination_params[0], goal->num_termination_params * sizeof(SharedBufferType));
//    }
}

  // Loads traj gen parameters into the designated current_free_shared_memory_index buffer
  // Requires a lock on the mutex of the designated current_free_shared_memory_index buffer
  void SharedMemoryHandler::loadTrajGenParamsUnprotected(const franka_action_lib::ExecuteSkillGoalConstPtr &goal,
                                                         int current_free_shared_memory_index)
  {
    if(current_free_shared_memory_index == 0)
    {
      traj_gen_buffer_0_[0] = static_cast<SharedBufferType>(goal->traj_gen_type);
      traj_gen_buffer_0_[1] = static_cast<SharedBufferType>(goal->num_traj_gen_params);
      memcpy(traj_gen_buffer_0_ + 2, &goal->traj_gen_params[0], goal->num_traj_gen_params * sizeof(SharedBufferType));
    }
    else if(current_free_shared_memory_index == 1)
    {
      traj_gen_buffer_1_[0] = static_cast<SharedBufferType>(goal->traj_gen_type);
      traj_gen_buffer_1_[1] = static_cast<SharedBufferType>(goal->num_traj_gen_params);
      memcpy(traj_gen_buffer_1_ + 2, &goal->traj_gen_params[0], goal->num_traj_gen_params * sizeof(SharedBufferType));
    }
  }

  // Loads feedback controller parameters into the designated current_free_shared_memory_index buffer
  // Requires a lock on the mutex of the designated current_free_shared_memory_index buffer
  void SharedMemoryHandler::loadFeedbackControllerParamsUnprotected(
      const franka_action_lib::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index)
  {
    if(current_free_shared_memory_index == 0)
    {
      feedback_controller_buffer_0_[0] = static_cast<SharedBufferType>(goal->feedback_controller_type);
      feedback_controller_buffer_0_[1] = static_cast<SharedBufferType>(goal->num_feedback_controller_params);
      memcpy(feedback_controller_buffer_0_ + 2,
          &goal->feedback_controller_params[0],
          goal->num_feedback_controller_params * sizeof(SharedBufferType));
    }
    else if(current_free_shared_memory_index == 1)
    {
      feedback_controller_buffer_1_[0] = static_cast<SharedBufferType>(goal->feedback_controller_type);
      feedback_controller_buffer_1_[1] = static_cast<SharedBufferType>(goal->num_feedback_controller_params);
      memcpy(feedback_controller_buffer_1_ + 2,
          &goal->feedback_controller_params[0],
          goal->num_feedback_controller_params * sizeof(SharedBufferType));
    }
  }

  // Loads termination parameters into the designated current_free_shared_memory_index buffer
  // Requires a lock on the mutex of the designated current_free_shared_memory_index buffer
  void SharedMemoryHandler::loadTerminationParamsUnprotected(const franka_action_lib::ExecuteSkillGoalConstPtr &goal,
                                                             int current_free_shared_memory_index)
  {
    if(current_free_shared_memory_index == 0)
    {
      termination_buffer_0_[0] = static_cast<SharedBufferType>(goal->termination_type);
      termination_buffer_0_[1] = static_cast<SharedBufferType>(goal->num_termination_params);
      memcpy(termination_buffer_0_ + 2, &goal->termination_params[0], goal->num_termination_params * sizeof(SharedBufferType));
    }
    else if(current_free_shared_memory_index == 1)
    {
      termination_buffer_1_[0] = static_cast<SharedBufferType>(goal->termination_type);
      termination_buffer_1_[1] = static_cast<SharedBufferType>(goal->num_termination_params);
      memcpy(termination_buffer_1_ + 2, &goal->termination_params[0], goal->num_termination_params * sizeof(SharedBufferType));
    }
  }

  // Loads timer parameters into the designated current_free_shared_memory_index buffer
  // Requires a lock on the mutex of the designated current_free_shared_memory_index buffer
  void SharedMemoryHandler::loadTimerParamsUnprotected(const franka_action_lib::ExecuteSkillGoalConstPtr &goal,
                                                       int current_free_shared_memory_index)
  {
    if(current_free_shared_memory_index == 0)
    {
      timer_buffer_0_[0] = static_cast<SharedBufferType>(goal->timer_type);
      timer_buffer_0_[1] = static_cast<SharedBufferType>(goal->num_timer_params);
      memcpy(timer_buffer_0_ + 2, &goal->timer_params[0], goal->num_timer_params * sizeof(SharedBufferType));
    }
    else if(current_free_shared_memory_index == 1)
    {
      timer_buffer_1_[0] = static_cast<SharedBufferType>(goal->timer_type);
      timer_buffer_1_[1] = static_cast<SharedBufferType>(goal->num_timer_params);
      memcpy(timer_buffer_1_ + 2, &goal->timer_params[0], goal->num_timer_params * sizeof(SharedBufferType));
    }
  }

}
