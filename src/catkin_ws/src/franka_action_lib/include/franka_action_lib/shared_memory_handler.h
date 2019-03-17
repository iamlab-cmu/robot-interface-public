#ifndef FRANKA_ACTION_LIB_SHARED_MEMORY_HANDLER_H
#define FRANKA_ACTION_LIB_SHARED_MEMORY_HANDLER_H

#include <franka_action_lib/ExecuteSkillAction.h> // Note: "Action" is appended
#include <franka_action_lib/RobotState.h>
#include <franka_action_lib/RobolibStatus.h>
#include <franka_action_lib/RunLoopProcessInfoState.h>

#include "ros/ros.h" // For ROS::ERROR messages

#include <array>
#include <vector>
#include <algorithm>
#include <cassert>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <iam_robolib_common/iam_robolib_state_info.h>
#include <iam_robolib_common/run_loop_process_info.h>
#include <iam_robolib_common/SharedMemoryInfo.h>

namespace franka_action_lib
{
  class SharedMemoryHandler {
    public:
      SharedMemoryHandler();

      ~SharedMemoryHandler(void){}

      int loadSkillParametersIntoSharedMemory(const franka_action_lib::ExecuteSkillGoalConstPtr &goal);

      // void startSensorSubscribers(const franka_action_lib::ExecuteSkillGoalConstPtr &goal);

      bool getSkillRunningFlagInSharedMemory();

      int getDoneSkillIdInSharedMemory();

      void setSkillPreemptedFlagInSharedMemory(bool skill_preempted_flag);

      void setNewSkillDescriptionInSharedMemory(std::string description);

      franka_action_lib::ExecuteSkillFeedback getSkillFeedback();

      franka_action_lib::ExecuteSkillResult getSkillResult(int skill_id);

      franka_action_lib::RobotState getRobotState();

      franka_action_lib::RobolibStatus getRobolibStatus();

      franka_action_lib::RunLoopProcessInfoState getRunLoopProcessInfoState();
      
      bool getNewSkillAvailableFlagInSharedMemory();

      int getNewSkillIdInSharedMemory();

      void incrementWatchdogCounter();

    private:

      SharedMemoryInfo shared_memory_info_;

      boost::interprocess::managed_shared_memory managed_shared_memory_;

      RunLoopProcessInfo *run_loop_process_info_;
      boost::interprocess::interprocess_mutex *run_loop_info_mutex_;

      IAMRobolibStateInfo *iam_robolib_state_info_;
      boost::interprocess::interprocess_mutex *iam_robolib_state_info_mutex_;

      boost::interprocess::interprocess_mutex *shared_memory_object_0_mutex_;
      boost::interprocess::interprocess_mutex *shared_memory_object_1_mutex_;

      boost::interprocess::interprocess_mutex *shared_sensor_data_0_mutex_;
      boost::interprocess::interprocess_mutex *shared_sensor_data_1_mutex_;

      boost::interprocess::interprocess_mutex *shared_execution_response_0_mutex_;
      boost::interprocess::interprocess_mutex *shared_execution_response_1_mutex_;

      boost::interprocess::interprocess_mutex *shared_current_robot_state_mutex_;

      boost::interprocess::shared_memory_object shared_memory_object_0_;
      boost::interprocess::shared_memory_object shared_memory_object_1_;
      boost::interprocess::shared_memory_object shared_sensor_data_0_;
      boost::interprocess::shared_memory_object shared_sensor_data_1_;
      boost::interprocess::shared_memory_object shared_execution_result_0_;
      boost::interprocess::shared_memory_object shared_execution_result_1_;
      boost::interprocess::shared_memory_object shared_current_robot_state_;

      boost::interprocess::mapped_region region_traj_params_0_;
      boost::interprocess::mapped_region region_feedback_controller_params_0_;
      boost::interprocess::mapped_region region_termination_params_0_;
      boost::interprocess::mapped_region region_timer_params_0_;

      boost::interprocess::mapped_region region_traj_params_1_;
      boost::interprocess::mapped_region region_feedback_controller_params_1_;
      boost::interprocess::mapped_region region_termination_params_1_;
      boost::interprocess::mapped_region region_timer_params_1_;

      boost::interprocess::mapped_region region_traj_sensor_data_0_;
      boost::interprocess::mapped_region region_feedback_controller_sensor_data_0_;
      boost::interprocess::mapped_region region_termination_sensor_data_0_;
      boost::interprocess::mapped_region region_timer_sensor_data_0_;

      boost::interprocess::mapped_region region_traj_sensor_data_1_;
      boost::interprocess::mapped_region region_feedback_controller_sensor_data_1_;
      boost::interprocess::mapped_region region_termination_sensor_data_1_;
      boost::interprocess::mapped_region region_timer_sensor_data_1_;

      boost::interprocess::mapped_region execution_feedback_region_0_;
      boost::interprocess::mapped_region execution_result_region_0_;
      boost::interprocess::mapped_region execution_feedback_region_1_;
      boost::interprocess::mapped_region execution_result_region_1_;

      boost::interprocess::mapped_region shared_current_robot_region_;

      double *traj_gen_buffer_0_;
      double *feedback_controller_buffer_0_;
      double *termination_buffer_0_;
      double *timer_buffer_0_;
      double *traj_gen_buffer_1_;
      double *feedback_controller_buffer_1_;
      double *termination_buffer_1_;
      double *timer_buffer_1_;

      double *traj_gen_sensor_buffer_0_;
      double *feedback_controller_sensor_buffer_0_;
      double *termination_sensor_buffer_0_;
      double *timer_sensor_buffer_0_;

      double *traj_gen_sensor_buffer_1_;
      double *feedback_controller_sensor_buffer_1_;
      double *termination_sensor_buffer_1_;
      double *timer_sensor_buffer_1_;

      double *execution_feedback_buffer_0_;
      double *execution_result_buffer_0_;
      double *execution_feedback_buffer_1_;
      double *execution_result_buffer_1_;

      double *current_robot_state_buffer_;

      int getCurrentFreeSharedMemoryIndexInSharedMemoryUnprotected();
      int getCurrentSkillIdInSharedMemoryUnprotected();
      void setCurrentSkillIdInSharedMemoryUnprotected(int current_skill_id);
      int getDoneSkillIdInSharedMemoryUnprotected();
      bool getNewSkillAvailableFlagInSharedMemoryUnprotected();
      void setNewSkillFlagInSharedMemoryUnprotected(bool new_skill_flag);
      int getNewSkillIdInSharedMemoryUnprotected();
      void setNewSkillIdInSharedMemoryUnprotected(int new_skill_id);
      void setNewSkillDescriptionInSharedMemoryUnprotected(std::string description);
      void setNewSkillTypeInSharedMemoryUnprotected(int new_skill_type);
      void setNewMetaSkillIdInSharedMemoryUnprotected(int new_meta_skill_id);
      void setNewMetaSkillTypeInSharedMemoryUnprotected(int new_meta_skill_type);
      void setResultSkillIdInSharedMemoryUnprotected(int result_skill_id);

      void loadSensorDataUnprotected(const franka_action_lib::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index);
      void loadTrajGenParamsUnprotected(const franka_action_lib::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index);
      void loadFeedbackControllerParamsUnprotected(const franka_action_lib::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index);
      void loadTerminationParamsUnprotected(const franka_action_lib::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index);
      void loadTimerParamsUnprotected(const franka_action_lib::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index);
  };
}


#endif // FRANKA_ACTION_LIB_SHARED_MEMORY_HANDLER_H
