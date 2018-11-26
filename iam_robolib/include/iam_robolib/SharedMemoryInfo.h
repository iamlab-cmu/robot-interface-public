//
// Created by mohit on 11/25/18.
//

#ifndef TEST_IAM_ROBOLIB_SHAREDMEMORYINFO_H
#define TEST_IAM_ROBOLIB_SHAREDMEMORYINFO_H

#include <string>

class SharedMemoryInfo {
 public:
  SharedMemoryInfo();

  std::string getSharedMemoryNameForParameters(int index);

  std::string getSharedMemoryNameForObjects();

  std::string getSharedMemoryNameForSensorData(int index);

  std::string getRunLoopInfoObjectName();

  std::string getRunLoopInfoMutexName();

  std::string getParameterMemoryMutexName(int index);

  int getParameterMemorySize(int index);
  int getSensorDataMemorySize();

  int getObjectMemorySize();

  int getSizeForTrajectoryParameters();
  int getOffsetForTrajectoryParameters();
  int getSizeForTrajectorySensorData();
  int getOffsetForTrajectorySensorData();

  int getSizeForFeedbackControllerParameters();
  int getOffsetForFeedbackControllerParameters();
  int getSizeForFeedbackControllerSensorData();
  int getOffsetForFeedbackControllerSensorData();

  int getSizeForTerminationParameters();
  int getOffsetForTerminationParameters();
  int getSizeForTerminationSensorData();
  int getOffsetForTerminationSensorData();

  int getSizeForTimerParameters();
  int getOffsetForTimerParameters();
  int getSizeForTimerSensorData();
  int getOffsetForTimerSensorData();

  int getOffsetForExtraSensorData();
  int getSizeForExtraSensorData();

 private:
  const std::string params_memory_name_0_="run_loop_shared_obj_0";
  const std::string params_memory_name_1_="run_loop_shared_obj_1";
  const std::string objects_memory_name_="run_loop_shared_memory";
  const std::string sensor_data_memory_name_0_="run_loop_sensor_data_0";
  const std::string sensor_data_memory_name_1_="run_loop_sensor_data_1";

  // Object names
  const std::string run_loop_info_name_="run_loop_info";
  const std::string run_loop_info_mutex_name_="run_loop_info_mutex";

  // Declare mutexes
  const std::string params_memory_mutex_name_0_="run_loop_shared_obj_0_mutex";
  const std::string params_memory_mutex_name_1_="run_loop_shared_obj_1_mutex";

  // Declare sizes
  const int params_memory_size_0_=4*1024*sizeof(float);
  const int params_memory_size_1_=4*1024*sizeof(float);
  const int objects_memory_size_=1024*sizeof(float);
  const int sensor_buffer_size_ = 5*1024* sizeof(float);

  const int trajectory_params_buffer_size_= 1024 * sizeof(float);
  const int feedback_controller_params_buffer_size_=1024 * sizeof(float);
  const int termination_params_buffer_size_=1024 * sizeof(float);
  const int timer_params_buffer_size_=1024 * sizeof(float);

  const int trajectory_sensor_data_buffer_size_= 1024 * sizeof(float);
  const int feedback_controller_sensor_data_buffer_size_= 1024 * sizeof(float);
  const int termination_sensor_data_buffer_size_= 1024 * sizeof(float);
  const int timer_sensor_data_buffer_size_ = 1024 * sizeof(float);
  const int extra_sensor_data_buffer_size_ = 1024 * sizeof(float);

};

#endif //TEST_IAM_ROBOLIB_SHAREDMEMORYINFO_H
