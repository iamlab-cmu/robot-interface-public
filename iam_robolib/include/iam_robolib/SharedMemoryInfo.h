//
// Created by mohit on 11/25/18.
//

#ifndef TEST_IAM_ROBOLIB_SHAREDMEMORYINFO_H
#define TEST_IAM_ROBOLIB_SHAREDMEMORYINFO_H

#include <string>

class SharedMemoryInfo {
 public:
  SharedMemoryInfo();

 private:
  const std::string params_memory_name_0_="run_loop_shared_obj_0";
  const std::string params_memory_name_1_="run_loop_shared_obj_1";
  const std::string objects_memory_name_="run_loop_shared_memory";

  // Object names
  const std::string run_loop_info_name_="run_loop_info";
  const std::string run_loop_info_mutex_name_="run_loop_info_mutex";

  // Declare mutexes
  const std::string params_memory_mutex_name_0_="run_loop_shared_obj_0_mutex";
  const std::string params_memory_mutex_name_1_="run_loop_shared_obj_1_mutex";

  // Declare sizes
  const int params_memory_size_0_=1024;
  const int params_memory_size_1_=1024;
  const int objects_memory_size_=1024;
};

#endif //TEST_IAM_ROBOLIB_SHAREDMEMORYINFO_H
