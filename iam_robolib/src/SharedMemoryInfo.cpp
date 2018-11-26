//
// Created by mohit on 11/25/18.
//

#include <iam_robolib/SharedMemoryInfo.h>

#include <cassert>

SharedMemoryInfo::SharedMemoryInfo() {
  // pass
}


std::string SharedMemoryInfo::getSharedMemoryNameForParameters(int index) {
  if (index == 0) {
    return params_memory_name_0_;
  } else if (index == 1) {
    return params_memory_name_1_;
  } else {
    assert(false);
    return "";
  }
}

std::string SharedMemoryInfo::getSharedMemoryNameForObjects() {
  return objects_memory_name_;
}

std::string SharedMemoryInfo::getRunLoopInfoObjectName() {
  return run_loop_info_name_;
}

std::string SharedMemoryInfo::getRunLoopInfoMutexName() {
  return run_loop_info_mutex_name_;
}

std::string SharedMemoryInfo::getParameterMemoryMutexName(int index) {
  if (index == 0) {
    return params_memory_mutex_name_0_;
  } else if (index == 1) {
    return params_memory_mutex_name_1_;
  } else {
    assert(false);
    return "";
  }
}

int SharedMemoryInfo::getParameterMemorySize(int index) {
  if (index == 0) {
    return params_memory_size_0_;
  } else if (index == 1) {
    return params_memory_size_1_;
  } else {
    assert(false);
    return 0;
  }
}

int SharedMemoryInfo::getObjectMemorySize() {
  return objects_memory_size_;
}
