#include "iam_robolib/sensor_data_manager.h"


#include <google/protobuf/message.h>

//read sensor_data mutex, read sensor_data_type, read sensor_data_size

SensorDataManagerReadStatus SensorDataManager::readBoundingBoxMessage(BoundingBox &message){

  SensorDataManagerReadStatus status;

  std::cout << "Try to read message" << std::endl;
  try {
    if (buffer_mutex_->try_lock()) {
      float data_size = buffer_[1];
      std::cout << "Will try to read sensor data data of size: " << data_size << std::endl;
      if (message.ParseFromArray(buffer_ + 2, data_size)) {
        status =  SensorDataManagerReadStatus::SUCCESS;
        std::cout << "Did read successfully from buffer";
      } else {
        status =  SensorDataManagerReadStatus::FAIL_TO_READ;
      }

      // TODO: Should we update the first value in the buffer to 0 (implying that we read it correctly?)

      buffer_mutex_->unlock();
    }
  } catch (boost::interprocess::lock_exception) {
    status = SensorDataManagerReadStatus::FAIL_TO_GET_LOCK;
    // Do nothing
  }

  return status;
}

