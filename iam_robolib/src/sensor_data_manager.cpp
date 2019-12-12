#include "iam_robolib/sensor_data_manager.h"


#include <google/protobuf/message.h>

//read sensor_data mutex, read sensor_data_type, read sensor_data_size

SensorDataManagerReadStatus SensorDataManager::readBoundingBoxMessage(BoundingBox &message){

  SensorDataManagerReadStatus status;

  std::cout << "Try to read message" << std::endl;
  try {
    if (buffer_mutex_->try_lock()) {
      int has_new_message = static_cast<int>(buffer_[0]);
      if (has_new_message == 1) {
          int sensor_msg_type = static_cast<int>(buffer_[1]);
          int data_size = static_cast<int>(buffer_[2]);
          std::cout << "Will try to read sensor message of type: " << sensor_msg_type << " size: " << data_size << std::endl;
          if (message.ParseFromArray(buffer_ + 3, data_size)) {
              status = SensorDataManagerReadStatus::SUCCESS;
              std::cout << "Did read successfully from buffer" << std::endl;

              // Update the first value in the buffer to 0 (implying that we read it correctly?)
              buffer_[0] = 0;

          } else {
              status = SensorDataManagerReadStatus::FAIL_TO_READ;
              std::cout << "Protobuf failed to read message from memory" << std::endl;
          }


      } else {
          std::cout << "No new sensor message" << std::endl;
          status = SensorDataManagerReadStatus::NO_NEW_MESSAGE;
      }

      buffer_mutex_->unlock();
    }
  } catch (boost::interprocess::lock_exception) {
    status = SensorDataManagerReadStatus::FAIL_TO_GET_LOCK;
    // Do nothing
  }

  return status;
}

