#ifndef MAIN_IAM_ROBOLIB_SENSOR_DATA_MANAGER_H
#define MAIN_IAM_ROBOLIB_SENSOR_DATA_MANAGER_H

#include <iam_robolib_common/definitions.h>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include "sensor_msg.pb.h"

class SensorDataManager {
    public:
    SensorDataManager(SensorBufferTypePtr buffer,
                      boost::interprocess::interprocess_mutex *mutex) :
                      buffer_(buffer),
                      buffer_mutex_(mutex)
                      {};

    /**
     * Read the current bounding box message.
     * @param message
     * @return
     */
    SensorDataManagerReadStatus readBoundingBoxMessage(BoundingBox& message);


    /**
     * Read the sensor message with joint positions for @joint_position_dynamic_interp_skill
     * @param message
     * @return
     */
    SensorDataManagerReadStatus readJointSensorInfoMessage(JointSensorInfo& message);

 private:
    SensorBufferTypePtr buffer_;
    boost::interprocess::interprocess_mutex* buffer_mutex_= nullptr;

    /**
     * Get current message size.
     * @return
     */
    float getMessageSize();

    SensorDataManagerReadStatus readMessageAsBytes(std::function< bool(const void *bytes, int data_size)> parse_callback);
};

#endif //MAIN_IAM_ROBOLIB_SENSOR_DATA_MANAGER_H
