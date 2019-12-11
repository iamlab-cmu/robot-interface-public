#ifndef MAIN_IAM_ROBOLIB_BASE_SENSOR_DATA_WRAPPER_H
#define MAIN_IAM_ROBOLIB_BASE_SENSOR_DATA_WRAPPER_H
#include <iam_robolib_common/definitions.h>


class BaseSensorDataWrapper {
    public:
    BaseSensorDataWrapper(SharedBufferTypePtr buffer) : buffer_(buffer) {};

private:
    SharedBufferTypePtr buffer_ ;
};

#endif //MAIN_IAM_ROBOLIB_BASE_SENSOR_DATA_WRAPPER_H
