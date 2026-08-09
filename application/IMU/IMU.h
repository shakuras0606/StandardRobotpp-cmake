#ifndef IMU_H
#define IMU_H

#include "struct_typedef.h"
#include "macro_typedef.h"
#include "IMU_task.h"

static inline float GetImuAngle(uint8_t axis)
{
    return get_INS_angle_point()[axis];
}

static inline float GetImuVelocity(uint8_t axis)
{
    return get_gyro_data_point()[axis];
}

static inline float GetImuAccel(uint8_t axis)
{
    return get_accel_data_point()[axis];
}

extern float GetYawBias(void);

extern float get_raw_gyro(uint8_t axis);
extern float get_raw_accel(uint8_t axis);

#endif  // IMU_H
/*------------------------------ End of File ------------------------------*/
