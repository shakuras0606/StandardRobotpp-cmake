#ifndef POS_DEBUG_ROLL_H
#define POS_DEBUG_ROLL_H

#include <math.h>

typedef struct {
    float integral;
    float p_out;
    float i_out;
    float d_out;
    float output;
} PosDebugRollPid_s;

static inline float PosDebugRollClamp(float value, float limit)
{
    if (limit <= 0.0f) return 0.0f;
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/* Convert the installed IMU's raw Euler roll into the controller coordinate.
 * remainderf keeps the result continuous across the +/-pi representation cut.
 */
static inline float PosDebugRollFeedback(
    float raw_roll, float zero_roll, float direction)
{
    const float two_pi = 6.28318530718f;
    return direction * remainderf(raw_roll - zero_roll, two_pi);
}

static inline void PosDebugRollPidReset(PosDebugRollPid_s *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;
}

/* Roll outer loop.  Output is the complete left-minus-right leg-length
 * reference difference in metres.  Integral and derivative terms use seconds
 * and rad/s explicitly, so their tuning does not depend on the task period.
 */
static inline float PosDebugRollPidStep(
    PosDebugRollPid_s *pid,
    float feedback,
    float reference,
    float measured_rate,
    float dt,
    float kp,
    float ki,
    float kd,
    float max_iout,
    float max_output)
{
    if (pid == 0) return 0.0f;

    const float error = reference - feedback;
    pid->p_out = kp * error;

    if (ki > 0.0f && dt > 0.0f && max_iout > 0.0f) {
        pid->integral += error * dt;
        pid->i_out = PosDebugRollClamp(ki * pid->integral, max_iout);
        pid->integral = pid->i_out / ki;
    } else {
        pid->integral = 0.0f;
        pid->i_out = 0.0f;
    }

    pid->d_out = -kd * measured_rate;
    pid->output = PosDebugRollClamp(
        pid->p_out + pid->i_out + pid->d_out, max_output);
    return pid->output;
}

#endif
