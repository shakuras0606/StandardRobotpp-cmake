#ifndef POS_DEBUG_REFERENCE_H
#define POS_DEBUG_REFERENCE_H

#include <math.h>
#include <stdbool.h>

typedef struct {
    bool powered;
    float leg_length_target;
} PosDebugSwitchCommand_s;

/* Translate the three-position mode switch into the powered state and leg target.
 * Up has priority if malformed input ever reports both powered positions at once.
 */
static inline PosDebugSwitchCommand_s PosDebugSelectSwitchCommand(
    bool is_up, bool is_mid, float mid_leg_length, float up_leg_length)
{
    PosDebugSwitchCommand_s command = {
        .powered = false,
        .leg_length_target = mid_leg_length,
    };

    if (is_up) {
        command.powered = true;
        command.leg_length_target = up_leg_length;
    } else if (is_mid) {
        command.powered = true;
    }

    return command;
}

/* Rate limit in physical units; no position/travel limit is applied. */
static inline float PosDebugVelocityStep(float current, float target,
                                        float acceleration, float dt)
{
    const float step = acceleration * dt;
    if (step <= 0.0f) return current;
    if (target - current > step) return current + step;
    if (target - current < -step) return current - step;
    return target;
}

/* Bound target lead, not absolute heading. This permits continuous turns
 * without allowing a stalled robot's target to run past the +/-pi error cut.
 * Releasing the stick holds the target, even after an external yaw disturbance.
 */
static inline float PosDebugYawReferenceStep(float reference, float measured,
                                            float rate, float dt, float max_lead)
{
    const float two_pi = 6.28318530718f;
    if (rate == 0.0f) return remainderf(reference, two_pi);
    float error = remainderf(reference - measured, two_pi) + rate * dt;
    if (error > max_lead) error = max_lead;
    if (error < -max_lead) error = -max_lead;
    return remainderf(measured + error, two_pi);
}

#endif
