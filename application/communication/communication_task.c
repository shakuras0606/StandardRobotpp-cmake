#include "communication_task.h"

#include "CAN_receive.h"
#include "IMU.h"
#include "IMU_solve.h"
#include "IMU_task.h"
#include "ahrs.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "chassis_balance.h"
#include "chassis_balance_extras.h"
#include "cmsis_os.h"
#include "communication.h"
#include "data_exchange.h"
#include "detect_task.h"
#include "main.h"
#include "math.h"
#include "pid.h"
#include "pos_debug_roll.h"
#include "robot_param.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "usb_debug.h"
#include "Vofa.h"
#include <string.h>

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t communication_high_water;
#endif

/*==================== 配置 ====================*/
#define COMMUNICATION_TASK_INIT_TIME_MS 100U

/*
 * 只需修改 VOFA_SEND_HZ 即可改变发送频率。
 * 28 通道 + JustFloat 帧尾共 116 byte；115200 baud 下推荐 40~50 Hz。
 */
#ifndef VOFA_SEND_HZ
#define VOFA_SEND_HZ 50U
#endif

#define VOFA_CH_COUNT       28U
#define VOFA_SEND_PERIOD_MS (1000U / VOFA_SEND_HZ)

#if VOFA_SEND_HZ == 0U
#error "VOFA_SEND_HZ must be greater than zero"
#elif VOFA_SEND_HZ > 50U
#error "VOFA_SEND_HZ above 50 Hz is unsafe for the current 28-channel 115200-baud link"
#elif (1000U % VOFA_SEND_HZ) != 0U
#error "VOFA_SEND_HZ must divide 1000 exactly (for example 20, 25, 40 or 50 Hz)"
#endif

extern Chassis_s CHASSIS;

/*
 * CH2 状态字（整数小于 2^24，存入 float 后仍可精确表示）：
 * bit  0..3 : CHASSIS.mode
 * bit  4..8 : CHASSIS.error_code
 * bit  9/10 : 左/右腿离地
 * bit 11/12 : 左/右轮力矩达到软件限幅
 * bit 13/14 : 左/右腿长力达到软件限幅
 * bit 15/16 : 左/右髋关节 Tp 达到软件限幅
 */
static float BuildStatusWord(void)
{
    const float saturation_epsilon = 1e-4f;
    uint32_t status = ((uint32_t)CHASSIS.mode & 0x0fU) |
        (((uint32_t)CHASSIS.error_code & 0x1fU) << 4U);

    if (CHASSIS.fdb.leg[0].is_take_off) {
        status |= (1UL << 9U);
    }
    if (CHASSIS.fdb.leg[1].is_take_off) {
        status |= (1UL << 10U);
    }
    if (fabsf(CHASSIS.cmd.leg[0].wheel.T) >=
        POS_DEBUG_MAX_WHEEL_TORQUE - saturation_epsilon) {
        status |= (1UL << 11U);
    }
    if (fabsf(CHASSIS.cmd.leg[1].wheel.T) >=
        POS_DEBUG_MAX_WHEEL_TORQUE - saturation_epsilon) {
        status |= (1UL << 12U);
    }
    if (fabsf(CHASSIS.cmd.leg[0].rod.F) >=
        MAX_OUT_POS_DEBUG_LEG_LENGTH_FORCE - saturation_epsilon) {
        status |= (1UL << 13U);
    }
    if (fabsf(CHASSIS.cmd.leg[1].rod.F) >=
        MAX_OUT_POS_DEBUG_LEG_LENGTH_FORCE - saturation_epsilon) {
        status |= (1UL << 14U);
    }
    if (fabsf(CHASSIS.cmd.leg[0].rod.Tp) >=
        POS_DEBUG_MAX_LEG_TP - saturation_epsilon) {
        status |= (1UL << 15U);
    }
    if (fabsf(CHASSIS.cmd.leg[1].rod.Tp) >=
        POS_DEBUG_MAX_LEG_TP - saturation_epsilon) {
        status |= (1UL << 16U);
    }

    return (float)status;
}



static void FillFloatBuffer(float *buf)
{
    const float theta_error_left =
        CHASSIS.fdb.leg_state[0].theta - CHASSIS.ref.leg_state[0].theta;
    const float theta_error_right =
        CHASSIS.fdb.leg_state[1].theta - CHASSIS.ref.leg_state[1].theta;
    const float x_error_left =
        CHASSIS.fdb.leg_state[0].x - CHASSIS.ref.leg_state[0].x;
    const float x_error_right =
        CHASSIS.fdb.leg_state[1].x - CHASSIS.ref.leg_state[1].x;
    const float x_dot_error_left =
        CHASSIS.fdb.leg_state[0].x_dot - CHASSIS.ref.leg_state[0].x_dot;
    const float x_dot_error_right =
        CHASSIS.fdb.leg_state[1].x_dot - CHASSIS.ref.leg_state[1].x_dot;

    // 公共信息
    buf[0] = (float)CHASSIS.last_time;  // CH0: control_time_ms
    buf[1] = (float)CHASSIS.duration;   // CH1: control_dt_ms
    buf[2] = BuildStatusWord();         // CH2: status_word

    // LQR 六状态。avg=(L+R)/2，diff=(L-R)/2，可还原左右原值。
    buf[3] = 0.5f * (theta_error_left + theta_error_right);  // CH3: theta_error_avg (rad)
    buf[4] = 0.5f *
        (CHASSIS.fdb.leg_state[0].theta_dot +
         CHASSIS.fdb.leg_state[1].theta_dot);                // CH4: theta_dot_avg (rad/s)
    buf[5] = 0.5f * (theta_error_left - theta_error_right);  // CH5: theta_error_diff (rad)
    buf[6] = 0.5f *
        (CHASSIS.fdb.leg_state[0].theta_dot -
         CHASSIS.fdb.leg_state[1].theta_dot);                // CH6: theta_dot_diff (rad/s)
    buf[7] = 0.5f * (x_error_left + x_error_right);          // CH7: x_error_avg (m)
    buf[8] = 0.5f * (x_dot_error_left + x_dot_error_right);  // CH8: x_dot_error_avg (m/s)
    buf[9] = CHASSIS.fdb.leg_state[0].phi;                   // CH9: phi/pitch (rad)
    buf[10] = CHASSIS.fdb.leg_state[0].phi_dot;              // CH10: phi_dot (rad/s)

    // 腿长 PID：参考、反馈和输出均用平均/差分压缩。
    buf[11] = 0.5f *
        (CHASSIS.ref.rod_L0[0] + CHASSIS.ref.rod_L0[1]);  // CH11: L0_ref_avg (m)
    buf[12] = 0.5f *
        (CHASSIS.fdb.leg[0].rod.L0 + CHASSIS.fdb.leg[1].rod.L0);  // CH12: L0_avg (m)
    buf[13] = 0.5f *
        (CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0);  // CH13: L0_diff (m)
    buf[14] = 0.5f *
        (CHASSIS.cmd.leg[0].rod.F + CHASSIS.cmd.leg[1].rod.F);  // CH14: F_avg (N)
    buf[15] = 0.5f *
        (CHASSIS.cmd.leg[0].rod.F - CHASSIS.cmd.leg[1].rod.F);  // CH15: F_diff (N)

    // Roll 腿长差 PID。角度减去安装零点，角速度转换到同一控制坐标。
    buf[16] = PosDebugRollFeedback(
        CHASSIS.fdb.body.roll,
        POS_DEBUG_ROLL_ZERO_RAD,
        POS_DEBUG_ROLL_ANGLE_DIRECTION);  // CH16: normalized_roll (rad)
    buf[17] = POS_DEBUG_ROLL_RATE_DIRECTION *
        CHASSIS.fdb.body.roll_dot;         // CH17: roll_dot (rad/s)
    buf[18] = CHASSIS.pid.roll_angle.out;  // CH18: L0_ref_left-L0_ref_right (m)

    // Yaw 双环 PID。
    buf[19] = CHASSIS.ref.body.yaw;          // CH19: yaw_ref (rad)
    buf[20] = CHASSIS.fdb.body.yaw;          // CH20: yaw (rad)
    buf[21] = CHASSIS.pid.yaw_velocity.set;  // CH21: yaw_dot_ref (rad/s)
    buf[22] = CHASSIS.fdb.body.yaw_dot;      // CH22: yaw_dot (rad/s)
    buf[23] = CHASSIS.pid.yaw_velocity.out;  // CH23: yaw_PID_out (N*m)

    // LQR/Yaw 叠加后的轮力矩与 LQR/劈叉叠加后的 Tp，仍使用平均/差分压缩。
    buf[24] = 0.5f *
        (CHASSIS.cmd.leg[0].wheel.T + CHASSIS.cmd.leg[1].wheel.T);  // CH24: wheel_T_avg (N*m)
    buf[25] = 0.5f *
        (CHASSIS.cmd.leg[0].wheel.T - CHASSIS.cmd.leg[1].wheel.T);  // CH25: wheel_T_diff (N*m)
    buf[26] = 0.5f *
        (CHASSIS.cmd.leg[0].rod.Tp + CHASSIS.cmd.leg[1].rod.Tp);  // CH26: Tp_avg (N*m)
    buf[27] = 0.5f *
        (CHASSIS.cmd.leg[0].rod.Tp - CHASSIS.cmd.leg[1].rod.Tp);  // CH27: Tp_diff (N*m)

    // NaN/Inf 安全过滤
    for (int i = 0; i < VOFA_CH_COUNT; i++) {
        if (!isfinite(buf[i])) {
            buf[i] = 0.0f;
        }
    }
}


/*==================== 主发送逻辑 ====================*/
static void CommunicationTaskStep(void)
{
    static float vofa_buf[VOFA_CH_COUNT];

    FillFloatBuffer(vofa_buf);

    Vofa_JustFloat(NULL, vofa_buf, VOFA_CH_COUNT);
}
/*==================== 任务入口 ====================*/
void communication_task(void const *pvParameters)
{
    (void)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(COMMUNICATION_TASK_INIT_TIME_MS));
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        CommunicationTaskStep();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(VOFA_SEND_PERIOD_MS));

#if INCLUDE_uxTaskGetStackHighWaterMark
        communication_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
