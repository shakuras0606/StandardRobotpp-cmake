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
#define COMMUNICATION_TASK_INIT_TIME 100
#define COMMUNICATION_TASK_TIME_MS   1
#define VOFA_CH_COUNT                16

extern Chassis_s CHASSIS;

/*==================== 数据填充 ====================*/

static void FillFloatBuffer(float *buf)
{
    // CH0-7: 左腿摆角环与腿长环
    buf[0] = CHASSIS.ref.leg_state[0].theta;                         // theta 目标 (rad)
    buf[1] = CHASSIS.fdb.leg_state[0].theta;                         // theta 实际 (rad)
    buf[2] = CHASSIS.ref.leg_state[0].theta -
             CHASSIS.fdb.leg_state[0].theta;                         // theta 误差 (rad)
    buf[3] = CHASSIS.fdb.leg_state[0].theta_dot;                     // theta_dot 实际 (rad/s)
    buf[4] = CHASSIS.cmd.leg[0].rod.Tp;                              // 摆角环输出 Tp (N*m)
    buf[5] = CHASSIS.ref.rod_L0[0];                                  // 腿长目标 (m)
    buf[6] = CHASSIS.fdb.leg[0].rod.L0;                              // 腿长实际 (m)
    buf[7] = CHASSIS.cmd.leg[0].rod.F;                               // 腿长 PID 输出 F (N)

    // CH8-15: 右腿摆角环与腿长环
    buf[8] = CHASSIS.ref.leg_state[1].theta;                         // theta 目标 (rad)
    buf[9] = CHASSIS.fdb.leg_state[1].theta;                         // theta 实际 (rad)
    buf[10] = CHASSIS.ref.leg_state[1].theta -
              CHASSIS.fdb.leg_state[1].theta;                       // theta 误差 (rad)
    buf[11] = CHASSIS.fdb.leg_state[1].theta_dot;                    // theta_dot 实际 (rad/s)
    buf[12] = CHASSIS.cmd.leg[1].rod.Tp;                             // 摆角环输出 Tp (N*m)
    buf[13] = CHASSIS.ref.rod_L0[1];                                 // 腿长目标 (m)
    buf[14] = CHASSIS.fdb.leg[1].rod.L0;                             // 腿长实际 (m)
    buf[15] = CHASSIS.cmd.leg[1].rod.F;                              // 腿长 PID 输出 F (N)

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
    vTaskDelay(COMMUNICATION_TASK_INIT_TIME);

    while (1) {
        CommunicationTaskStep();
        vTaskDelay(COMMUNICATION_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        communication_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
