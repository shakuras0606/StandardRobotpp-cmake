/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信（仅接收，发送由 communication_task.c 的 VOFA+ 处理）
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done
  *  V1.1.0     2026-4-8        CJH             1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <stdbool.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "remote_control.h"
#include "supervisory_computer_cmd.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

#define USB_TASK_CONTROL_TIME 1  // ms

#define USB_OFFLINE_THRESHOLD 100  // ms
#define USB_CONNECT_CNT 10

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

// 判断USB连接状态用到的一些变量
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataPidDebug_s RECEIVE_PID_DEBUG_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void);
static void GetVirtualRcCtrlData(void);

/******************************************************************/
/* Task                                                           */
/******************************************************************/

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const * argument)
{
    Publish(&ROBOT_CMD_DATA, ROBOT_CMD_DATA_NAME);
    Publish(&USB_OFFLINE, USB_OFFLINE_NAME);
    Publish(&VIRTUAL_RC_CTRL, VIRTUAL_RC_NAME);

    MX_USB_DEVICE_Init();

    vTaskDelay(10);  //等待USB设备初始化完成
    UsbInit();

    while (1) {
        UsbReceiveData();
        GetCmdData();
        GetVirtualRcCtrlData();

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_PID_DEBUG_DATA, 0, sizeof(ReceiveDataPidDebug_s));
    memset(&RECEIVE_VIRTUAL_RC_DATA, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t * rx_data_start_address = USB_RX_BUF;  // 接收数据包时存放于缓存区的起始位置
    static uint8_t * rx_data_end_address;  // 接收数据包时存放于缓存区的结束位置
    uint8_t * sof_address = USB_RX_BUF;

    // 计算数据包的结束位置
    rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
    // 读取数据
    USB_Receive(rx_data_start_address, &len);  // Read data into the buffer

    while (sof_address <= rx_data_end_address) {  // 解析缓冲区中的所有数据包
        // 寻找帧头位置
        while (*(sof_address) != RECEIVE_SOF && (sof_address <= rx_data_end_address)) {
            sof_address++;
        }
        // 判断是否超出接收数据范围
        if (sof_address > rx_data_end_address) {
            break;  // 退出循环
        }
        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            // 检查整包CRC16校验 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, 4 + data_len + 2);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(ReceiveDataRobotCmd_s));
                    } break;
                    case PID_DEBUG_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_PID_DEBUG_DATA, sof_address, sizeof(ReceiveDataPidDebug_s));
                    } break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID: {
                        memcpy(
                            &RECEIVE_VIRTUAL_RC_DATA, sof_address, sizeof(ReceiveDataVirtualRc_s));
                    } break;
                    default:
                        break;
                }
                if (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP) {
                    LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4]));
                    RECEIVE_TIME = HAL_GetTick();
                }
            }
            sof_address += (data_len + HEADER_SIZE + 2);
        } else {  //CRC8校验失败，移动到下一个字节
            sof_address++;
        }
    }
    // 更新下一次接收数据的起始位置
    if (sof_address > rx_data_start_address + USB_RECEIVE_LEN) {
        // 缓冲区中没有剩余数据，下次接收数据的起始位置为缓冲区的起始位置
        rx_data_start_address = USB_RX_BUF;
    } else {
        uint16_t remaining_data_len = USB_RECEIVE_LEN - (sof_address - rx_data_start_address);
        // 缓冲区中有剩余数据，下次接收数据的起始位置为缓冲区中剩余数据的起始位置
        rx_data_start_address = USB_RX_BUF + remaining_data_len;
        // 将剩余数据移到缓冲区的起始位置
        memcpy(USB_RX_BUF, sof_address, remaining_data_len);
    }
}

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void)
{
    ROBOT_CMD_DATA.speed_vector.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    ROBOT_CMD_DATA.speed_vector.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    ROBOT_CMD_DATA.speed_vector.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;

    ROBOT_CMD_DATA.chassis.yaw = RECEIVE_ROBOT_CMD_DATA.data.chassis.yaw;
    ROBOT_CMD_DATA.chassis.pitch = RECEIVE_ROBOT_CMD_DATA.data.chassis.pitch;
    ROBOT_CMD_DATA.chassis.roll = RECEIVE_ROBOT_CMD_DATA.data.chassis.roll;
    ROBOT_CMD_DATA.chassis.leg_length = RECEIVE_ROBOT_CMD_DATA.data.chassis.leg_lenth;

    ROBOT_CMD_DATA.gimbal.yaw = RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw;
    ROBOT_CMD_DATA.gimbal.pitch = RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch;

    ROBOT_CMD_DATA.shoot.fire = RECEIVE_ROBOT_CMD_DATA.data.shoot.fire;
    ROBOT_CMD_DATA.shoot.fric_on = RECEIVE_ROBOT_CMD_DATA.data.shoot.fric_on;

    ROBOT_CMD_DATA.tracking.tracking = RECEIVE_ROBOT_CMD_DATA.data.tracking.tracking;
}

static void GetVirtualRcCtrlData(void)
{
    memcpy(&VIRTUAL_RC_CTRL, &RECEIVE_VIRTUAL_RC_DATA.data, sizeof(RC_ctrl_t));
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
 * @brief 获取上位机控制指令：云台姿态，基于欧拉角 r×p×y
 * @param axis 轴id，可配合定义好的轴id宏 AX_PITCH,AX_YAW 使用
 * @return (rad) 云台姿态
 */
inline float GetScCmdGimbalAngle(uint8_t axis)
{
    if (axis == AX_YAW) {
        return ROBOT_CMD_DATA.gimbal.yaw;
    } else if (axis == AX_PITCH) {
        return ROBOT_CMD_DATA.gimbal.pitch;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动线速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (m/s) 底盘坐标系下axis方向运动线速度
 */
inline float GetScCmdChassisSpeed(uint8_t axis)
{
    if (axis == AX_X)
    {
        return ROBOT_CMD_DATA.speed_vector.vx;
    } 
    else if (axis == AX_Y) 
    {
        return ROBOT_CMD_DATA.speed_vector.vy;
    }
    else if (axis == AX_Z)
    {
        return 0;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动角速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (rad/s) 底盘坐标系下axis方向运动角速度
 */
inline float GetScCmdChassisVelocity(uint8_t axis)
{
    if (axis == AX_Z)
    {
        return ROBOT_CMD_DATA.speed_vector.wz;
    } 
    return 0.0f;
}


/**
 * @brief 获取上位机控制指令：底盘离地高度，平衡底盘中可用作腿长参数
 * @param void
 * @return (m) 底盘离地高度
 */
inline float GetScCmdChassisHeight(void)
{
    return ROBOT_CMD_DATA.chassis.leg_length;
}

/**
 * @brief 获取上位机控制指令：开火
 * @param void
 * @return bool 是否开火
 */
inline uint8_t GetScCmdFire(void)
{
    return ROBOT_CMD_DATA.shoot.fire;
}


/**
 * @brief 获取上位机控制指令：启动摩擦轮
 * @param void
 * @return bool 是否启动摩擦轮
 */
inline uint8_t GetScCmdFricOn(void)
{
    return ROBOT_CMD_DATA.shoot.fric_on;
}

/**
 * @brief 获取上位机状态：发现敌方装甲板
 * @param void
 * @return bool 是否发现
 */
inline bool GetSCcmdtracking(void)
{
    return ROBOT_CMD_DATA.tracking.tracking;
}
/*------------------------------ End of File ------------------------------*/
