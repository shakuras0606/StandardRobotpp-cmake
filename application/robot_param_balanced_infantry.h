/**
  * @file       robot_param_balanced_infantry.h
  * @brief      这里是平衡步兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_BALANCE             // 选择底盘类型

// clang-format off
#define __SELF_BOARD_ID C_BOARD_BALANCE_CHASSIS // 本板ID
#define __GYRO_BIAS_YAW  0.003096855f           // 陀螺仪零飘，单位rad/s

#define __CONTROL_LINK_RC  CL_RC_DIRECT  // 控制链路选择：遥控器接收机接底盘C板UART3
#define __CONTROL_LINK_KM  CL_KM_RC      // 控制链路选择：键鼠数据

#define __VIRTUAL_GIMBAL_FROM VG_FROM_CAN // 虚拟云台数据来源（用于云台底盘分离控制）

/*-------------------- Chassis --------------------*/
// 底盘任务相关宏定义
#define CHASSIS_TASK_INIT_TIME 357   // 任务开始空闲一段时间
#define CHASSIS_CONTROL_TIME_MS 1    // 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)   // 底盘任务控制间隔

// 底盘的遥控器相关宏定义 ---------------------
#define CHASSIS_MODE_CHANNEL   0  // 选择底盘状态 开关通道号
#define CHASSIS_X_CHANNEL      3  // 前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL      2  // 左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL     2  // 旋转的遥控器通道号码
#define CHASSIS_ANGLE_CHANNEL  4  // 腿摆角的遥控器通道号码
#define CHASSIS_LENGTH_CHANNEL 4  // 腿长的遥控器通道号码
#define CHASSIS_ROLL_CHANNEL   4  // ROLL角的遥控器通道号码
#define CHASSIS_RC_DEADLINE    20 // 摇杆死区

// POS_DEBUG 遥控前后运动与三档腿长参数
#define POS_DEBUG_X_CHANNEL            1        // 遥控器 ch[1] 控制前进/后退
#define POS_DEBUG_X_REF_SPEED_MAX      (1.00f)  // (m/s) 满杆目标速度 0.3
#define POS_DEBUG_X_REF_ACCEL_MAX      (1.00f)  // (m/s^2) 加减速斜坡；0.5
#define POS_DEBUG_X_REF_DIRECTION      (1.0f)   // 前后控制极性；方向相反时改为 -1.0f
#define POS_DEBUG_MAX_JOINT_TORQUE     (3.0f)   // (N*m) 架空调试关节力矩限幅
#define POS_DEBUG_LEG_FORCE_DIRECTION  (1.0f)   // 正误差对应伸腿；方向反了改为 -1.0f
#define POS_DEBUG_LEG_LENGTH_MID_REF    (0.20f)  // (m) 模式开关中档腿长目标
#define POS_DEBUG_LEG_LENGTH_UP_REF     (0.35f)  // (m) 模式开关上档腿长目标
#define POS_DEBUG_LEG_LENGTH_RATE_MAX   (0.20f)  // (m/s) 中档与上档切换时的最大腿长变化速度
#define POS_DEBUG_GRAVITY_FF_RATIO      (0.30f)  // POS_DEBUG 重力前馈比例；下地初调从 20% 开始

// POS_DEBUG 腿长相关 theta 平衡点。
// 控制器使用 theta_error = theta_fdb - theta_ref；等价的 X0_OFFSET 为 -theta_ref。
// 当前采用固定腿长数据的第一轮半量修正：短腿偏置较大，随腿长增加逐渐减小。
// 后续结合 x、x_dot、phi、phi_dot 和轮力矩分量，再继续收敛下面四组标定值。
#define POS_DEBUG_THETA_REF_L0_0       (0.15f)
#define POS_DEBUG_THETA_REF_VALUE_0    (0.085f)
#define POS_DEBUG_THETA_REF_L0_1       (0.20f)
#define POS_DEBUG_THETA_REF_VALUE_1    (0.070f)
#define POS_DEBUG_THETA_REF_L0_2       (0.30f)
#define POS_DEBUG_THETA_REF_VALUE_2    (0.045f)
#define POS_DEBUG_THETA_REF_L0_3       (0.38f)
#define POS_DEBUG_THETA_REF_VALUE_3    (0.025f)

// POS_DEBUG 六状态 LQR 调试参数（theta、theta_dot、x、x_dot、phi、phi_dot）。
// 缩放腿端虚拟髋关节力矩 Tp；1.0f 表示使用完整六状态增益。
#define POS_DEBUG_LEG_ANGLE_GAIN_SCALE    (1.00f)

// 虚拟髋关节力矩 Tp 的绝对值上限，单位 N*m。
// Tp 经 VMC 映射到四个关节电机，用于控制腿部摆角；当前限制为 [-0.50, +0.50] N*m。
// 该值只负责安全限幅，不是 LQR 增益；过小会导致腿摆角控制力不足，过大可能造成关节冲击。
#define POS_DEBUG_MAX_LEG_TP              (15.00f)

// 左右腿的 Tp 执行器极性。两侧五连杆为镜像结构，相同 Tp 经 VMC 后产生的摆角方向相反，
// 因此必须分别设置极性；只改变 LQR 输出到关节电机的最终方向，不改变 theta 状态定义。
#define POS_DEBUG_LEFT_LEG_TP_DIRECTION   (-1.0f)
#define POS_DEBUG_RIGHT_LEG_TP_DIRECTION  (-1.0f)


// POS_DEBUG 驱动轮 LQR 输出缩放。
// 当前 1.0f 沿用已经实现基础平衡的实车调试值。
#define POS_DEBUG_WHEEL_LQR_GAIN_SCALE       (1.00f)

// 单个驱动轮允许的最大 LQR 力矩，单位 N*m。
#define POS_DEBUG_MAX_WHEEL_TORQUE            (5.00f)

// 驱动轮最终执行极性，只在最终电机赋值处改变方向，不修改状态量定义。
#define POS_DEBUG_WHEEL_TORQUE_DIRECTION     (1.0f)



/*================ POS_DEBUG 腿部劈叉抑制 ================*/

// 一键开关：0 关闭，1 开启。
#define POS_DEBUG_LEG_SPLIT_ENABLE          (1)

// 劈叉抑制输出方向。
// 如果 theta_left - theta_right 越控越大，只修改这里为 -1.0f。
#define POS_DEBUG_LEG_SPLIT_DIRECTION       (-1.0f)

// 左右腿摆角差 PD。
// 输入：theta_left - theta_right、theta_dot_left - theta_dot_right。
// 输出：差动力矩 Tp_split，单位 N*m。
#define KP_POS_DEBUG_LEG_SPLIT              (1.80f)  // N*m/rad
#define KD_POS_DEBUG_LEG_SPLIT              (0.03f)  // N*m/(rad/s)

// 劈叉抑制最大差动力矩。
// 建议初期不超过 POS_DEBUG_MAX_LEG_TP 的 10% 左右。
#define MAX_OUT_POS_DEBUG_LEG_SPLIT         (0.30f)  // N*m


/*================ POS_DEBUG Yaw 双环控制 ================*/

// Yaw 总开关：0 关闭，1 开启。
#define POS_DEBUG_YAW_ENABLE                (1)

// 0：遥控器直接控制 yaw 角速度，用于先调内环。
// 1：遥控器以 add 形式改变 yaw 目标角度，用于最终双环控制。
#define POS_DEBUG_YAW_ANGLE_LOOP_ENABLE     (1)

// 遥控器满杆时 yaw 目标角度的最大变化速度。
#define POS_DEBUG_YAW_REF_RATE_MAX          (1.50f)  // rad/s 0.5
#define POS_DEBUG_YAW_REF_LEAD_MAX          (0.50f)  // rad，目标相对反馈最大超前量，必须小于 PI
#define POS_DEBUG_YAW_RATE_FF               (1.00f)  // 目标角速度前馈系数；0 关闭，1 完整前馈

// Yaw 差动力矩方向。
// 正 yaw 误差产生的差动力矩若让 yaw 误差增大，则修改为 -1.0f。
#define POS_DEBUG_YAW_TORQUE_DIRECTION      (1.0f)



// deadzone parameters ---------------------
#define WHEEL_DEADZONE (0.01f)  // (m/s)轮子速度死区

// ratio parameters ---------------------
#define FF_RATIO               (0.8f)   // 前馈比例系数
#define ROLL_VEL_LIMIT_FACTOR  (0.1f)    // roll角速度抑制比例系数

// motor parameters ---------------------
#define JOINT_CAN (1)
#define WHEEL_CAN (1)

#define J0_DIRECTION (1)//-1，下同
#define J1_DIRECTION (1)
#define J2_DIRECTION (-1)
#define J3_DIRECTION (-1)

#define W0_DIRECTION ( 1)
#define W1_DIRECTION (-1)

// DM控制参数
#define CALIBRATE_VEL_KP  (4.0f)  // 校准MIT速度控制KP
#define DEBUG_VEL_KP      (4.0f)  // 调试MIT速度控制KP
#define ZERO_FORCE_VEL_KP (4.0f)  // 无力MIT速度控制KP

#define NORMAL_POS_KP (20.0f) // 正常MIT位置控制KP
#define NORMAL_POS_KD (1.0f)  // 正常MIT位置控制KD

#define DEBUG_POS_KP (8.0f) // 调试MIT位置控制KP
#define DEBUG_POS_KD (0.8f) // 调试MIT位置控制KD

//physical parameters ---------------------
#define LEG_L1 (0.210f)  // (m)腿1长度
#define LEG_L2 (0.250f)  // (m)腿2长度
#define LEG_L3 (LEG_L2)  // (m)腿3长度
#define LEG_L4 (LEG_L1)  // (m)腿4长度
#define LEG_L5 (0.001f)    // (m)关节间距

#define BODY_MASS            (18.0f)      // (kg)机身重量
#define WHEEL_MASS           (0.65f)      // (kg)轮子重量
#define WHEEL_RADIUS         (0.068f)    // (m)轮子半径
#define WHEEL_START_TORQUE   (0.3f)      // (Nm)轮子起动力矩
#define WHEEL_BASE           (0.51175f)  // (m)驱动轮轴距

// #define J0_ANGLE_OFFSET     (+0.19163715f + M_PI) // (rad)关节0角度偏移量(电机0点到水平线的夹角)
// #define J1_ANGLE_OFFSET     (M_PI-0.19163715f )         // (rad)关节1角度偏移量(电机0点到水平线的夹角)
// #define J2_ANGLE_OFFSET     (-0.19163715f + M_PI)  // (rad)关节2角度偏移量(电机0点到水平线的夹角)
// #define J3_ANGLE_OFFSET     (M_PI+0.19163715f)        // (rad)关节3角度偏移量(电机0点到水平线的夹角)

#define J0_ANGLE_OFFSET     (+0.41364303f + M_PI) // (rad)关节0角度偏移量(电机0点到水平线的夹角)
#define J1_ANGLE_OFFSET     (M_PI-0.93043502f )         // (rad)关节1角度偏移量(电机0点到水平线的夹角)
#define J2_ANGLE_OFFSET     (-0.41364303f + M_PI)  // (rad)关节2角度偏移量(电机0点到水平线的夹角)
#define J3_ANGLE_OFFSET     (M_PI+0.93043502f)        // (rad)关节3角度偏移量(电机0点到水平线的夹角)

// #define J0_ANGLE_OFFSET     (0.19163715f + M_PI) // (rad)关节0角度偏移量(电机0点到水平线的夹角)
// #define J1_ANGLE_OFFSET     (-0.19163715f + M_PI)         // (rad)关节1角度偏移量(电机0点到水平线的夹角)
// #define J2_ANGLE_OFFSET     (0.19163715f + M_PI)  // (rad)关节2角度偏移量(电机0点到水平线的夹角)
// #define J3_ANGLE_OFFSET     (-0.19163715f + M_PI)        // (rad)关节3角度偏移量(电机0点到水平线的夹角)


#define DLENGTH_DIRECTION  (-1) // ROLL角补偿量方向(腿长增加方向)

//upper_limit parameters ---------------------

#define MAX_DELTA_ROD_ANGLE (0.40f) // (rad)腿摆角最大变化量
#define MAX_TORQUE_PROTECT  (25.0f)  // (Nm)最大扭矩保护

#define MAX_DELTA_VEL_FDB_TO_REF (0.8f) // (m/s)速度反馈到参考速度的最大变化量

#define MAX_THETA      (1.0f)
#define MAX_THETA_DOT  (2.0f)
#define MAX_X          (1.0f)
#define MAX_X_DOT      (10.0f)
#define MAX_PHI        (1.0f)
#define MAX_PHI_DOT    (2.0f)

#define MAX_SPEED_INTEGRAL  (0.5f)
#define MAX_ROLL            (0.3f)
#define MAX_ROLL_VELOCITY   (1.0f)
#define MAX_YAW             (M_PI)
#define MAX_YAW_VELOCITY    (3.0f)

#define MAX_J0_ANGLE  (M_PI) // (rad)关节角度上限
#define MAX_J1_ANGLE  (M_PI) // (rad)关节角度上限
#define MAX_J2_ANGLE  (M_PI) // (rad)关节角度上限
#define MAX_J3_ANGLE  (M_PI) // (rad)关节角度上限

#define MAX_LEG_LENGTH       (0.4f)
#define MAX_LEG_ANGLE        (M_PI_2 + MAX_DELTA_ROD_ANGLE)
#define MAX_SPEED            (3.5f)
#define MAX_SPEED_VECTOR_VX  (3.5f)
#define MAX_SPEED_VECTOR_VY  (3.5f)
#define MAX_SPEED_VECTOR_WZ  (6.0f)

#define MAX_JOINT_TORQUE      (15.0f)   // (Nm)关节最大扭矩
#define MAX_JOINT_TORQUE_JUMP (20.0f)  // (Nm)跳跃时的关节最大扭矩
#define MAX_VEL_ADD           (1.0f)   // (m/s)速度增量上限
#define MAX_PITCH_VEL         (0.1f)   // (rad/s)pitch轴速度上限

#define MAX_TOUCH_INTERVAL    (200)    // (ms)最大离地时间，超过这个时间认为离地

//lower_limit parameters ---------------------

#define MIN_DELTA_ROD_ANGLE (-MAX_DELTA_ROD_ANGLE) // (rad)腿摆角最小变化量

#define MIN_DELTA_VEL_FDB_TO_REF (-MAX_DELTA_VEL_FDB_TO_REF) // (m/s)速度反馈到参考速度的最小变化量

#define MIN_THETA      (-MAX_THETA)
#define MIN_THETA_DOT  (-MAX_THETA_DOT)
#define MIN_X          (-MAX_X)
#define MIN_X_DOT      (-MAX_X_DOT)
#define MIN_PHI        (-MAX_PHI)
#define MIN_PHI_DOT    (-MAX_PHI_DOT)

#define MIN_SPEED_INTEGRAL  (-MAX_SPEED_INTEGRAL)
#define MIN_ROLL            (-MAX_ROLL)
#define MIN_ROLL_VELOCITY   (-MAX_ROLL_VELOCITY)
#define MIN_YAW             (-MAX_YAW)
#define MIN_YAW_VELOCITY    (-MAX_YAW_VELOCITY)

#define MIN_J0_ANGLE (-M_PI) // (rad)关节角度下限
#define MIN_J1_ANGLE (-M_PI) // (rad)关节角度下限
#define MIN_J2_ANGLE (-M_PI) // (rad)关节角度下限
#define MIN_J3_ANGLE (-M_PI) // (rad)关节角度下限

#define MIN_LEG_LENGTH       ( 0.1f)
#define MIN_LEG_ANGLE        ( M_PI_2 - MAX_DELTA_ROD_ANGLE)
#define MIN_SPEED            (-MAX_SPEED)
#define MIN_SPEED_VECTOR_VX  (-MAX_SPEED_VECTOR_VX)
#define MIN_SPEED_VECTOR_VY  (-MAX_SPEED_VECTOR_VY)
#define MIN_SPEED_VECTOR_WZ  (-MAX_SPEED_VECTOR_WZ)

#define MIN_JOINT_TORQUE      (-MAX_JOINT_TORQUE)  // 
#define MIN_JOINT_TORQUE_JUMP (-MAX_JOINT_TORQUE_JUMP)  // 
#define MIN_VEL_ADD           (-MAX_VEL_ADD)    // (m/s)速度增量下限
#define MIN_PITCH_VEL         (-MAX_PITCH_VEL)  // (rad/s)pitch轴速度下限

// Yaw 角度外环：输出目标角速度 rad/s。
// 初期只使用 P，避免积分导致目标方向缓慢漂移。
#define KP_CHASSIS_YAW_ANGLE                (1.50f)
#define KI_CHASSIS_YAW_ANGLE                (0.00f)
#define KD_CHASSIS_YAW_ANGLE                (0.00f)
#define MAX_IOUT_CHASSIS_YAW_ANGLE          (0.00f)
#define MAX_OUT_CHASSIS_YAW_ANGLE           (1.00f)  // rad/s

// Yaw 角速度内环：输出左右轮差动力矩 N*m。
// 初期只使用 P，先确认极性和响应。
#define KP_CHASSIS_YAW_VELOCITY             (2.00f)
#define KI_CHASSIS_YAW_VELOCITY             (0.001f)
#define KD_CHASSIS_YAW_VELOCITY             (0.00f)
#define MAX_IOUT_CHASSIS_YAW_VELOCITY       (0.00f)
#define MAX_OUT_CHASSIS_YAW_VELOCITY        (2.00f)  // N*m

// vel_add PID参数
#define KP_CHASSIS_VEL_ADD        (0.1f) //0.1
#define KI_CHASSIS_VEL_ADD        (0.005f)//0.005
#define KD_CHASSIS_VEL_ADD        (0.001f)//0.001
#define MAX_IOUT_CHASSIS_VEL_ADD  (0.8f)//0.5
#define MAX_OUT_CHASSIS_VEL_ADD   (1.5f)//1.0

/*========== Start of locomotion control pid ==========*/

/* POS_DEBUG Roll -> 左右腿长差控制。
 * PID 输出定义为 left_L0_ref - right_L0_ref，单位 m。
 * 首轮实车只开启 P；确认 gyro-X 的方向后再逐步加入 KD。
 */
#define POS_DEBUG_ROLL_ENABLE                  (1)
#define POS_DEBUG_ROLL_ZERO_RAD                (M_PI)
#define POS_DEBUG_ROLL_ANGLE_DIRECTION         (-1.0f)
#define POS_DEBUG_ROLL_RATE_DIRECTION          (1.0f)
#define POS_DEBUG_ROLL_REF_MAX                 (0.080f)  // rad，约 4.6 deg
#define POS_DEBUG_ROLL_CONTROL_DT_MAX_S        (0.020f)

#define KP_CHASSIS_ROLL_ANGLE                  (1.00f)   // m/rad
#define KI_CHASSIS_ROLL_ANGLE                  (0.05f)   // m/(rad*s)
#define KD_CHASSIS_ROLL_ANGLE                  (0.00f)   // m/(rad/s)
#define MAX_IOUT_CHASSIS_ROLL_ANGLE            (0.00f)   // m
#define MAX_OUT_CHASSIS_ROLL_ANGLE             (0.020f)  // m，总腿长差限幅
#define N_CHASSIS_ROLL_ANGLE                   (0.1f)

//roll轴跟踪速度环PID参数
// #define KP_CHASSIS_ROLL_VELOCITY        (0.6f)
// #define KI_CHASSIS_ROLL_VELOCITY        (0.0001f)
// #define KD_CHASSIS_ROLL_VELOCITY        (0.005f)
// #define MAX_IOUT_CHASSIS_ROLL_VELOCITY  (0.01f)
// #define MAX_OUT_CHASSIS_ROLL_VELOCITY   (0.1f)

// 腿长跟踪长度环PID参数
#define KP_CHASSIS_LEG_LENGTH_LENGTH        (150.0f)
#define KI_CHASSIS_LEG_LENGTH_LENGTH        (0.1f)
#define KD_CHASSIS_LEG_LENGTH_LENGTH        (1500.0f)
#define MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH  (0.0f)
#define MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH   (40.0f)
#define N_LEG_LENGTH_LENGTH                 (0.1f)

// POS_DEBUG 纯腿长力环 PID 参数，输出为沿杆方向的虚拟力(N)
#define KP_POS_DEBUG_LEG_LENGTH_FORCE        (160.0f)
#define KI_POS_DEBUG_LEG_LENGTH_FORCE        (0.5f)
#define KD_POS_DEBUG_LEG_LENGTH_FORCE        (1500.0f)
#define MAX_IOUT_POS_DEBUG_LEG_LENGTH_FORCE  (10.0f)
#define MAX_OUT_POS_DEBUG_LEG_LENGTH_FORCE   (40.0f)
#define N_POS_DEBUG_LEG_LENGTH_FORCE         (0.1f)

// 腿长跟踪速度环PID参数
// #define KP_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define KI_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define KD_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define MAX_IOUT_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define MAX_OUT_CHASSIS_LEG_LENGTH_SPEED 0.0f

/*========== End of locomotion control pid ==========*/

// 起立用的pid
#define KP_CHASSIS_STAND_UP       (2000.0f)
#define KI_CHASSIS_STAND_UP       (0.0f)
#define KD_CHASSIS_STAND_UP       (10.0f)
#define MAX_IOUT_CHASSIS_STAND_UP (0.0f)
#define MAX_OUT_CHASSIS_STAND_UP  (2000.0f)

// 轮子停止用的pid
#define KP_CHASSIS_WHEEL_STOP       (100.0f)
#define KI_CHASSIS_WHEEL_STOP       (0.0f)
#define KD_CHASSIS_WHEEL_STOP       (40.0f)
#define MAX_IOUT_CHASSIS_WHEEL_STOP (0.0f)
#define MAX_OUT_CHASSIS_WHEEL_STOP  (2000.0f)

// 云台跟随用的pid
#define KP_CHASSIS_FOLLOW_GIMBAL       (5.0f)
#define KI_CHASSIS_FOLLOW_GIMBAL       (0.0f)
#define KD_CHASSIS_FOLLOW_GIMBAL       (0.0f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (0.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL  (6.0f)

//LPF parameters ---------------------
#define LEG_DDL0_LPF_ALPHA           (0.9f)
#define LEG_DDPHI0_LPF_ALPHA         (0.9f)
#define LEG_DDTHETA_LPF_ALPHA        (0.9f)
#define LEG_SUPPORT_FORCE_LPF_ALPHA  (0.9f)
#define CHASSIS_ROLL_ALPHA           (0.5f)

//offest parameters ---------------------

#define X0_OFFSET (-0.06f)   // 非 POS_DEBUG 模式沿用的固定 theta 偏置
#define X1_OFFSET (0.0f)   // 目标theta_dot偏移量
#define X2_OFFSET (0.0f)   // 目标x偏移量 -0.4
#define X3_OFFSET (0.0f)   // 目标x_dot偏移量
#define X4_OFFSET (0.0f)   // 目标phi偏移量
#define X5_OFFSET (0.0f)   // 目标phi_dot偏移量

//other parameters ---------------------

#define WHEEL_REDUCTION_RATIO (268.0f / 17.0f)    //xroll减速比
#define WHEEL_TORQUE_TO_3508_CMD  3287.0f      //(16384.0f / (20.0f * 0.3f * (268.0f / 17.0f)))  3287  173
#define WHEEL_3508_CMD_MAX        16384.0f        
#define WHEEL_DJI_STDID           0x200

// clang-format on
#endif /* INCLUDED_ROBOT_PARAM_H */
