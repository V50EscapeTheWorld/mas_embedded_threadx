/*
 * @Description: 云台功能模板 (云台板)
 *
 * 与单板版本的接口相同
 */

#ifndef _ARM_FUNC_H_
#define _ARM_FUNC_H_

#include "heavy_robot_def.h" 
#include "arm_ff_ctrl.h"

#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP)
#define GRIPPER_MOTOR_SPEEDE_MAP 5.0/783 //规定夹爪电机最大转速为5rad/s 
#endif
/* 状态结构体 */
typedef enum{
    SAFE_INIT,     //首次安全
    SAFE_RUN,      //安全运行
    UNSAFE_KEEP,   //运行不安全
    UNSAFE_STOP    //硬件不安全
}Safity_sta;

#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP)
/* 状态结构体 */
typedef enum{
    ARM_KEEP,  //保持姿态
    LOW_FOUR,  //下四轴
    HIGH_FOUR, //上三轴和夹爪
}Mode_sta;
#endif
/** 关节安全限制 **/
typedef struct
{
    float min_angle_rad;      ///< 最小模型关节角，单位 rad
    float max_angle_rad;      ///< 最大模型关节角，单位 rad
    float max_velocity_rad_s; ///< 最大关节角速度，单位 rad/s
    float max_torque_nm;      ///< 最大电机输出轴力矩，单位 Nm
} RoboticArmJointSafetyLimit;

/*误差控制器*/
typedef struct
{
    float k1;         // kp 
    float k2;         // kd
    float ref;        // 期望模型关节角, 单位 rad
    float error_last; // 上一周期位置误差, 单位 rad
} RobotArmErrorController;
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP ||ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_COMM_MAP)
/* 通讯控制结构体 */
typedef struct
{
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP)
    Mode_sta mode;         //控制模式
#endif    
    float    arm_ref[8];   //模型角期望值,第八个是夹爪电机速度
} RobotArmComController;
#endif
bool arm_init(void);
void arm_func(void);
void RoboticArm_StartAllMotors(void);
void RoboticArm_StopAllMotors(void);
bool RoboticArm_SetExternalWrench(const RobotArmWrench *wrench);
void RoboticArm_SetExpect(float expect[ROBOT_ARM_MOTOR_NUM]);
void RoboticArm_SetBaseMotion(RobotArmLinkMotion *base_motion);
void remote_ctrl_arm(void);
const RoboticArmJointSafetyLimit *RoboticArm_GetJointLimit(uint8_t index);

#endif // _ARM_FUNC_H_
