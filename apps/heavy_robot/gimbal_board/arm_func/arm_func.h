/*
 * @Description: 云台功能模板 (云台板)
 *
 * 与单板版本的接口相同
 */

#ifndef _ARM_FUNC_H_
#define _ARM_FUNC_H_

#include "heavy_robot_def.h" 
#include "arm_ff_ctrl.h"

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
    float kp;         ///< 位置误差增益
    float kd;         ///< 误差变化率增益(近似速度阻尼)
    float expect;     ///< 期望模型关节角, 单位 rad
    float error_last; ///< 上一周期位置误差, 单位 rad
} RobotArmErrorController;

extern RobotArmWrench *robotic_arm_external_wrench_tool_ptr;
/* J1~J7 电机对象指针, 定义在 arm_func.c; 辨识模块需读取各电机实际下发的扭矩 */
extern Motor_Base *robot_arm_motors[ROBOT_ARM_MOTOR_NUM];

bool arm_init(void);
void arm_func(void);
void RoboticArm_StartAllMotors(void);
void RoboticArm_StopAllMotors(void);
bool RoboticArm_SetExternalWrench(const RobotArmWrench *wrench);
void RoboticArm_SetExpect(float expect[ROBOT_ARM_MOTOR_NUM]);
void RoboticArm_SetBaseMotion(RobotArmLinkMotion *base_motion);
/**
 * @brief 获取指定关节的模型角安全限位 (角度/速度/扭矩上限)。
 * @param index 关节索引, 0~6 对应 J1~J7。
 * @return 索引有效时返回内部只读指针, 否则返回 NULL。
 */
const RoboticArmJointSafetyLimit *RoboticArm_GetJointLimit(uint8_t index);

#endif // _ARM_FUNC_H_
