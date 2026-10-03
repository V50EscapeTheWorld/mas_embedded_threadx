#ifndef _HEAVY_ROBOT_DEF_H_
#define _HEAVY_ROBOT_DEF_H_

#include <stdint.h>

/* 机械臂 */
/*  机械臂控制模式 */
#define ROBOTIC_ARM_CTRL_MODE_GRAVITY_COMP 0 /* 纯重力补偿 */
#define ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP   1 /* 遥控器映射(记得开离线检测) */
#define ROBOTIC_ARM_CTRL_MODE_COMM_MAP     2 /* 通信映射(记得开离线检测) */
#define ROBOTIC_ARM_CTRL_MODE_IDENTIFY     3 /* 参数辨识 */
//模式选择
#define ROBOTIC_ARM_CTRL_MODE ROBOTIC_ARM_CTRL_MODE_COMM_MAP
/* 纯重力类模式: 模型侧屏蔽速度/加速度动态项, 只算静态重力前馈。*/
#define ROBOTIC_ARM_MODE_IS_GRAVITY_ONLY                                                      \
    ((ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_GRAVITY_COMP) ||                         \
     (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY))
/* 常量的上层定义 */
/*重力加速度*/
#define ROBOTIC_ARM_GRAVITY_ACCELERATION 9.8011f
/* 缓存并开放 ^0T_1~^0T_7 中间关节位姿。按惯量调度要读它们 */
#define ROBOT_ARM_ENABLE_BASE_TO_JOINT 1
/* 关节角加速度低通滤波的默认时间常数，单位 s；0 表示不滤波 */
#define ROBOT_ARM_DEFAULT_ACCELERATION_FILTER_TAU_S 0.02f
/* 速度反馈死区 (rad/s): |速度| 小于此值当作 0, 滤除反馈小抖动, 防被加速度估计/阻尼项放大 */
#define ROBOT_ARM_VELOCITY_DEADBAND_RAD_S 0.02f
/* 机械臂遥控死区 */
#define ROBOT_ARM_RC_DEADZONE 120
/* ---- 前馈模式 ----
   0 = 只重力前馈 G(q_meas): 只用当前姿态算静态重力矩。（无法获得期望角度和加速度时，用参考值有延时会震荡，尤其是加速度）
   1 = 参考轨迹驱动的完整逆动力学: 上层每拍调 RobotArm_SetJointReferenceMotion(q̇_d,q̈_d) 传入
       参考运动。*/
#define ROBOT_ARM_FF_MODE 1
/* FF_MODE 1 里"参考速率"一阶低通的时间常数(s)  */
#define ROBOT_ARM_FF_REF_FILTER_TAU_S 0.02f
/* 前馈限幅比例: 前馈最多吃掉该轴力矩上限的这么多, 剩下的留给位置环 */
#define ROBOT_ARM_FF_LIMIT_RATIO 0.80f
/* ---- 按惯量调度 K1(逐轴可开, 表驱动) ----
   控制律: K1_j = min(C_j * J_j_total, K1_nominal_j), 其中 K1_nominal_j 取该轴 K 表里的
   固定 K1(见 arm_func.c)。
   J_j_total = 质量阵 (j,j) 项(由 RobotArm_GetJointInertia 从缓存的 ^0T_i 直接算,
   不再多跑一遍递推) + 该关节的电机转子/减速箱折算惯量。*/
#define ROBOT_ARM_SCHED_ENABLE_J1_TO_J7 \
    {1U, 0U, 0U, 0U, 0U, 0U, 0U}                        /* 逐轴开关, 索引 0~6 = J1~J7 */
#define ROBOT_ARM_SCHED_C_RAD_S \
   {17.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f}  /* K1 = C*J_total, 单位 rad/s */
#define ROBOT_ARM_SCHED_ROTOR_INERTIA_KGM2 \
    {9.252e-3f, 2.074499e-2f, 2.074499e-2f, 1.728120e-3f, 2.887630e-2f, 3.364727e-4f, 1.728120e-3f}
    /* 电机转子+减速箱折算到输出轴的惯量, 上位机读数(MG8016/DM8009/DM4310/DM4340/DM3507)，换电机或换机械臂要重测 */
#define ROBOT_ARM_SCHED_FILTER_TAU_S 0.03f              /* 惯量估计的一阶低通(s), 防增益抖动 */
/* 固定为 7 个旋转关节；数组索引 0~6 分别对应机械关节 J1~J7。 */
#define ROBOT_ARM_KINEMATICS_JOINT_NUM 7U
/* 上电门控: 电机还没回过帧时 measure 全 0, 而 Module_Offline 注册时是在线状态,
   直接使能会把"零偏"当成模型角。等过两个离线超时(100ms×2)再放行。 */
#define ARM_BOOT_GATE_MS 200U
/* 七杆长度(m) */
#define length_1 0.0793f
#define length_2 0.3400f
#define length_3 0.1905f
#define length_4 0.0520f
#define length_5 0.1494f
#define length_6 0.0615f
#define length_7 0.1081f
/* 七杆质量(kg) */
#define weight_1 0.80f
#define weight_2 2.40f
#define weight_3 1.0643f
#define weight_4 0.4542f
#define weight_5 0.4237f
#define weight_6 0.4092f
#define weight_7 0.6096f
/* 第七轴偏移(m) */
#define d_7      0.0385f

#endif /* _HEAVY_ROBOT_DEF_H_ */
