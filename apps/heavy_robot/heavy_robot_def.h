#ifndef _HEAVY_ROBOT_DEF_H_
#define _HEAVY_ROBOT_DEF_H_

#include <stdint.h>

/* 机械臂 */
/*  机械臂控制模式 */
#define ROBOTIC_ARM_CTRL_MODE_GRAVITY_COMP 0 /* 纯重力补偿 */
#define ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP   1 /* 遥控器映射 */
#define ROBOTIC_ARM_CTRL_MODE_COMM_MAP     2 /* 通信映射 */
#define ROBOTIC_ARM_CTRL_MODE_IDENTIFY     3 /* 参数辨识 */
//模式选择
#define ROBOTIC_ARM_CTRL_MODE ROBOTIC_ARM_CTRL_MODE_GRAVITY_COMP
/* 纯重力类模式: 模型侧屏蔽速度/加速度动态项, 只算静态重力前馈。
   重力补偿要的就只是重力矩; 参数辨识同样要求前馈部分干净(信息由位置环提供),
   因此共用同一判据。注意这只影响模型算出的 torque_ff, 与位置环无关。 */
#define ROBOTIC_ARM_MODE_IS_GRAVITY_ONLY                                                      \
    ((ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_GRAVITY_COMP) ||                         \
     (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY))
/* 常量的上层定义 */
/*重力加速度*/
#define ROBOTIC_ARM_GRAVITY_ACCELERATION 9.8011f
/* 缓存并开放 ^0T_1~^0T_7 中间关节位姿, 0 表示不缓存 */
#define ROBOT_ARM_ENABLE_BASE_TO_JOINT 0
/* 关节角加速度低通滤波的默认时间常数，单位 s；0 表示不滤波 */
#define ROBOT_ARM_DEFAULT_ACCELERATION_FILTER_TAU_S 0.02f
/* 速度反馈死区 (rad/s): |速度| 小于此值当作 0, 滤除反馈小抖动, 防被加速度估计/阻尼项放大 */
#define ROBOT_ARM_VELOCITY_DEADBAND_RAD_S 0.02f
/* 固定为 7 个旋转关节；数组索引 0~6 分别对应机械关节 J1~J7。 */
#define ROBOT_ARM_KINEMATICS_JOINT_NUM 7U
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
