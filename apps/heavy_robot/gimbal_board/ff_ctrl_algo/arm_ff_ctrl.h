#ifndef __ARM_FF_CTRL_H__
#define __ARM_FF_CTRL_H__

#include "motor_def.h"
#include "heavy_robot_def.h"
#include <stdbool.h>
#include "motor_damiao.h"
#include "motor_dji.h"
#include "motor_lk.h"
#include "arm_ff_ctrl_fun.h"

/* 电机槽位数与运动学关节数保持一致，索引 0~6 对应 J1~J7 */
#define ROBOT_ARM_MOTOR_NUM ROBOT_ARM_KINEMATICS_JOINT_NUM // 机械臂电机数量
/* 是否缓存并开放 ^0T_1~^0T_7 中间关节位姿 */
#ifndef ROBOT_ARM_ENABLE_BASE_TO_JOINT
#define ROBOT_ARM_ENABLE_BASE_TO_JOINT 0
#endif
/* 速度反馈死区 (rad/s): |速度| 小于此值当作 0, 滤除反馈小抖动, 防被加速度估计/阻尼项放大 */
#ifndef ROBOT_ARM_VELOCITY_DEADBAND_RAD_S
#define ROBOT_ARM_VELOCITY_DEADBAND_RAD_S 0.02f
#endif
/* 关节角加速度低通滤波的默认时间常数，单位 s；0 表示不滤波 */
#ifndef ROBOT_ARM_DEFAULT_ACCELERATION_FILTER_TAU_S
#define ROBOT_ARM_DEFAULT_ACCELERATION_FILTER_TAU_S 0.02f
#endif

/**
 * @brief 单个机械臂关节的电机绑定和角度标定数据。
 *
 * 不同品牌电机通过 motor_name 解释 motor 指针。current_angle 始终保存
 * 已转换到机器人模型定义的关节角，正运动学不直接读取电机原始值。
 */
typedef struct
{
    Motor_Base  *motor;                   // 对应电机对象；实际类型由 motor_name 决定，未注册时为 NULL
    Motor_Type_e motor_name;              // 电机类型，用于选择 MOTOR_DJI、MOTOR_DM 或 MOTOR_LK 的反馈结构
    float        torque_ff;               // 电机输出轴正方向的扭矩前馈，单位 Nm；当前仅保存，不直接发送
    float        current_angle;           // 当前模型关节角 q_i，已经过方向和零偏校正，单位 rad
    float        current_velocity;        // 当前模型关节角速度 q_dot_i，已经过方向校正，单位 rad/s
    float        current_acceleration;    // 低通滤波后的模型关节角加速度 q_ddot_i，单位 rad/s^2
    float        last_velocity;           // 上一次速度采样，用于计算 q_ddot_i，单位 rad/s
    float        angle_direction;         // 电机角到模型角的方向系数，只允许 +1.0f 或 -1.0f
    float        angle_offset_rad;        // 电机反馈总角度为 0 时的模型关节角，单位 rad
    float        feedback_ratio;          // 电机反馈 → 输出轴的换算系数，见 RobotArmJointInitConfig 注释
    bool         acceleration_initialized;// 已获得第一次速度样本；第一次更新不进行差分
} RobotArmJoint;

/**
 * @brief 单个关节的初始化配置。
 *
 * 数组索引决定关节编号，不再依赖调用顺序：joint[0] 对应 J1，
 * joint[6] 对应 J7。
 */
typedef struct
{
    Motor_Base  *motor;            // 电机对象指针，实际类型必须与 motor_type 一致
    Motor_Type_e motor_type;       // 电机类型，用于选择对应的反馈数据结构
    float        angle_direction;  // 电机反馈方向到模型正方向的映射，只允许 +1.0f 或 -1.0f
    float        angle_offset_rad; // 电机反馈总角度为 0 时对应的模型关节角，单位 rad
    float        feedback_ratio;   // 电机反馈→输出轴换算系数：模型角 = 方向×(反馈×ratio)+零偏。
                                   // 电机端反馈(如 LK/DJI 减速电机)填 1/减速比；输出轴反馈(达妙)填 1.0f
} RobotArmJointInitConfig;

/**
 * @brief 7 轴机械臂的完整初始化配置。
 *
 * 建议定义为 static const，填完参数后只调用一次 RobotArm_Init()。
 * joint7_to_tool 传 NULL 时，工具坐标系与 J7 坐标系重合。
 */
typedef struct
{
    RobotArmJointInitConfig   joint[ROBOT_ARM_MOTOR_NUM];         // J1~J7 电机和角度标定
    RobotArmMdhParam          dh_params[ROBOT_ARM_MOTOR_NUM];     // J1~J7 Craig 修改 DH 参数
    RobotArmLinkDynamicsParam link_dynamics[ROBOT_ARM_MOTOR_NUM]; // J1~J7 连杆质量、质心和质心惯量
    const RobotArmTransform  *joint7_to_tool;                     // 固定工具变换 ^7T_tool，可为 NULL
    float                     acceleration_filter_tau_s;          // q_ddot 一阶低通滤波时间常数，单位 s；0 表示不滤波
} RobotArmInitConfig;

/* ============================== 模块初始化与配置 ============================== */

/**
 * @brief 使用完整配置初始化机械臂模块。
 *
 * 本函数统一完成：模块状态清零、J1~J7 电机绑定、角度方向和零偏标定、
 * 加速度估计器初始化、连杆质量属性复制、Craig 修改 DH 预计算以及工具变换初始化。
 * 可再次调用本函数重新初始化整个模块。
 *
 * @param config 完整初始化配置，生命周期只需覆盖本次调用。
 * @return 所有电机、方向、滤波参数、质量及运动学参数有效时返回 true；否则返回 false。
 */
bool RobotArm_Init(const RobotArmInitConfig *config);

/**
 * @brief 设置 q_ddot 一阶低通滤波时间常数。
 * @param time_constant_s 时间常数，单位 s；0 表示不滤波，不得小于 0。
 * @return 参数有效时返回 true，否则返回 false。
 * @note 默认值为 0.02 s。时间常数越大，加速度越平滑但延迟越大。
 */
bool RobotArm_SetAccelerationFilter(float time_constant_s);

/**
 * @brief 设置电机角度到模型关节角的标定关系（方向与零偏）。
 *
 * 完整换算为 模型关节角 = direction × (电机反馈总角度 × feedback_ratio) + offset_rad，
 * 本函数仅设置 direction 与 offset_rad，feedback_ratio 在 RobotArm_Init 中配置。
 * DH 的 theta_offset 只描述坐标系定义，不应拿它代替编码器零位标定。
 * @param num 关节索引，J1 对应 0，J7 对应 6。
 * @param direction 电机反馈到模型正方向的映射，只接受 +1.0f 或 -1.0f。
 * @param offset_rad 电机反馈总角度为 0 时的模型角，单位 rad。
 * @return 索引和方向有效时返回 true，否则返回 false。
 */
bool RobotArm_SetJointAngleCalibration(uint8_t num, float direction, float offset_rad);

/**
 * @brief 配置 7 轴 Craig 修改 DH 参数和可选工具变换。
 * @param dh_params 7 组 DH 参数，索引 0~6 依次对应 J1~J7。
 * @param joint7_to_tool 固定变换 ^7T_tool；传 NULL 表示工具系与 J7 重合。
 * @return 参数有效并成功建立运动学缓存时返回 true。
 */
bool RobotArm_KinematicsInit(const RobotArmMdhParam   dh_params[ROBOT_ARM_MOTOR_NUM],
                                     const RobotArmTransform *joint7_to_tool);

/* ============================== 步骤 1：关节状态 ============================== */

/**
 * @brief 更新 q、q_dot，并通过速度差分和一阶低通滤波估计 q_ddot。
 *
 * q_dot 直接读取各电机减速后的输出轴速度，q_ddot 使用：
 * raw_acceleration = (q_dot[k] - q_dot[k-1]) / dt。
 * 第一次调用只记录速度并把 q_ddot 置 0，避免启动冲击。
 *
 * @param dt_s 本次与上次调用之间的实际时间，单位 s，必须大于 0。
 * @return dt 有效且 7 个电机均已注册、类型受支持时返回 true。
 */
bool RobotArm_UpdateJointState(float dt_s);

/* ========================== 步骤 2~4：递归逆动力学 =========================== */

/**
 * @brief 更新 q、q_dot、q_ddot，并正向递推所有连杆运动和质心惯性结果。
 *
 * 本函数是后续逆动力学周期应调用的入口。它使用同一份关节状态快照，
 * 更新 J1~J7 的连杆运动状态、质心加速度、惯性合力和关于质心的惯性
 * 合力矩。仅在请求位姿输出时维护累计的 ^0T_tool。
 *
 * @param dt_s 关节状态更新周期，单位 s，必须大于 0。
 * @param base_motion 基座运动状态；固定基座传 NULL。重力补偿时可传入
 *                    origin_linear_acceleration_m_s2 = -g 的基座状态。
 * @param base_to_tool 可选输出 ^0T_tool；传 NULL 时跳过累计末端位姿计算。
 * @return 关节状态读取和正向递推都成功时返回 true，否则返回 false。
 */
bool RobotArm_UpdateForwardMotion(float dt_s, const RobotArmLinkMotion *base_motion,
                                          RobotArmTransform *base_to_tool);

/**
 * @brief 执行完整递归 Newton-Euler 逆动力学并更新七个关节扭矩前馈。
 *
 * 本函数先更新 q、q_dot、q_ddot 并完成 J1 到 J7 的外推，再由 J7 到 J1
 * 内推子树载荷。内推得到的模型关节力矩会按 angle_direction 转换到电机
 * 输出轴正方向，并写入模块内部扭矩前馈缓存；本函数不发送电机指令。
 *
 * external_wrench_tool 表示环境作用在机械臂上的工具端外力，表达在工具
 * 坐标系中。函数直接将它作为 Newton-Euler 内推的末端边界条件，不单独
 * 构造雅可比矩阵。固定基座仅计算运动惯性时 base_motion 传 NULL；重力
 * 补偿时传入原点线加速度为 -g 的基座运动状态。
 *
 * @param dt_s 关节状态更新周期，单位 s，必须大于 0。
 * @param base_motion 基座运动状态；传 NULL 表示静止且不计重力。
 * @param external_wrench_tool 环境施加到机械臂的工具端六维力；传 NULL 表示没有外部接触力。
 * @param base_to_tool 可选输出 ^0T_tool；传 NULL 时跳过累计末端位姿计算。
 * @return 外推、连杆惯性计算和内推均成功时返回 true，否则返回 false。
 */
bool RobotArm_UpdateInverseDynamics(float dt_s, const RobotArmLinkMotion *base_motion,
                                            const RobotArmWrench *external_wrench_tool,
                                            RobotArmTransform *base_to_tool);

/* ============================== 只读结果接口 ================================ */

/**
 * @brief 读取关节电机的实时反馈，并换算成模型关节角/角速度（输出轴语义）。
 *
 * 换算关系: 模型角 = direction × (电机反馈 × feedback_ratio) + offset；
 * 模型角速度 = direction × (电机反馈速度 × feedback_ratio)。与
 * RobotArm_UpdateJointState 内部使用的换算完全一致，但不依赖 current_angle
 * 缓存，可于使能/安全检查等模型尚未更新时调用。
 * @param joint_index 关节索引，0~6 对应 J1~J7。
 * @param angle_rad 输出模型关节角 q，单位 rad。
 * @param velocity_rad_s 输出模型关节角速度 q_dot，单位 rad/s。
 * @return 索引有效、电机已注册且反馈可用时返回 true。
 */
bool RobotArm_GetJointFeedback(uint8_t joint_index, float *angle_rad, float *velocity_rad_s);

/**
 * @brief 获取指定关节的电机绑定、模型关节状态和电机方向扭矩前馈。
 * @param joint_index 关节索引，0~6 对应 J1~J7。
 * @return 有效索引返回内部只读指针，否则返回 NULL；调用者不得修改或释放。
 */
const RobotArmJoint *RobotArm_GetJoint(uint8_t joint_index);

/**
 * @brief 获取最近一次请求更新的基座到工具坐标系变换。
 * @return 模块内部只读指针；仅非 NULL 位姿输出请求会覆盖其内容。
 */
const RobotArmTransform *RobotArm_GetBaseToTool(void);

/**
 * @brief 获取最近一次计算得到的基座到指定关节坐标系变换。
 * @param joint_index 关节索引，J1 对应 0，J7 对应 6。
 * @return 有效索引返回模块内部只读指针，否则返回 NULL；调用者不得修改或释放。
 */
#if ROBOT_ARM_ENABLE_BASE_TO_JOINT
const RobotArmTransform *RobotArm_GetBaseToJoint(uint8_t joint_index);
#endif

/**
 * @brief 获取指定关节最近一次正向递推得到的连杆运动状态。
 * @param joint_index 关节索引，0~6 对应 J1~J7。
 * @return 索引有效时返回内部只读指针，否则返回 NULL。
 */
const RobotArmLinkMotion *RobotArm_GetLinkMotion(uint8_t joint_index);

/**
 * @brief 获取指定连杆最近一次计算得到的质心惯性结果。
 * @param joint_index 连杆索引，0~6 对应 J1~J7 所连接的七个连杆。
 * @return 有效索引返回内部只读指针，否则返回 NULL；下一次正向运动更新会覆盖其内容。
 */
const RobotArmLinkInertia *RobotArm_GetLinkInertia(uint8_t joint_index);

/**
 * @brief 获取指定连杆最近一次内推得到的子树载荷和模型关节力矩。
 * @param joint_index 连杆索引，0~6 对应 J1~J7。
 * @return 有效索引返回内部只读指针，否则返回 NULL；下一次逆动力学更新会覆盖其内容。
 */
const RobotArmLinkLoad *RobotArm_GetLinkLoad(uint8_t joint_index);

#endif
