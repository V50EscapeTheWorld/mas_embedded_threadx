#include "arm_ff_ctrl.h"
#include <stdint.h>
#include <string.h>

/*J1~J7 的电机绑定、角度标定、实时关节状态和电机方向扭矩前馈。*/
static RobotArmJoint robot_arm_joint_buf[ROBOT_ARM_MOTOR_NUM];
/*单实例机械臂的 Craig 修改 DH 预计算上下文,保存固定 DH 参数的三角函数缓存和 J7 到工具系的固定变换，由初始化函数建立 */
static RobotArmKinematics robot_arm_kinematics;
/* 最近一次外推得到的基座到各关节坐标系变换 */
#if ROBOT_ARM_ENABLE_BASE_TO_JOINT
static RobotArmTransform robot_arm_base_to_joint[ROBOT_ARM_MOTOR_NUM];
#endif
/* 最近一次外推得到的相邻关节坐标系变换 */
static RobotArmTransform robot_arm_parent_to_joint[ROBOT_ARM_MOTOR_NUM];
/* 最近一次正向递推得到的基座到工具坐标系变换 ^0T_tool */
static RobotArmTransform robot_arm_base_to_tool;
/* J1~J7 连杆最近一次外推得到的速度和加速度.每个元素中的运动向量均表达在对应连杆自身坐标系中。*/
static RobotArmLinkMotion robot_arm_link_motion[ROBOT_ARM_MOTOR_NUM];
/* J1~J7 连杆的固定质量、质心位置和质心惯量张量,初始化时从配置复制，惯量关于质心取矩并表达在对应连杆坐标系中 */
static RobotArmLinkDynamicsParam robot_arm_link_dynamics[ROBOT_ARM_MOTOR_NUM];
/* J1~J7 连杆最近一次计算得到的质心加速度、惯性合力和惯性合力矩，所有向量表达在对应连杆坐标系中 */
static RobotArmLinkInertia robot_arm_link_inertia[ROBOT_ARM_MOTOR_NUM];
/* ~J7 最近一次内推得到的子树载荷和模型关节力矩,子树合力及合力矩表达在对应连杆坐标系中，joint_torque_nm 沿模型 z_i 正方向 */
static RobotArmLinkLoad robot_arm_link_load[ROBOT_ARM_MOTOR_NUM];
/* q_ddot 一阶低通滤波器的当前时间常数，单位 s；0 表示不滤波 */
static float robot_arm_acceleration_filter_tau_s = ROBOT_ARM_DEFAULT_ACCELERATION_FILTER_TAU_S;

/**
 * @brief 判断电机类型是否具有本模块支持的角度和速度反馈。
 */
static bool RobotArm_IsMotorTypeSupported(Motor_Type_e motor_type)
{
    switch (motor_type)
    {
    case M3508:
    case M2006:
    case GM6020_CURRENT:
    case GM6020_VOLTAGE:
    case DM4310:
    case DM4340:
    case DM6220:
    case DM8009:
    case DM3507:
    case DM3519:
    case MG8016:
        return true;

    default:
        return false;
    }
}

/**
 * @brief 清除一个关节的角加速度差分历史。
 *
 * 初始化或角度方向标定发生变化后，旧速度样本不能继续参与差分，
 * 否则下一帧会产生与真实运动无关的 q_ddot 尖峰。
 */
static void RobotArm_ResetAccelerationEstimator(RobotArmJoint *controller)
{
    if (controller == NULL)
    {
        return;
    }

    controller->current_velocity         = 0.0f;
    controller->current_acceleration     = 0.0f;
    controller->last_velocity            = 0.0f;
    controller->acceleration_initialized = false;
}

/**
 * @brief 清空整个模块的运行状态，并恢复与硬件无关的默认值。
 */
static void RobotArm_ResetModuleState(void)
{
    memset(robot_arm_joint_buf, 0, sizeof(robot_arm_joint_buf));
    memset(&robot_arm_kinematics, 0, sizeof(robot_arm_kinematics));
    memset(robot_arm_parent_to_joint, 0, sizeof(robot_arm_parent_to_joint));
    memset(robot_arm_link_motion, 0, sizeof(robot_arm_link_motion));
    memset(robot_arm_link_dynamics, 0, sizeof(robot_arm_link_dynamics));
    memset(robot_arm_link_inertia, 0, sizeof(robot_arm_link_inertia));
    memset(robot_arm_link_load, 0, sizeof(robot_arm_link_load));

    RobotArmTransform_SetIdentity(&robot_arm_base_to_tool);
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
#if ROBOT_ARM_ENABLE_BASE_TO_JOINT
        RobotArmTransform_SetIdentity(&robot_arm_base_to_joint[i]);
#endif
        RobotArmTransform_SetIdentity(&robot_arm_parent_to_joint[i]);
        robot_arm_joint_buf[i].angle_direction = 1.0f;
    }

    robot_arm_acceleration_filter_tau_s = ROBOT_ARM_DEFAULT_ACCELERATION_FILTER_TAU_S;
}

/**
 * @brief 清除逆动力学输出，防止计算失败后继续使用上一周期的旧力矩。
 */
static void RobotArm_ResetInverseDynamicsOutput(void)
{
    memset(robot_arm_link_load, 0, sizeof(robot_arm_link_load));
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        robot_arm_joint_buf[i].torque_ff = 0.0f;
    }
}

/**
 * @brief 读取一个已注册电机的反馈，并转换为模型关节角和角速度。
 *
 * 本函数内部完成（feedback_ratio 为电机反馈→输出轴的换算系数，见
 * RobotArmJointInitConfig 注释，达妙等输出轴反馈电机为 1）：
 *   q     = angle_direction * (motor_angle * feedback_ratio) + angle_offset_rad
 *   q_dot = angle_direction * (motor_velocity * feedback_ratio)
 * 输出先写入调用者提供的局部快照，不直接修改模块状态，保证七轴反馈可以
 * 在全部读取成功后一次性提交，避免读取中途失败造成半帧新旧数据混用。
 *
 * @return 电机指针和类型有效时返回 true，否则返回 false。
 */
static bool RobotArm_ReadMotorFeedback(const RobotArmJoint *controller, float *joint_angle_rad, float *joint_velocity_rad_s)
{
    if ((controller == NULL) || (controller->motor == NULL) || (joint_angle_rad == NULL) || (joint_velocity_rad_s == NULL))
    {
        return false;
    }

    /* 电机结构体均以 Motor_Base 为首字段, 各品牌驱动统一填充 base.measure (多圈角/角速度) */
    const Motor_Base *motor = controller->motor;

    *joint_angle_rad      = controller->angle_direction * ((motor->measure.total_angle * controller->feedback_ratio) + controller->angle_offset_rad);
    *joint_velocity_rad_s = controller->angle_direction * (motor->measure.speed_rad * controller->feedback_ratio);
    return true;    
}

/* ============================== 模块初始化与配置 ============================== */

/**
 * @brief 使用一个配置结构体初始化完整的 7 轴机械臂。
 */
bool RobotArm_Init(const RobotArmInitConfig *config)
{
    if ((config == NULL) || (config->acceleration_filter_tau_s < 0.0f))
    {
        return false;
    }

    // 修改任何模块状态前先验证完整配置，避免初始化失败后留下半套参数。
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        const RobotArmJointInitConfig *joint_config = &config->joint[i];
        if ((joint_config->motor == NULL) || !RobotArm_IsMotorTypeSupported(joint_config->motor_type) ||
            ((joint_config->angle_direction != 1.0f) && (joint_config->angle_direction != -1.0f)) || (config->link_dynamics[i].mass_kg < 0.0f))
        {
            return false;
        }
    }

    RobotArm_ResetModuleState();

    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        robot_arm_joint_buf[i].motor      = config->joint[i].motor;
        robot_arm_joint_buf[i].motor_name = config->joint[i].motor_type;

        if (!RobotArm_SetJointAngleCalibration(i, config->joint[i].angle_direction, config->joint[i].angle_offset_rad))
        {
            RobotArm_ResetModuleState();
            return false;
        }
        /* 反馈→输出轴换算系数，未显式提供(≤0)时兜底为 1(视为输出轴反馈) */
        robot_arm_joint_buf[i].feedback_ratio = (config->joint[i].feedback_ratio > 0.0f) ? config->joint[i].feedback_ratio : 1.0f;
    }

    if (!RobotArm_SetAccelerationFilter(config->acceleration_filter_tau_s) ||
        !RobotArm_KinematicsInit(config->dh_params, config->joint7_to_tool))
    {
        RobotArm_ResetModuleState();
        return false;
    }

    memcpy(robot_arm_link_dynamics, config->link_dynamics, sizeof(robot_arm_link_dynamics));

    return true;
}

/**
 * @brief 设置关节角加速度估计的一阶低通滤波时间常数。
 */
bool RobotArm_SetAccelerationFilter(float time_constant_s)
{
    if (time_constant_s < 0.0f)
    {
        return false;
    }

    robot_arm_acceleration_filter_tau_s = time_constant_s;
    return true;
}

/**
 * @brief 配置一个关节从电机反馈角到模型关节角的标定关系。
 */
bool RobotArm_SetJointAngleCalibration(uint8_t num, float direction, float offset_rad)
{
    if ((num >= ROBOT_ARM_MOTOR_NUM) || ((direction != 1.0f) && (direction != -1.0f)))
    {
        return false;
    }

    robot_arm_joint_buf[num].angle_direction  = direction;
    robot_arm_joint_buf[num].angle_offset_rad = offset_rad;
    RobotArm_ResetAccelerationEstimator(&robot_arm_joint_buf[num]);
    return true;
}

/**
 * @brief 初始化模块内部的 7 轴 Craig 修改 DH 运动学模型。
 */
bool RobotArm_KinematicsInit(const RobotArmMdhParam   dh_params[ROBOT_ARM_MOTOR_NUM],
                                     const RobotArmTransform *joint7_to_tool)
{
    const bool initialized = RobotArmKinematics_Init(&robot_arm_kinematics, dh_params, joint7_to_tool);

    if (initialized)
    {
        // 在第一次实时计算前提供确定的单位矩阵结果，避免暴露未初始化内存。
        RobotArmTransform_SetIdentity(&robot_arm_base_to_tool);
        memset(robot_arm_link_motion, 0, sizeof(robot_arm_link_motion));
        memset(robot_arm_link_inertia, 0, sizeof(robot_arm_link_inertia));
        memset(robot_arm_link_load, 0, sizeof(robot_arm_link_load));
        for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
        {
#if ROBOT_ARM_ENABLE_BASE_TO_JOINT
            RobotArmTransform_SetIdentity(&robot_arm_base_to_joint[i]);
#endif
            RobotArmTransform_SetIdentity(&robot_arm_parent_to_joint[i]);
        }
    }

    return initialized;
}

/* ============================== 步骤 1：关节状态 ============================== */

/**
 * @brief 更新模型关节角、角速度，并估计滤波后的角加速度。
 */
bool RobotArm_UpdateJointState(float dt_s)
{
    float joint_angle_rad[ROBOT_ARM_MOTOR_NUM];
    float joint_velocity_rad_s[ROBOT_ARM_MOTOR_NUM];

    if (dt_s <= 0.0f)
    {
        return false;
    }

    // 先完整读取一帧；若任一关节失败，不提交部分更新。
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        if (!RobotArm_ReadMotorFeedback(&robot_arm_joint_buf[i], &joint_angle_rad[i], &joint_velocity_rad_s[i]))
        {
            return false;
        }
    }

    const float filter_gain =
        (robot_arm_acceleration_filter_tau_s <= 0.0f) ? 1.0f : dt_s / (robot_arm_acceleration_filter_tau_s + dt_s);

    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        RobotArmJoint *controller = &robot_arm_joint_buf[i];

        /* 速度死区: |速度| 低于阈值当作 0, 滤除反馈小抖动 */
        float velocity_rad_s = joint_velocity_rad_s[i];
        if (velocity_rad_s > -ROBOT_ARM_VELOCITY_DEADBAND_RAD_S && velocity_rad_s < ROBOT_ARM_VELOCITY_DEADBAND_RAD_S)
        {
            velocity_rad_s = 0.0f;
        }

        controller->current_angle    = joint_angle_rad[i];
        controller->current_velocity = velocity_rad_s;

        if (!controller->acceleration_initialized)
        {
            controller->current_acceleration     = 0.0f;
            controller->acceleration_initialized = true;
        }
        else
        {
            const float raw_acceleration = (velocity_rad_s - controller->last_velocity) / dt_s;
            controller->current_acceleration += filter_gain * (raw_acceleration - controller->current_acceleration);
        }

        controller->last_velocity = velocity_rad_s;
    }

    return true;
}

/* ========================== 步骤 2~4：递归逆动力学 =========================== */

/**
 * @brief 更新关节状态并正向递推 J1~J7 的速度和加速度。
 */
bool RobotArm_UpdateForwardMotion(float dt_s, const RobotArmLinkMotion *base_motion,
                                          RobotArmTransform *base_to_tool)
{
    float joint_angle_rad[ROBOT_ARM_MOTOR_NUM];
    float joint_velocity_rad_s[ROBOT_ARM_MOTOR_NUM];
    float joint_acceleration_rad_s2[ROBOT_ARM_MOTOR_NUM];

    if (!RobotArm_UpdateJointState(dt_s))
    {
        return false;
    }

    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        joint_angle_rad[i]           = robot_arm_joint_buf[i].current_angle;
#if ROBOTIC_ARM_MODE_IS_GRAVITY_ONLY
        /* 纯重力示教/保持/辨识: 只发静态重力前馈。屏蔽速度/加速度动态项,
           避免无闭环系统上由加速度差分驱动的惯性前馈正反馈自激。 */
        joint_velocity_rad_s[i]      = 0.0f;
        joint_acceleration_rad_s2[i] = 0.0f;
#else
        joint_velocity_rad_s[i]      = robot_arm_joint_buf[i].current_velocity;
        joint_acceleration_rad_s2[i] = robot_arm_joint_buf[i].current_acceleration;
#endif
    }

    if (!RobotArmKinematics_ForwardMotion(&robot_arm_kinematics, joint_angle_rad, joint_velocity_rad_s, joint_acceleration_rad_s2,
                                                  base_motion, base_to_tool,
#if ROBOT_ARM_ENABLE_BASE_TO_JOINT
                                                  robot_arm_base_to_joint,
#else
                                                  NULL,
#endif
                                                  robot_arm_parent_to_joint, robot_arm_link_motion))
    {
        return false;
    }

    if (base_to_tool != NULL)
    {
        robot_arm_base_to_tool = *base_to_tool;
    }

    return RobotArmJointDynamics_ComputeLinkInertia(robot_arm_link_dynamics, robot_arm_link_motion,
                                                       robot_arm_link_inertia);
}

/**
 * @brief 执行完整递归 Newton-Euler 逆动力学并更新电机方向的扭矩前馈。
 */
bool RobotArm_UpdateInverseDynamics(float dt_s, const RobotArmLinkMotion *base_motion,
                                            const RobotArmWrench *external_wrench_tool,
                                            RobotArmTransform *base_to_tool)
{
    if (!RobotArm_UpdateForwardMotion(dt_s, base_motion, base_to_tool) ||
        !RobotArmJointDynamics_Backward(robot_arm_link_dynamics, robot_arm_link_inertia, robot_arm_parent_to_joint,
                                           &robot_arm_kinematics.joint7_to_tool, external_wrench_tool,
                                           robot_arm_link_load))
    {
        RobotArm_ResetInverseDynamicsOutput();
        return false;
    }

    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        // q_dot = direction*motor_velocity，因此功率一致性要求 motor_torque = direction*model_torque。
        robot_arm_joint_buf[i].torque_ff = robot_arm_joint_buf[i].angle_direction * robot_arm_link_load[i].joint_torque_nm;
    }

    return true;
}

/* ============================== 只读结果接口 ================================ */

/**
 * @brief 获取指定关节的电机绑定、模型关节状态和电机方向扭矩前馈。
 */
const RobotArmJoint *RobotArm_GetJoint(uint8_t joint_index)
{
    if (joint_index >= ROBOT_ARM_MOTOR_NUM)
    {
        return NULL;
    }

    return &robot_arm_joint_buf[joint_index];
}

/**
 * @brief 读取关节电机的实时反馈，并换算成模型关节角/角速度（输出轴语义）。
 */
bool RobotArm_GetJointFeedback(uint8_t joint_index, float *angle_rad, float *velocity_rad_s)
{
    if ((joint_index >= ROBOT_ARM_MOTOR_NUM) || (angle_rad == NULL) || (velocity_rad_s == NULL))
    {
        return false;
    }

    return RobotArm_ReadMotorFeedback(&robot_arm_joint_buf[joint_index], angle_rad, velocity_rad_s);
}

/**
 * @brief 获取最近一次请求更新的基座到工具坐标系变换 ^0T_tool。
 * @return 模块内部只读指针；仅非 NULL 位姿输出请求会覆盖其内容。
 */
const RobotArmTransform *RobotArm_GetBaseToTool(void) { return &robot_arm_base_to_tool; }

/**
 * @brief 获取最近一次成功计算的基座到指定关节坐标系变换。
 * @param joint_index 关节数组索引，0 返回 ^0T_1，6 返回 ^0T_7。
 * @return 索引有效时返回模块内部只读指针，否则返回 NULL。
 */
#if ROBOT_ARM_ENABLE_BASE_TO_JOINT
const RobotArmTransform *RobotArm_GetBaseToJoint(uint8_t joint_index)
{
    if (joint_index >= ROBOT_ARM_MOTOR_NUM)
    {
        return NULL;
    }

    return &robot_arm_base_to_joint[joint_index];
}
#endif

/**
 * @brief 获取指定关节最近一次递推得到的连杆运动状态。
 */
const RobotArmLinkMotion *RobotArm_GetLinkMotion(uint8_t joint_index)
{
    if (joint_index >= ROBOT_ARM_MOTOR_NUM)
    {
        return NULL;
    }

    return &robot_arm_link_motion[joint_index];
}

/**
 * @brief 获取指定连杆最近一次正向计算得到的质心惯性结果。
 */
const RobotArmLinkInertia *RobotArm_GetLinkInertia(uint8_t joint_index)
{
    if (joint_index >= ROBOT_ARM_MOTOR_NUM)
    {
        return NULL;
    }

    return &robot_arm_link_inertia[joint_index];
}

/**
 * @brief 获取指定连杆最近一次内推得到的子树载荷和模型关节力矩。
 */
const RobotArmLinkLoad *RobotArm_GetLinkLoad(uint8_t joint_index)
{
    if (joint_index >= ROBOT_ARM_MOTOR_NUM)
    {
        return NULL;
    }

    return &robot_arm_link_load[joint_index];
}
