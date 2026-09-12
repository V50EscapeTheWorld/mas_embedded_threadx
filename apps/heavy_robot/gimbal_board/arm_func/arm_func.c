#include "arm_func.h"
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
#include "arm_identify.h"
#endif
#include "motor_lk_def.h"
#include "arm_ff_ctrl.h"
#include "tx_api.h"
#include "bsp_dwt.h"

#define LOG_TAG "app_arm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 控制模式名: 与 heavy_robot_def.h 中 ROBOTIC_ARM_CTRL_MODE_* 取值 0/1/2/3 对应 */
static const char *const robotic_arm_ctrl_mode_name[] = {"gravity_comp", "remote_map", "comm_map", "identify"};
/* J1~J7 电机对象指针。 */
Motor_Base *robot_arm_motors[ROBOT_ARM_MOTOR_NUM];
/* 关节安全限制 */
static const RoboticArmJointSafetyLimit robotic_arm_joint_limit[ROBOT_ARM_MOTOR_NUM] = {
    {-2*PI, 2*PI, 10.0f, 37.0f}, {-1.9f, 1.8f, 8.0f, 35.0f}, {-3.2f, 3.0f, 8.0f, 35.0f}, {-2*PI, 2*PI, 10.0f, 12.0f},
    {-2.1f, 2.7f, 3.6f, 30.0f}, {-2*PI, 2*PI, 10.0f, 3.0f}, {-2.0f, 2.0f, 10.0f, 12.0f},
};
#if ROBOTIC_ARM_CTRL_MODE!=ROBOTIC_ARM_CTRL_MODE_IDENTIFY
/* 重力补偿模式的关节阻尼 (Nm per rad/s), 索引 J1~J7 */
static const float robotic_arm_gravity_damping[ROBOT_ARM_MOTOR_NUM] = {
    0.3f, 0, 0, 0.65f, 0, 0.2f, 0,
};
#endif
/* 误差控制器 */
RobotArmErrorController robotic_arm_error_controller[ROBOT_ARM_MOTOR_NUM]=
{
    {.kp = 13.0f, .kd = 1000.0f,.expect = 0.0f, .error_last = 0.0f},
    {.kp = 35.0f, .kd = 800.0f, .expect = 0.0f, .error_last = 0.0f},
    {.kp = 35.0f, .kd = 600.0f, .expect = 0.0f, .error_last = 0.0f},
    {.kp = 2.0f,  .kd = 200.0f, .expect = 0.0f, .error_last = 0.0f},
    {.kp = 6.0f,  .kd = 850.0f, .expect = 0.0f, .error_last = 0.0f},
    {.kp = 0.5f,  .kd = 90.0f,  .expect = 0.0f, .error_last = 0.0f},
    {.kp = 1.0f,  .kd = 350.0f, .expect = 0.0f, .error_last = 0.0f},
};
/**
 * Newton-Euler 递推使用的固定基座运动状态。
 * 机械臂基座默认静止，重力补偿通过传入虚拟基座加速度 -g 实现；
 * 所有向量均表达在基座坐标系中，单位分别为 rad/s^2、rad/s、m/s 和 m/s^2。
 */
RobotArmLinkMotion robotic_arm_base_motion = {
    .angular_acceleration_rad_s2     = {0.0f, 0.0f, 0.0f},
    .angular_velocity_rad_s          = {0.0f, 0.0f, 0.0f},
    .origin_linear_velocity_m_s      = {0.0f, 0.0f, 0.0f},
    .origin_linear_acceleration_m_s2 = {0.0f, 0.0f, ROBOTIC_ARM_GRAVITY_ACCELERATION},
};
/* 当前工具坐标系下的有效末端外力；由应用接口更新，控制任务读取。 */
RobotArmWrench robotic_arm_external_wrench_tool;
RobotArmWrench* robotic_arm_external_wrench_tool_ptr = NULL;
/* 机械臂初始化 */
bool arm_init(void)
{
/* 初始化机械臂电机 */
    Motor_Init_Config_s motor_init_config = 
    {
        .setting_init_config=
        {
            .loop_type = OPEN_LOOP,
            .enableflag = 1,
        },
        .offline_init_config=
        {
            .enable = 1,
            .timeout_ms = 100,
        },
        .transport = MOTOR_TRANSPORT_CAN,
    };
    //J1
    motor_init_config.motor_init_info.motor_type = MG8016;
    motor_init_config.motor_init_info.gear_ratio = 6;
    motor_init_config.motor_init_info.torque_constant = 0.24; 
    motor_init_config.motor_init_info.max_torque = 37;
    motor_init_config.offline_init_config.name = "J1";
    motor_init_config.offline_init_config.beep_times=1;
    motor_init_config.transport_config.can.hcan = BSP_CAN_HANDLE1;
    motor_init_config.transport_config.can.tx_id = 1+LK_CAN_CMD_BASE_ID;
    motor_init_config.transport_config.can.rx_id = 1+LK_CAN_REPLY_BASE_ID;
    robot_arm_motors[0] = (Motor_Base *)Motor_LK_Init(&motor_init_config,LK_CMD_TORQUE_CLOSED_LOOP);
    //J2
    motor_init_config.motor_init_info.motor_type = DM8009;
    motor_init_config.motor_init_info.gear_ratio = 9;
    motor_init_config.motor_init_info.torque_constant = 1;
    motor_init_config.motor_init_info.max_torque = 40;
    motor_init_config.offline_init_config.name = "J2";
    motor_init_config.offline_init_config.beep_times=2;
    motor_init_config.transport_config.can.hcan = BSP_CAN_HANDLE1;
    motor_init_config.transport_config.can.tx_id = 0x02;
    motor_init_config.transport_config.can.rx_id = 0x12;
    robot_arm_motors[1] = (Motor_Base *)Motor_DM_Init(&motor_init_config,DM_MIT_MODE);
    //J3
    motor_init_config.motor_init_info.motor_type = DM8009;
    motor_init_config.motor_init_info.gear_ratio = 9;
    motor_init_config.motor_init_info.torque_constant = 1;
    motor_init_config.motor_init_info.max_torque = 40;
    motor_init_config.offline_init_config.name = "J3";
    motor_init_config.offline_init_config.beep_times=3;
    motor_init_config.transport_config.can.hcan = BSP_CAN_HANDLE1;
    motor_init_config.transport_config.can.tx_id = 0x03;
    motor_init_config.transport_config.can.rx_id = 0x13;
    robot_arm_motors[2] = (Motor_Base *)Motor_DM_Init(&motor_init_config,DM_MIT_MODE);
    //J4
    motor_init_config.motor_init_info.motor_type = DM4310;
    motor_init_config.motor_init_info.gear_ratio = 10;
    motor_init_config.motor_init_info.torque_constant = 0.756;
    motor_init_config.motor_init_info.max_torque = 10;
    motor_init_config.offline_init_config.name = "J4";
    motor_init_config.offline_init_config.beep_times=4;
    motor_init_config.transport_config.can.hcan = BSP_CAN_HANDLE1;
    motor_init_config.transport_config.can.tx_id = 0x04;
    motor_init_config.transport_config.can.rx_id = 0x14;
    robot_arm_motors[3] = (Motor_Base *)Motor_DM_Init(&motor_init_config,DM_MIT_MODE);
    //J5
    motor_init_config.motor_init_info.motor_type = DM4340;
    motor_init_config.motor_init_info.gear_ratio = 40;
    motor_init_config.motor_init_info.torque_constant = 2.5;
    motor_init_config.motor_init_info.max_torque = 28;
    motor_init_config.offline_init_config.name = "J5";
    motor_init_config.offline_init_config.beep_times=5;
    motor_init_config.transport_config.can.hcan = BSP_CAN_HANDLE2;
    motor_init_config.transport_config.can.tx_id = 0x05;
    motor_init_config.transport_config.can.rx_id = 0x15;
    robot_arm_motors[4] = (Motor_Base *)Motor_DM_Init(&motor_init_config,DM_MIT_MODE);
    //J6
    motor_init_config.motor_init_info.motor_type = DM3507;
    motor_init_config.motor_init_info.gear_ratio = 7;
    motor_init_config.motor_init_info.torque_constant = 0.667;
    motor_init_config.motor_init_info.max_torque = 3;
    motor_init_config.offline_init_config.name = "J6";
    motor_init_config.offline_init_config.beep_times=6;
    motor_init_config.transport_config.can.hcan = BSP_CAN_HANDLE2;
    motor_init_config.transport_config.can.tx_id = 0x06;
    motor_init_config.transport_config.can.rx_id = 0x16;
    robot_arm_motors[5] = (Motor_Base *)Motor_DM_Init(&motor_init_config,DM_MIT_MODE);
    //J7
    motor_init_config.motor_init_info.motor_type = DM4310;
    motor_init_config.motor_init_info.gear_ratio = 10;
    motor_init_config.motor_init_info.torque_constant = 0.756;
    motor_init_config.motor_init_info.max_torque = 10;
    motor_init_config.offline_init_config.name = "J7";
    motor_init_config.offline_init_config.beep_times=7;
    motor_init_config.transport_config.can.hcan = BSP_CAN_HANDLE2;
    motor_init_config.transport_config.can.tx_id = 0x07;
    motor_init_config.transport_config.can.rx_id = 0x17;
    robot_arm_motors[6] = (Motor_Base *)Motor_DM_Init(&motor_init_config,DM_MIT_MODE);

    for (uint8_t i = 0; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        if (robot_arm_motors[i] == NULL)
        {
            LOG_E("J%u motor init failed", i + 1U);
            return false;
        }
    }

/*机械臂动力学模型参数*/
    RobotArmInitConfig robot_arm_init_config = {
        .acceleration_filter_tau_s = ROBOT_ARM_DEFAULT_ACCELERATION_FILTER_TAU_S,
        .joint7_to_tool            = NULL,
        .joint                     = {{.motor = robot_arm_motors[0], .motor_type = MG8016, .angle_direction = -1.0f, .angle_offset_rad = -5.6224f,  .feedback_ratio = 1.0f},         /* MG8016 双编码器: 状态2 回传角度/速度已是输出轴端, 不除减速比 */
                                      {.motor = robot_arm_motors[1], .motor_type = DM8009, .angle_direction = 1.0f,  .angle_offset_rad = -0.0250f,  .feedback_ratio = 1.0f},         /* 达妙反馈为输出轴端 */
                                      {.motor = robot_arm_motors[2], .motor_type = DM8009, .angle_direction = -1.0f, .angle_offset_rad = -0.1057f,  .feedback_ratio = 1.0f},
                                      {.motor = robot_arm_motors[3], .motor_type = DM4310, .angle_direction = 1.0f,  .angle_offset_rad =  0.1097f,  .feedback_ratio = 1.0f},
                                      {.motor = robot_arm_motors[4], .motor_type = DM4340, .angle_direction = -1.0f, .angle_offset_rad = -1.0810f,  .feedback_ratio = 1.0f},
                                      {.motor = robot_arm_motors[5], .motor_type = DM3507, .angle_direction = 1.0f,  .angle_offset_rad = -1.2182f,  .feedback_ratio = 1.0f},
                                      {.motor = robot_arm_motors[6], .motor_type = DM4310, .angle_direction = -1.0f, .angle_offset_rad = -1.2461f,  .feedback_ratio = 1.0f}},
        .link_dynamics             = {{.mass_kg           = weight_1,
                                       .center_of_mass_m  = {0.0f, 0.0f, 0.07f},
                                       .inertia_com_kg_m2 = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}},
                                      {.mass_kg           = weight_2,
                                       .center_of_mass_m  = { 0.2233f,  0.0332f,  0.0000f},
                                       .inertia_com_kg_m2 = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}},
                                      {.mass_kg           = weight_3,
                                       .center_of_mass_m  = {-0.1658f, -0.0996f,  0.0000f},
                                       .inertia_com_kg_m2 = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}},
                                      {.mass_kg           = weight_4,
                                       .center_of_mass_m  = { 0.1035f, -0.0132f,  0.0688f},
                                       .inertia_com_kg_m2 = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}},
                                      {.mass_kg           = weight_5,
                                       .center_of_mass_m  = {-0.1210f, -0.1422f, -0.0142f},
                                       .inertia_com_kg_m2 = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}},
                                      {.mass_kg           = weight_6,
                                       .center_of_mass_m  = {-0.0008f, -0.0015f,  0.0789f},
                                       .inertia_com_kg_m2 = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}},
                                      {.mass_kg           = weight_7,
                                       .center_of_mass_m  = {-0.0176f, -0.0234f, -0.0010f},
                                       .inertia_com_kg_m2 = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}}},
        .dh_params                 = {{.a_prev_m = 0.0f,     .alpha_prev_rad = 0.0f,   .d_m = 0.0f,     .theta_offset_rad = 0.0f},
                                      {.a_prev_m = 0.0f,     .alpha_prev_rad = PI/2,   .d_m = 0.0f,     .theta_offset_rad = PI/2},
                                      {.a_prev_m = length_2, .alpha_prev_rad = PI,     .d_m = 0.0f,     .theta_offset_rad = PI/2},
                                      {.a_prev_m = 0.0f,     .alpha_prev_rad = PI/2,   .d_m = length_3, .theta_offset_rad = 0.0f},
                                      {.a_prev_m = 0.0f,     .alpha_prev_rad = -PI/2,  .d_m = 0.0f,     .theta_offset_rad = 0.0f},
                                      {.a_prev_m = 0.0f,     .alpha_prev_rad = PI/2,   .d_m = length_5, .theta_offset_rad = 0.0f},
                                      {.a_prev_m = 0.0f,     .alpha_prev_rad = -PI/2,  .d_m = d_7,     .theta_offset_rad = 0.0f}}};
    if (!RobotArm_Init(&robot_arm_init_config))
    {
        LOG_E("robot arm model init failed");
        return false;
    }

    LOG_I("robot arm init ok, ctrl mode=%d (%s)", ROBOTIC_ARM_CTRL_MODE,
          robotic_arm_ctrl_mode_name[ROBOTIC_ARM_CTRL_MODE]);
    return true;
}


/** 检查机器人是否安全: 任一电机未注册/离线/越界即不安全
 * @param fault_joint 非空时, 返回 false 会写入第一个不安全关节索引(供故障日志用)
 */
static bool RoboticArm_CheckSafety(uint8_t *fault_joint)
{
    for (uint8_t i = 0; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        const RobotArmJoint *joint = RobotArm_GetJoint(i);
        if (joint == NULL)
        {
            if (fault_joint != NULL)
            {
                *fault_joint = i;
            }
            return false;
        }

        /* 各电机结构体均直接按 Motor_Base 读取 */
        const Motor_Base *motor = joint->motor;
        if (motor == NULL || Module_Offline_get_device_status(motor->offline_dev) == STATE_OFFLINE)
        {
            if (fault_joint != NULL)
            {
                *fault_joint = i;
            }
            return false;
        }

        /* 限位按“模型关节角”判断: 读取实时反馈并换算到输出轴模型量(含方向/零偏/减速比) */
        float angle_rad;
        float velocity_rad_s;
        if (!RobotArm_GetJointFeedback(i, &angle_rad, &velocity_rad_s))
        {
            if (fault_joint != NULL)
            {
                *fault_joint = i;
            }
            return false;
        }
        const RoboticArmJointSafetyLimit *limit = &robotic_arm_joint_limit[i];
        if (angle_rad < limit->min_angle_rad || angle_rad > limit->max_angle_rad ||
            velocity_rad_s > limit->max_velocity_rad_s || velocity_rad_s < -limit->max_velocity_rad_s)
        {
            if (fault_joint != NULL)
            {
                *fault_joint = i;
            }
            return false;
        }
    }
    return true;
}

/* 只在进入故障那一帧调用: 直接报哪一根轴不安全 */
static void RoboticArm_LogUnsafeJoint(uint8_t fault_joint)
{
    if (fault_joint < ROBOT_ARM_MOTOR_NUM)
    {
        LOG_W("arm stopped: J%u unsafe", fault_joint + 1U);
    }
}

/* 停止所有电机 */
void RoboticArm_StopAllMotors(void)
{
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        Motor_Base *motor = (Motor_Base *)robot_arm_motors[i];
        if (motor == NULL)
        {
            continue;
        }

        /* 先清零输出再停用 (DM MIT / LK 力矩闭环均从 output_torque 下发) */
        Motor_SetOutputTorque(motor, 0.0f);
        Motor_Stop(motor);
    }
}

/* 启动所有电机 */
void RoboticArm_StartAllMotors(void)
{
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        Motor_Base *motor = (Motor_Base *)robot_arm_motors[i];
        if (motor == NULL)
        {
            continue;
        }

        Motor_Start(motor);
    }
}

/* 写入所有电机的扭矩 (DM MIT / LK 力矩闭环均从 output_torque 下发) */
static bool RoboticArm_WriteTorque(float desired_torque_nm[ROBOT_ARM_MOTOR_NUM])
{
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        Motor_Base *motor = (Motor_Base *)robot_arm_motors[i];
        if (motor == NULL)
        {
            return false;
        }

        float torque_nm = desired_torque_nm[i];
        const float max_torque_nm = robotic_arm_joint_limit[i].max_torque_nm;

        if (torque_nm > max_torque_nm)
        {
            torque_nm = max_torque_nm;
        }
        else if (torque_nm < -max_torque_nm)
        {
            torque_nm = -max_torque_nm;
        }

        Motor_SetOutputTorque(motor, torque_nm);
    }
    return true;
}

/* 控制机械臂 */
static bool RoboticArm_Control(float dt_s)
{
    /*更新递推 (更新 q/q_dot/q_ddot, 正向递推 + 内推得到 torque_ff, 含重力) */
    if(!RobotArm_UpdateInverseDynamics(dt_s, &robotic_arm_base_motion,robotic_arm_external_wrench_tool_ptr,NULL ))
        return false;

    float expect_torque_nm[ROBOT_ARM_MOTOR_NUM];

    for (uint8_t i = 0; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        const RobotArmJoint *joint = RobotArm_GetJoint(i);

        /* 公共项: 重力前馈 —— 重力补偿的本体, 所有模式都有 */
        float torque = joint->torque_ff;

#if ROBOTIC_ARM_CTRL_MODE != ROBOTIC_ARM_CTRL_MODE_IDENTIFY
        /* 关节阻尼: 物理项, 与角速度反向, 再按 angle_direction 换算到电机方向，辨识模式不加 */
        torque -= joint->angle_direction * (robotic_arm_gravity_damping[i] * joint->current_velocity);
#endif

#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP) || \
    (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_COMM_MAP) || \
    (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
        /* 位置环: 叠加在"前馈 + 阻尼"之上。
           REMOTE/COMM: 跟踪外部下发的期望角;
           IDENTIFY:    跟踪内部扫描轨迹 */
        RobotArmErrorController *controller = &robotic_arm_error_controller[i];
        const float error = controller->expect - joint->current_angle;
        torque += joint->angle_direction * (error * controller->kp + controller->kd * (error - controller->error_last));
        controller->error_last = error;
#endif

        expect_torque_nm[i] = torque;
    }
    /*写入扭矩*/
    if(!RoboticArm_WriteTorque(expect_torque_nm))
        return false;
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
    /* 辨识模式: 推进扫描轨迹(写入下一帧期望角), 并在机械臂停稳时输出一组样本 */
    ArmIdentify_Update();
#endif
    return true;
}

/* 设置机械臂外部力 */
bool RoboticArm_SetExternalWrench(const RobotArmWrench *wrench)
{
    if (wrench == NULL)
    {
        return false;
    }

    UINT interrupt_posture = tx_interrupt_control(TX_INT_DISABLE);
    robotic_arm_external_wrench_tool     = *wrench;
    robotic_arm_external_wrench_tool_ptr = &robotic_arm_external_wrench_tool;
    tx_interrupt_control(interrupt_posture);
    return true;
}

/* 设置机械臂期望值 */
void RoboticArm_SetExpect(float expect[ROBOT_ARM_MOTOR_NUM])
{
    if (expect == NULL)
    {
        return;
    }
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
        robotic_arm_error_controller[i].expect = expect[i];
}

/* 设置机械臂基座运动 */
void RoboticArm_SetBaseMotion(RobotArmLinkMotion *base_motion)
{
    if(!base_motion)
        return;
    robotic_arm_base_motion = *base_motion;
}

/* 获取指定关节的模型角安全限位 */
const RoboticArmJointSafetyLimit *RoboticArm_GetJointLimit(uint8_t index)
{
    return (index < ROBOT_ARM_MOTOR_NUM) ? &robotic_arm_joint_limit[index] : NULL;
}

/* 使能瞬间以当前模型角锁位: 令首帧误差≈0, 避免开机向零位猛冲 */
static void RoboticArm_LockAtCurrentPosition(void)
{
    for (uint8_t i = 0; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        const RobotArmJoint *joint = RobotArm_GetJoint(i);
        if (joint == NULL || joint->motor == NULL)
            continue;

        /* 读取实时反馈并换算到输出轴模型关节角 (与模型读取同一换算逻辑) */
        float angle_rad;
        float velocity_rad_s;
        if (!RobotArm_GetJointFeedback(i, &angle_rad, &velocity_rad_s))
            continue;
        RobotArmErrorController *controller = &robotic_arm_error_controller[i];
        controller->expect     = angle_rad;
        controller->error_last = 0.0f;
    }
}

void arm_func(void)
{
    static uint8_t  safety   = 0;
    static uint32_t cnt_last = 0;
    float           dt_s     = 0;
/* 实现机械臂控制 */
    dt_s = BSP_DWT_GetDeltaT(&cnt_last);
    uint8_t fault_joint = 0;
    if(!RoboticArm_CheckSafety(&fault_joint))
    {
        if (safety == 1)
        {
            RoboticArm_LogUnsafeJoint(fault_joint);
        }
        safety = 0;
        RoboticArm_StopAllMotors();
    }
    else if(safety == 0)
    {
        /* 使能前锁位: 目标=当前位置, 首帧误差≈0, 只做前馈保持 */
        RoboticArm_LockAtCurrentPosition();
        RoboticArm_StartAllMotors();
        if (!RoboticArm_Control(dt_s))
        {
            LOG_W("arm stopped: control failed on enable");
            RoboticArm_StopAllMotors();
        }
        else
        {
            safety = 1;
        }
    }
    else
    {
        /* 运行中控制失败(逆动力学/写扭矩异常): 停机并复位使能 */
        if (!RoboticArm_Control(dt_s))
        {
            LOG_W("arm stopped: control failed");
            safety = 0;
            RoboticArm_StopAllMotors();
        }
    }
}
