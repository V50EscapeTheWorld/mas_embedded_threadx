#include "arm_func.h"
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
#include "arm_identify.h"
#endif
#include "motor_lk_def.h"
#include "arm_ff_ctrl.h"
#include "tx_api.h"
#include "bsp_dwt.h"
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP)
#include "module_remote.h"
#endif
#define LOG_TAG "app_arm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 控制模式名: 与 heavy_robot_def.h 中 ROBOTIC_ARM_CTRL_MODE_* 取值 0/1/2/3 对应 */
static const char *const robotic_arm_ctrl_mode_name[] = {"gravity_comp", "remote_map", "comm_map", "identify"};
/* J1~J7 电机对象指针。 */
Motor_Base *robot_arm_motors[ROBOT_ARM_MOTOR_NUM];
/* 关节安全限制 */
static const RoboticArmJointSafetyLimit robotic_arm_joint_limit[ROBOT_ARM_MOTOR_NUM] = {
    {-2*PI, 2*PI, 1.44f, 37.0f}, {-1.9f, 1.8f, 0.98f, 35.0f}, {-3.2f, 3.0f, 0.98f, 35.0f}, {-2*PI, 2*PI, 10.0f, 12.0f},
    {-2.1f, 2.7f, 3.6f, 30.0f}, {-2*PI, 2*PI, 10.0f, 3.0f}, {-2.0f, 2.0f, 10.0f, 12.0f},
};
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_GRAVITY_COMP)
/* 关节阻尼 (Nm per rad/s), 索引 J1~J7 */
static const float robotic_arm_gravity_damping[ROBOT_ARM_MOTOR_NUM] = {
    0.3f, 0.0f, 0.0f, 0.65f, 0.0f, 0.2f, 0.0f,
};
#endif
/* 辨识使用的没有补偿阻尼的参数，因此和有前馈时参数不一致 */
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
/* 误差控制器 */
RobotArmErrorController robotic_arm_error_controller[ROBOT_ARM_MOTOR_NUM]=
{
    {.k1 = 13.0f, .k2 = 1000.0f,.ref = 0.0f, .error_last = 0.0f},
    {.k1 = 35.0f, .k2 = 800.0f, .ref = 0.0f, .error_last = 0.0f},
    {.k1 = 35.0f, .k2 = 600.0f, .ref = 0.0f, .error_last = 0.0f},
    {.k1 = 2.0f,  .k2 = 200.0f, .ref = 0.0f, .error_last = 0.0f},
    {.k1 = 6.0f,  .k2 = 850.0f, .ref = 0.0f, .error_last = 0.0f},
    {.k1 = 0.5f,  .k2 = 90.0f,  .ref = 0.0f, .error_last = 0.0f},
    {.k1 = 1.0f,  .k2 = 350.0f, .ref = 0.0f, .error_last = 0.0f},
};
#else
/* lqr */
static LQRInstance LqrForArm[ROBOT_ARM_MOTOR_NUM];
/* 按惯量调度 K1 用的逐轴表 */
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP) || (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_COMM_MAP)
static const uint8_t robotic_arm_sched_enable[ROBOT_ARM_MOTOR_NUM] = ROBOT_ARM_SCHED_ENABLE_J1_TO_J7;
static const float   robotic_arm_sched_c[ROBOT_ARM_MOTOR_NUM]      = ROBOT_ARM_SCHED_C_RAD_S;
static const float   robotic_arm_sched_rotor[ROBOT_ARM_MOTOR_NUM]  = ROBOT_ARM_SCHED_ROTOR_INERTIA_KGM2;
static float robotic_arm_sched_k1_cap[ROBOT_ARM_MOTOR_NUM];
static float robotic_arm_sched_j_filt[ROBOT_ARM_MOTOR_NUM];
static bool  robotic_arm_sched_logged[ROBOT_ARM_MOTOR_NUM];
#endif
#endif

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
/* 安全状态枚举 */
static Safity_sta safe_sta=SAFE_INIT;
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP ||ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_COMM_MAP)
/* 通讯用结构体 */
static RobotArmComController arm_com;
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP)
/* 遥控器实例 */
static Remote_Data_t *remote_data;
#endif
#endif
/* 机械臂初始化 */
bool arm_init(void)
{
/* 初始化机械臂电机以及lqr */
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
#if (ROBOTIC_ARM_CTRL_MODE != ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
    LQR_Init_Config_s LqrForArm_init ={
        .state_dim =2
    };
#endif
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
#if (ROBOTIC_ARM_CTRL_MODE != ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
    LqrForArm_init.K[0] = 87.18f;
    LqrForArm_init.K[1] = 5.22f;
    LQRInit(&LqrForArm[0],&LqrForArm_init);
#endif
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
#if (ROBOTIC_ARM_CTRL_MODE != ROBOTIC_ARM_CTRL_MODE_IDENTIFY) 
    LqrForArm_init.K[0] = 87.50f;
    LqrForArm_init.K[1] = 4.02f;
    LQRInit(&LqrForArm[1],&LqrForArm_init); 
#endif
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
#if (ROBOTIC_ARM_CTRL_MODE != ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
    LqrForArm_init.K[0] = 87.50f;
    LqrForArm_init.K[1] = 4.93;
    LQRInit(&LqrForArm[2],&LqrForArm_init);
#endif
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
#if (ROBOTIC_ARM_CTRL_MODE != ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
    LqrForArm_init.K[0] = 7.10f;
    LqrForArm_init.K[1] = 0.565;
    LQRInit(&LqrForArm[3],&LqrForArm_init);
#endif
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
#if (ROBOTIC_ARM_CTRL_MODE != ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
    LqrForArm_init.K[0] = 75.00f;
    LqrForArm_init.K[1] = 1.380f;
    LQRInit(&LqrForArm[4],&LqrForArm_init);
#endif
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
#if (ROBOTIC_ARM_CTRL_MODE != ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
    LqrForArm_init.K[0] = 2.69f;
    LqrForArm_init.K[1] = 0.0486;
    LQRInit(&LqrForArm[5],&LqrForArm_init);
#endif
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
#if (ROBOTIC_ARM_CTRL_MODE != ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
    LqrForArm_init.K[0] = 6.86f;
    LqrForArm_init.K[1] = 0.0675;
    LQRInit(&LqrForArm[6],&LqrForArm_init);
#endif

#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP) || (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_COMM_MAP)
    /* 记下各轴固定的 K1 作为调度封顶值(之后 LqrForArm[i].K[1] 每拍可能被改写) */
    for (uint8_t i = 0; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        robotic_arm_sched_k1_cap[i] = LqrForArm[i].K[1];
        robotic_arm_sched_j_filt[i] = 0.0f;
    }
#endif

    for (uint8_t i = 0; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        if (robot_arm_motors[i] == NULL)
        {
            LOG_E("J%u motor init failed", i + 1U);
            return false;
        }
    }

/*机械臂动力学模型参数(当前为慢速近匀速映射，忽略惯量的影响),机械结构变化不要忘了更改，尤其是j1*/
    RobotArmInitConfig robot_arm_init_config = {
        .acceleration_filter_tau_s = ROBOT_ARM_DEFAULT_ACCELERATION_FILTER_TAU_S,
        .joint7_to_tool            = NULL,
        .joint                     = {{.motor = robot_arm_motors[0], .motor_type = MG8016, .angle_direction = -1.0f, .angle_offset_rad = -4.0206f,  .feedback_ratio = 1.0f},         /* MG8016: 状态2 编码器为输出轴端, 速度已由驱动÷减速比换算到输出轴端, 此处不再换算 */
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

/*-----------------------------------------------------控制部分---------------------------------------------------------------*//* 控制模式名: 与 heavy_robot_def.h 中 ROBOTIC_ARM_CTRL_MODE_* 取值 0/1/2/3 对应 */
/** 检查机器人是否安全: 任一电机未注册/离线/越界即不安全
 * @param fault_joint 非空时,会写入第一个不安全关节索引(供故障日志用),返回0硬件故障，返回2运行故障。
 */
static uint8_t RoboticArm_CheckSafety(uint8_t *fault_joint)
{
    uint8_t result = 1; /* 1: 正常, 2: 越限/超速(软故障), 0: 硬件/反馈故障 */

    for (uint8_t i = 0; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        const RobotArmJoint *joint = RobotArm_GetJoint(i);
        const Motor_Base    *motor = (joint != NULL) ? joint->motor : NULL;

        /* 各电机结构体均直接按 Motor_Base 读取; 硬件类故障优先级最高, 立即返回 */
        if (motor == NULL || Module_Offline_get_device_status(motor->offline_dev) == STATE_OFFLINE)
        {
            if (fault_joint != NULL)
            {
                *fault_joint = i;
            }
            return 0;
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
            return 0;
        }
        const RoboticArmJointSafetyLimit *limit = &robotic_arm_joint_limit[i];
        if (angle_rad < limit->min_angle_rad || angle_rad > limit->max_angle_rad ||
            velocity_rad_s > limit->max_velocity_rad_s || velocity_rad_s < -limit->max_velocity_rad_s)
        {
            /* 越限/超速是软故障: 先记录并继续扫描, 后面的硬件故障会被 return 0 覆盖 */
            result = 2;
            if (fault_joint != NULL)
            {
                *fault_joint = i;
            }
        }
    }

    return result;
}
/* 限位保持 */
static uint8_t RoboticArm_KeepUnsafeJoint(uint8_t fault_joint)
{
    if(fault_joint >= ROBOT_ARM_MOTOR_NUM)
        return 0;
    float angle_rad;
    const RobotArmJoint *joint              = RobotArm_GetJoint(fault_joint);
    const RoboticArmJointSafetyLimit *limit = &robotic_arm_joint_limit[fault_joint];
    #if ROBOTIC_ARM_CTRL_MODE==ROBOTIC_ARM_CTRL_MODE_IDENTIFY
    RobotArmErrorController *controller     = &robotic_arm_error_controller[fault_joint];
    #else
    LQRInstance *controller = &LqrForArm[fault_joint];
    #endif

    if(!joint||!limit||!controller)
        return 0;

    if(joint->current_angle>limit->max_angle_rad)
    {
        angle_rad = joint->current_angle - 0.1;
    }
    else if(joint->current_angle<limit->min_angle_rad)
    {
        angle_rad = joint->current_angle + 0.1; 
    }
    else
    {
        angle_rad = joint->current_angle;
    
    }

    controller->ref = angle_rad;
    #if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP ||ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_COMM_MAP)
    arm_com.arm_ref[fault_joint] = angle_rad;
    #endif

    return 1;
}
/* 只在进入故障那一帧调用: 直接报哪一根轴不安全 */
static void RoboticArm_LogUnsafeJoint(uint8_t fault_joint)
{
    if (fault_joint < ROBOT_ARM_MOTOR_NUM)
    {
        LOG_W("arm error: J%u unsafe", fault_joint + 1U);
    }
}

/* 停止所有电机 */
void RoboticArm_StopAllMotors(void)
{
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        Motor_Base *motor = robot_arm_motors[i];
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
        Motor_Base *motor = robot_arm_motors[i];
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
        Motor_Base *motor = robot_arm_motors[i];
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
#if (ROBOT_ARM_FF_MODE == 1) && \
    ((ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP) || (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_COMM_MAP))
    /* 参考驱动前馈: 把"参考速率"喂给前馈模块(位置仍用实测 q 求值)。由于慢速控制，速度影响极小，开不开效果差不多，
       加速度可视为0，遥控映射本身是速率指令, 所以参考速率由【参考】差分得到 */
    {
        static float ref_last[ROBOT_ARM_MOTOR_NUM];
        static float ref_vel[ROBOT_ARM_MOTOR_NUM];
        static bool  ref_last_valid = false;
        const float  alpha = (dt_s > 0.0f) ? (dt_s / (ROBOT_ARM_FF_REF_FILTER_TAU_S + dt_s)) : 0.0f;
        float        ref_acc[ROBOT_ARM_MOTOR_NUM];

        if (!ref_last_valid)
        {
            for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
            {
                ref_last[i] = LqrForArm[i].ref;
                ref_vel[i]  = 0.0f;
            }
            ref_last_valid = true;
        }

        for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
        {
            const float vmax = robotic_arm_joint_limit[i].max_velocity_rad_s;
            float       raw  = (dt_s > 0.0f) ? ((LqrForArm[i].ref - ref_last[i]) / dt_s) : 0.0f;

            /* 换模式/锁位那一拍 ref 会阶跃, 限一下幅, 免得给出一次性的冲击 */
            if (raw > vmax)
            {
                raw = vmax;
            }
            else if (raw < -vmax)
            {
                raw = -vmax;
            }

            ref_last[i] = LqrForArm[i].ref;
            ref_vel[i] += alpha * (raw - ref_vel[i]);   /* 一阶低通: ref 是台阶式累加 */
            ref_acc[i]  = 0.0f;                         /* 遥控映射没有加速度指令 */
        }
        RobotArm_SetJointReferenceMotion(ref_vel, ref_acc);
    }
#endif
    /*更新递推 (更新 q/q_dot/q_ddot, 正向递推 + 内推得到 torque_ff, 含重力) */
    if(!RobotArm_UpdateInverseDynamics(dt_s, &robotic_arm_base_motion,robotic_arm_external_wrench_tool_ptr,NULL ))
        return false;

#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP) || (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_COMM_MAP)
    /* 按惯量调度 K1: 对每个开了开关的轴做 K1 = min(C*J_total, 固定 K)。*/
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        float j_total;
        float alpha;
        float k1_sched;

        if (robotic_arm_sched_enable[i] == 0U)
        {
            continue;
        }

        j_total = RobotArm_GetJointInertia(i) + robotic_arm_sched_rotor[i];
        if (j_total <= 0.0f)
        {
            continue;
        }

        alpha = dt_s / (ROBOT_ARM_SCHED_FILTER_TAU_S + dt_s);
        robotic_arm_sched_j_filt[i] = (robotic_arm_sched_j_filt[i] > 0.0f)
                                          ? (robotic_arm_sched_j_filt[i] + alpha * (j_total - robotic_arm_sched_j_filt[i]))
                                          : j_total;

        k1_sched =  robotic_arm_sched_c[i] * robotic_arm_sched_j_filt[i];
        if (k1_sched > robotic_arm_sched_k1_cap[i])
        {
            k1_sched = robotic_arm_sched_k1_cap[i];
        }
        LqrForArm[i].K[1] = k1_sched;

        if (!robotic_arm_sched_logged[i])
        {
            robotic_arm_sched_logged[i] = true;
            LOG_I("sched J%u: J=%.5f (model %.5f + rotor %.5f), K1=%.4f (C=%.2f, cap=%.4f)",
                  (unsigned)(i + 1U), robotic_arm_sched_j_filt[i],
                  robotic_arm_sched_j_filt[i] - robotic_arm_sched_rotor[i],
                  (double)robotic_arm_sched_rotor[i], k1_sched,
                  (double)robotic_arm_sched_c[i], robotic_arm_sched_k1_cap[i]);
        }
    }
#endif

    float expect_torque_nm[ROBOT_ARM_MOTOR_NUM];

    for (uint8_t i = 0; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        const RobotArmJoint *joint = RobotArm_GetJoint(i);

        /* 公共项: 重力前馈 —— 重力补偿的本体, 所有模式都有 */
        float torque = joint->torque_ff;
        const float ff_limit = ROBOT_ARM_FF_LIMIT_RATIO * robotic_arm_joint_limit[i].max_torque_nm;
        if (torque > ff_limit)
        {
            torque = ff_limit;
        }
        else if (torque < -ff_limit)
        {
            torque = -ff_limit;
        }

#if ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_GRAVITY_COMP
        /* 关节阻尼: 物理项, 与角速度反向, 再按 angle_direction 换算到电机方向，辨识模式不加 */
        torque -= joint->angle_direction * (robotic_arm_gravity_damping[i] * joint->current_velocity);
#endif
        /* 位置环: 叠加在"前馈 + 阻尼"之上。
           REMOTE/COMM: 跟踪外部下发的期望角;
           IDENTIFY:    跟踪内部扫描轨迹 */
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP) || \
    (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_COMM_MAP)
        /* 传入控制律的速度先按该轴速度限值夹一下。*/
        float qd_ctrl = joint->current_velocity;
        const float qd_lim = robotic_arm_joint_limit[i].max_velocity_rad_s;
        if (qd_ctrl > qd_lim)
        {
            qd_ctrl = qd_lim;
        }
        else if (qd_ctrl < -qd_lim)
        {
            qd_ctrl = -qd_lim;
        }
        torque += joint->angle_direction * LQRCalculate(&LqrForArm[i], joint->current_angle, qd_ctrl, LqrForArm[i].ref);
#elif (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY)
        RobotArmErrorController *controller = &robotic_arm_error_controller[i];
        const float error = controller->ref - joint->current_angle;
        torque += joint->angle_direction * (error * controller->k1 + controller->k2 * (error - controller->error_last));
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
    #if ROBOTIC_ARM_CTRL_MODE==ROBOTIC_ARM_CTRL_MODE_IDENTIFY
    RobotArmErrorController *controller     = robotic_arm_error_controller;
    #else
    LQRInstance *controller = LqrForArm;  
    #endif

    if (expect == NULL)
    {
        return;
    }
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
        controller[i].ref = expect[i];
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

/* 以当前模型角锁位: 令首帧误差≈0, 避免猛冲 */
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
        #if ROBOTIC_ARM_CTRL_MODE==ROBOTIC_ARM_CTRL_MODE_IDENTIFY
        RobotArmErrorController *controller     = &robotic_arm_error_controller[i];
        #else
        LQRInstance *controller = &LqrForArm[i];  
        #endif
        controller->ref     = angle_rad;
        #if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP ||ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_COMM_MAP)
        arm_com.arm_ref[i]    = angle_rad;
        #endif
        #if ROBOTIC_ARM_CTRL_MODE==ROBOTIC_ARM_CTRL_MODE_IDENTIFY
        controller->error_last = 0.0f;
        #endif       
    }
}
/* 控制逻辑 */
void arm_func(void)
{
    static uint32_t cnt_last    = 0;
    static uint32_t boot_ms     = 0;
    float           dt_s        = 0;
    uint8_t         check_result= 1;
    uint8_t         fault_joint = ROBOT_ARM_KINEMATICS_JOINT_NUM;
    Safity_sta      sta_last    = safe_sta;

    /* 上电门控: 电机还没回过帧时 measure 全 0, 而 Module_Offline 注册时会先报"在线",
       此时使能会把零偏当成模型角, 算出错误的期望角/前馈(实测表现为上电猛冲一下)。 */
    if (boot_ms == 0U)
    {
        boot_ms = (uint32_t)BSP_DWT_GetTimeline_ms();
    }
    if ((uint32_t)BSP_DWT_GetTimeline_ms() - boot_ms < ARM_BOOT_GATE_MS)
    {
        RoboticArm_StopAllMotors();
        (void)BSP_DWT_GetDeltaT(&cnt_last); /* 刷新时间戳, 放行后首帧 dt 才是真实的 2ms */
        return;
    }
/* 获取安全情况 */
    check_result=RoboticArm_CheckSafety(&fault_joint);
/* 实现机械臂控制 */
    dt_s = BSP_DWT_GetDeltaT(&cnt_last);

    /* UNSAFE_STOP(遥控掉线等硬件/链路故障) 优先级最高, 不被越限判定覆盖 */
    if (safe_sta == UNSAFE_STOP)
    {
        RoboticArm_StopAllMotors();
    }
    else if(check_result!=1)
    {
        switch (check_result) 
        {
            case 0:
                safe_sta = UNSAFE_STOP;
                break;
            case 2:
                safe_sta = UNSAFE_KEEP;
                break;          
            default:   
                safe_sta = UNSAFE_STOP;
                break;
        }
    }
    else if(safe_sta == SAFE_INIT)
    {
        /* 使能前锁位: 目标=当前位置, 首帧误差≈0, 只做前馈保持 */
        RoboticArm_LockAtCurrentPosition();
        RoboticArm_StartAllMotors();
        if (!RoboticArm_Control(dt_s))
        {
            safe_sta = UNSAFE_STOP;
        }
        else
        {
            safe_sta = SAFE_RUN;
        }
    }
    else if(safe_sta == SAFE_RUN)
    {
        /* 运行中控制失败(逆动力学/写扭矩异常): 停机并复位使能 */
        if (!RoboticArm_Control(dt_s))
        {
            safe_sta = UNSAFE_STOP;
        }
    }
    else if(safe_sta == UNSAFE_KEEP)
    {
            safe_sta = SAFE_INIT;
    }

    if(safe_sta == UNSAFE_KEEP)
    {
        RoboticArm_StartAllMotors();
        /* 纯重力补偿下只等待回归 */
        #if ROBOTIC_ARM_CTRL_MODE!=ROBOTIC_ARM_CTRL_MODE_GRAVITY_COMP
        if(!RoboticArm_KeepUnsafeJoint(fault_joint))
            safe_sta = UNSAFE_STOP;
        #endif
        if(!RoboticArm_Control(dt_s))
            safe_sta = UNSAFE_STOP;
    }
    if(safe_sta == UNSAFE_STOP)
    {
        RoboticArm_StopAllMotors();
    }

    /* 日志只在状态变化那一帧打 */
    if (safe_sta != sta_last)
    {
        switch (safe_sta)
        {
            case UNSAFE_KEEP: 
                RoboticArm_LogUnsafeJoint(fault_joint); 
                break;
            case UNSAFE_STOP:
                LOG_E("arm stop, joint:%u", fault_joint + 1U);
                break;
            default: break;
        }
    }
        
}
/*-----------------------------------------------------通讯部分---------------------------------------------------------------*/
#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_REMOTE_MAP)

/** @brief 摇杆死区: 绝对值小于 ROBOT_ARM_RC_DEADZONE 时返回 0 */
static int16_t RoboticArm_RcDeadzone(int16_t channel)
{
    return ((channel > -ROBOT_ARM_RC_DEADZONE) && (channel < ROBOT_ARM_RC_DEADZONE)) ? 0 : channel;
}

/* 遥控器映射 */
void remote_ctrl_arm(void)
{
    static uint8_t keepflag  = 0;
    static uint8_t unsafe_flag = 0;
    static uint8_t remotback_time = 0;

    remote_data = Module_Remote_get_data();
    if(!remote_data)
    {
        safe_sta = UNSAFE_STOP;
        return;
    }
        
    int16_t mode_channel = remote_data->channels[4];
    int16_t lx_channel   = RoboticArm_RcDeadzone(remote_data->channels[2]);
    int16_t ly_channel   = RoboticArm_RcDeadzone(remote_data->channels[3]);
    int16_t rx_channel   = RoboticArm_RcDeadzone(remote_data->channels[1]);
    int16_t ry_channel   = RoboticArm_RcDeadzone(remote_data->channels[0]);
    
    if((Module_Remote_get_offline_status()&0x01) == 0)
    {
        safe_sta = UNSAFE_STOP;
        RoboticArm_StopAllMotors();
        unsafe_flag = 1;
        remotback_time = 0;
    }
    else 
    {         
        if(unsafe_flag)
        {
            if(remotback_time < 100)
                remotback_time++;
            if(remotback_time >= 100)
            {
                safe_sta=SAFE_INIT;
                unsafe_flag = 0;
            }
        }
            
        if(safe_sta == SAFE_RUN)
        {
            //控制模式设置
            switch(mode_channel)
            {
                case SBUS_CHX_UP:
                    arm_com.mode = ARM_KEEP;
                    break;
                case SBUS_CHX_BIAS:
                    keepflag = 0;
                    arm_com.mode = LOW_FOUR;
                    break;
                case SBUS_CHX_DOWN:
                    keepflag = 0;
                    arm_com.mode = HIGH_FOUR;
                    break;
                default:
                    keepflag = 0;
                    arm_com.mode = ARM_KEEP;
                    break;
            }
            //控制输出值
            switch(arm_com.mode)
            {
                case ARM_KEEP:
                    if(keepflag == 0)
                    {
                        RoboticArm_LockAtCurrentPosition();
                        keepflag = 1;
                    }
                    break;
                case LOW_FOUR:
                    arm_com.arm_ref[0]+=0.0000009*ly_channel;//推满约每秒1度
                    arm_com.arm_ref[1]-=0.0000022*lx_channel;//推满约每秒5度
                    arm_com.arm_ref[2]-=0.0000022*rx_channel;
                    arm_com.arm_ref[3]+=0.0000022*ry_channel;
                    break;
                case HIGH_FOUR:
                    arm_com.arm_ref[4]-=0.0000022*lx_channel;
                    arm_com.arm_ref[5]+=0.0000022*ly_channel;
                    arm_com.arm_ref[6]-=0.0000022*rx_channel;
                    arm_com.arm_ref[7] =GRIPPER_MOTOR_SPEEDE_MAP*ry_channel;
                    break;
                default:                    
                    break;                    
            }   
            RoboticArm_SetExpect(arm_com.arm_ref);        
        }
    }
}
#endif

