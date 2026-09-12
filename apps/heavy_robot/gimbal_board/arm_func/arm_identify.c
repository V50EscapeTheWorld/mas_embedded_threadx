#include "arm_identify.h"

#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY)

#include "arm_func.h"
#include "arm_ff_ctrl.h"
#include <math.h>

#define LOG_TAG "arm_identify"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* Halton 低差异序列使用的前 7 个素数, 让各轴的目标点组合尽量分散 */
static const uint32_t identify_halton_base[ROBOT_ARM_MOTOR_NUM] = {2U, 3U, 5U, 7U, 11U, 13U, 17U};

/* 轨迹状态 (仅本文件使用) */
static float    identify_q_cmd[ROBOT_ARM_MOTOR_NUM]; /* 当前期望角 */
static float    identify_lo[ROBOT_ARM_MOTOR_NUM];    /* 扫描区间下限 */
static float    identify_span[ROBOT_ARM_MOTOR_NUM];  /* 扫描区间宽度 */
static uint8_t  identify_inited  = 0U;
static uint32_t identify_wp      = 0U; /* 当前目标点序号 */
static uint32_t identify_hold    = 0U; /* 当前目标点已停稳的累计帧数 (抖动不清零, 见下方说明) */
static uint32_t identify_wait    = 0U; /* 当前目标点已等待的帧数 (超时用) */
static uint32_t identify_divider = 0U;

/** van der Corput 基函数, 返回 [0,1); Halton 序列的第 base 维分量 */
static float Identify_Vdc(uint32_t index, uint32_t base)
{
    float result = 0.0f;
    float weight = 1.0f;

    while (index > 0U)
    {
        weight /= (float)base;
        result += weight * (float)(index % base);
        index /= base;
    }
    return result;
}

/** 第 wp 个目标点上第 i 轴的目标角 */
static float Identify_Target(uint32_t wp, uint8_t i)
{
    return identify_lo[i] + Identify_Vdc(wp + 1U, identify_halton_base[i]) * identify_span[i];
}

/** 首次调用: 以当前姿态为中心构造扫描区间, 并先停在原位 */
static void Identify_Init(void)
{
    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        const RoboticArmJointSafetyLimit *limit = RoboticArm_GetJointLimit(i);
        float                             q     = 0.0f;
        float                             qd    = 0.0f;

        (void)RobotArm_GetJointFeedback(i, &q, &qd);

        float lo = q - ARM_IDENTIFY_SWEEP_RAD;
        float hi = q + ARM_IDENTIFY_SWEEP_RAD;

        if (limit != NULL)
        {
            /* 留出余量, 避免正好踩在限位上触发安全检查 */
            const float margin = 0.05f;
            const float lim_lo = limit->min_angle_rad + margin;
            const float lim_hi = limit->max_angle_rad - margin;
            if (lo < lim_lo)
            {
                lo = lim_lo;
            }
            if (hi > lim_hi)
            {
                hi = lim_hi;
            }
        }

        if (hi <= lo)
        {
            /* 区间退化(已经贴着限位): 该轴就地不动 */
            lo = q;
            hi = q;
        }

        identify_lo[i]    = lo;
        identify_span[i]  = hi - lo;
        identify_q_cmd[i] = q;
    }

    RoboticArm_SetExpect(identify_q_cmd);
    identify_inited = 1U;
}

bool ArmIdentify_Update(void)
{
    float q[ROBOT_ARM_MOTOR_NUM];
    float qd[ROBOT_ARM_MOTOR_NUM];
    float tau[ROBOT_ARM_MOTOR_NUM];
    float tau_ff[ROBOT_ARM_MOTOR_NUM];

    if (identify_inited == 0U)
    {
        Identify_Init();
        return false;
    }

    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        if (robot_arm_motors[i] == NULL || !RobotArm_GetJointFeedback(i, &q[i], &qd[i]))
        {
            return false;
        }

        /* 电机真正拿到的扭矩: WriteTorque 限幅后写入 output_torque, 再由此下发 */
        tau[i] = robot_arm_motors[i]->controller.output_torque;
        /* 限幅前的模型重力前馈, 供上位机校验 DH 约定 */
        tau_ff[i] = RobotArm_GetJoint(i)->torque_ff;
    }

    /* ---------- 推进扫描轨迹 ---------- */
    if (identify_wp >= ARM_IDENTIFY_WAYPOINTS)
    {
        return false;
    }

    bool settled = true;

    for (uint8_t i = 0U; i < ROBOT_ARM_MOTOR_NUM; ++i)
    {
        const float target = Identify_Target(identify_wp, i);
        const float diff   = target - identify_q_cmd[i];

        if (diff > ARM_IDENTIFY_SLEW_RAD)
        {
            identify_q_cmd[i] += ARM_IDENTIFY_SLEW_RAD;
        }
        else if (diff < -ARM_IDENTIFY_SLEW_RAD)
        {
            identify_q_cmd[i] -= ARM_IDENTIFY_SLEW_RAD;
        }
        else
        {
            identify_q_cmd[i] = target;
        }

        /* 停稳只看两件事: 设定点已经停住, 机械臂本身静止。
           不要求 q 落在设定点上 —— 位置环有稳态偏差(模型误差/kp),
           机械臂停在偏离处同样是静止平衡, T == tau_g_real(q) 照样成立。 */
        if (fabsf(target - identify_q_cmd[i]) > ARM_IDENTIFY_ARRIVE_RAD ||
            fabsf(qd[i]) > ARM_IDENTIFY_QD_OK_RAD_S)
        {
            settled = false;
        }
    }

    RoboticArm_SetExpect(identify_q_cmd);

    identify_wait++;
    if (settled)
    {
        /* 累加而非"连续计数": 机械臂在阈值附近轻微抖动时, 连续计数会被反复
           清零, 一个点也攒不满, 全靠超时跳过 —— 那样采到的仍是不静止的样本。 */
        identify_hold++;
    }

    if (identify_hold >= ARM_IDENTIFY_HOLD_TICKS || identify_wait >= ARM_IDENTIFY_TIMEOUT_TICKS)
    {
        identify_wp++;
        identify_hold = 0U;
        identify_wait = 0U;
    }

    /* 只有本帧确实静止才输出样本 */
    if (!settled)
    {
        return false;
    }

    if (++identify_divider < ARM_IDENTIFY_DECIMATION)
    {
        return false;
    }
    identify_divider = 0U;

    LOG_RAW("Q,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f" ULOG_NEWLINE_SIGN,
            q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
    LOG_RAW("D,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f" ULOG_NEWLINE_SIGN,
            qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], qd[6]);
    LOG_RAW("T,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f" ULOG_NEWLINE_SIGN,
            tau[0], tau[1], tau[2], tau[3], tau[4], tau[5], tau[6]);
    LOG_RAW("G,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f" ULOG_NEWLINE_SIGN,
            tau_ff[0], tau_ff[1], tau_ff[2], tau_ff[3], tau_ff[4], tau_ff[5], tau_ff[6]);

    return true;
}

#endif /* ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY */
