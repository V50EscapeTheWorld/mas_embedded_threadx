/*
 * @Description: 机械臂重力参数辨识 —— 位置环扫描 + 数据采集 (仅辨识模式编译)
 *
 * 仅在 ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY 时编译,
 * 其余模式下整个模块不产生任何代码, 也不占用 RAM。
 *
 * 为什么必须用位置环 (而不是纯重力补偿):
 *
 *   纯重力补偿下电机收到的是  T = tau_ff(q) = Y(q)·theta_current,
 *   而辨识要解的正是          Y(q)·theta = T。
 *   两式一合并得 theta == theta_current —— 恒等式, 与真实重力无关。
 *   换句话说模型错得再离谱, 这个回归都会"完美成功", 采集再多数据也没用。
 *
 *   加上位置环后
 *       T = tau_ff(q) + kp·(expect - q)
 *   静止时机械臂受力平衡, 有
 *       T = tau_g_real(q)
 *   位置误差 kp·(expect - q) 恰好承载了"真实重力 - 模型"的差值, 这才是有用信息。
 *
 * 采集流程 (全自动, 无需人工搬动):
 *   1. 以开机时的姿态为中心, 在关节限位内构造扫描区间;
 *   2. 用 Halton 低差异序列在工作空间里生成若干目标点;
 *   3. 限速插值走到目标点, 等所有轴速度降到阈值以下(停稳);
 *   4. 只在停稳窗口内输出样本 —— 保证上位机拿到的都是准静态数据。
 */

#ifndef _ARM_IDENTIFY_H_
#define _ARM_IDENTIFY_H_

#include "heavy_robot_def.h"

#if (ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY)

#include <stdbool.h>

/* ========== 位置环增益 ==========
   位置误差既是"执行误差"也是"辨识信息", 二者是同一个量。
   增益不走本文件的宏, 直接写在 arm_func.c 的 robotic_arm_error_controller[] 里,
   每个轴一个值 (各轴扭矩量级差很多, 不该共用同一个数)。
   kp 过大: 机械臂过刚, 误差被压得很小, 抗噪变差;
   kp 过小: 模型偏差会让机械臂偏得很远, 可能撞限位。
   注意静止时 T 恒等于 tau_g_real, 与 kp 取值无关 —— kp 只影响停在哪里。 */

/* ========== 扫描区间 ==========
   以开机姿态为中心、向两侧各展开该半幅(rad), 再按关节限位裁剪。 */
#ifndef ARM_IDENTIFY_SWEEP_RAD
#define ARM_IDENTIFY_SWEEP_RAD 0.6f
#endif

/* 目标点数量。7 轴需要足够多不同的姿态组合才能把参数分开, 太少会病态。 */
#ifndef ARM_IDENTIFY_WAYPOINTS
#define ARM_IDENTIFY_WAYPOINTS 24U
#endif

/* 每次调用把 q_cmd 朝目标推进的最大角度(rad/次)。控制环约 500Hz, 0.001 即 0.5 rad/s。 */
#ifndef ARM_IDENTIFY_SLEW_RAD
#define ARM_IDENTIFY_SLEW_RAD 0.0010f
#endif

/* 停稳判据: 与目标的偏差小于 ARRIVE_RAD 且速度小于 QD_OK_RAD_S */
#ifndef ARM_IDENTIFY_ARRIVE_RAD
#define ARM_IDENTIFY_ARRIVE_RAD 0.03f
#endif
#ifndef ARM_IDENTIFY_QD_OK_RAD_S
#define ARM_IDENTIFY_QD_OK_RAD_S 0.03f
#endif

/* 停稳后持续这么多帧才换下一个目标点 (约 0.6s), 其中按 DECIMATION 抽样输出 */
#ifndef ARM_IDENTIFY_HOLD_TICKS
#define ARM_IDENTIFY_HOLD_TICKS 300U
#endif
/* 单个目标点最多等这么多帧还没停稳就跳过 (约 4s) */
#ifndef ARM_IDENTIFY_TIMEOUT_TICKS
#define ARM_IDENTIFY_TIMEOUT_TICKS 2000U
#endif

/* 采样降频: 停稳窗口内每 N 帧输出一组 (500Hz / 10 = 50Hz) */
#ifndef ARM_IDENTIFY_DECIMATION
#define ARM_IDENTIFY_DECIMATION 10U
#endif

/**
 * @brief 推进扫描轨迹, 并在机械臂停稳时输出一组样本。
 *
 * 每控制周期调用一次, 内部依次完成: 更新期望角 → 判断是否停稳 → 采样输出。
 * 期望角通过 RoboticArm_SetExpect() 写入, 供 RoboticArm_Control 的位置环使用。
 *
 * 输出为 SEGGER RTT 上的四行 CSV (ULOG_LINE_BUF_SIZE=128 放不下 28 列, 故拆分):
 *     Q,<q1..q7>        模型关节角, 单位 rad
 *     D,<qd1..qd7>      电机原始角速度, 单位 rad/s
 *     T,<tau1..tau7>    实际下发到电机的扭矩, 单位 Nm (已含限幅)
 *     G,<tg1..tg7>      限幅前的模型重力前馈, 单位 Nm
 *
 * 上位机用 T 做辨识 (静止时 T == tau_g_real(q)), 用 G 校验 DH 约定
 * (Y(q)@当前参数 应当与 G 精确吻合)。
 *
 * @return 本帧是否输出了样本
 */
bool ArmIdentify_Update(void);

#endif /* ROBOTIC_ARM_CTRL_MODE == ROBOTIC_ARM_CTRL_MODE_IDENTIFY */

#endif // _ARM_IDENTIFY_H_
