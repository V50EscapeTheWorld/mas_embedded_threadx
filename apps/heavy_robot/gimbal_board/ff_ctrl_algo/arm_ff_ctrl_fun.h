#ifndef __ARM_FF_CTRL_FUN_H__
#define __ARM_FF_CTRL_FUN_H__

#include <stdbool.h>
#include <stdint.h>
#include "heavy_robot_def.h"
/* 固定为 7 个旋转关节；数组索引 0~6 分别对应机械关节 J1~J7。 */
#ifndef ROBOT_ARM_KINEMATICS_JOINT_NUM
#define ROBOT_ARM_KINEMATICS_JOINT_NUM 7U
#endif
/**
 * @brief 单个旋转关节的 Craig 修改 DH 参数。
 *
 * 相邻坐标系变换采用：
 *   ^{i-1}T_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
 *   theta_i = joint_angle_i + theta_offset_i
 *
 * 参数数组第 0 项描述 ^0T_1，第 6 项描述 ^6T_7。这里的 theta_offset
 * 是建系产生的模型偏置，不是编码器零位偏置；编码器标定由
 * RobotArm_SetJointAngleCalibration() 负责。
 *
 * 长度统一使用 m，角度统一使用 rad。
 */
typedef struct
{
    float a_prev_m;         // a_{i-1}：沿 x_{i-1} 的公垂线长度，单位 m
    float alpha_prev_rad;   // alpha_{i-1}：绕 x_{i-1} 的连杆扭转角，单位 rad
    float d_m;              // d_i：沿 z_i 的固定关节偏距，单位 m
    float theta_offset_rad; // theta_i 的固定模型偏置，单位 rad
} RobotArmMdhParam;

/**
 * @brief 刚体齐次变换 ^AT_B 的紧凑表示 [R(3x3) p(3x1)]。
 *
 * 该矩阵把 B 坐标系中的坐标转换到 A 坐标系。其内存布局为：
 *
 *   [ r00 r01 r02 px ]
 *   [ r10 r11 r12 py ]
 *   [ r20 r21 r22 pz ]
 *
 * 完整 4x4 齐次矩阵的最后一行恒为 [0 0 0 1]，因此不存储它，
 * 可以减少 RAM 占用和实时矩阵连乘次数。
 */
typedef struct
{
    float element[3][4]; // element[0..2][0..2] 为 R，element[0..2][3] 为 p，按行存储
} RobotArmTransform;

/**
 * @brief 7 轴正运动学的预计算上下文。
 *
 * 该结构由 RobotArmKinematics_Init() 填充，调用者不应手动
 * 修改缓存字段。alpha、a、d 都是固定机械参数，在初始化时转换为
 * 可直接参与递推的数值，避免每个控制周期重复计算固定三角函数。
 */
typedef struct
{
    float                     a_prev_m[ROBOT_ARM_KINEMATICS_JOINT_NUM];         // 缓存 a_{i-1}，单位 m
    float                     sin_alpha[ROBOT_ARM_KINEMATICS_JOINT_NUM];        // 缓存 sin(alpha_{i-1})
    float                     cos_alpha[ROBOT_ARM_KINEMATICS_JOINT_NUM];        // 缓存 cos(alpha_{i-1})
    float                     translate_y_m[ROBOT_ARM_KINEMATICS_JOINT_NUM];    // 缓存 -sin(alpha_{i-1})*d_i，单位 m
    float                     translate_z_m[ROBOT_ARM_KINEMATICS_JOINT_NUM];    // 缓存  cos(alpha_{i-1})*d_i，单位 m
    float                     theta_offset_rad[ROBOT_ARM_KINEMATICS_JOINT_NUM]; // 缓存 theta_i 模型偏置，单位 rad
    RobotArmTransform         joint7_to_tool;                                   // 固定工具变换 ^7T_tool
    bool                      initialized;                                      // 初始化完成标志；false 时禁止执行正运动学
} RobotArmKinematics;

/**
 * @brief 一个坐标系原点及其刚性连杆的空间运动状态。
 *
 * 对 link_motion[i]，四个三维向量全部在当前连杆坐标系 {i+1} 中表达：
 * - angular_velocity_rad_s：连杆角速度 ^i(omega_i)
 * - angular_acceleration_rad_s2：连杆角加速度 ^i(dot(omega_i))
 * - origin_linear_velocity_m_s：坐标系原点 O_i 的线速度 ^i(v_i)
 * - origin_linear_acceleration_m_s2：坐标系原点 O_i 的线加速度 ^i(a_i)
 *
 * 使用当前坐标系表达可直接衔接递归牛顿-欧拉算法。对于固定基座，
 * base_motion 通常全部为 0；计算重力补偿时只需令基座线加速度为 -g。
 */
typedef struct
{
    float angular_velocity_rad_s[3];          // 当前连杆角速度，单位 rad/s
    float angular_acceleration_rad_s2[3];     // 当前连杆角加速度，单位 rad/s^2
    float origin_linear_velocity_m_s[3];      // 当前坐标系原点线速度，单位 m/s
    float origin_linear_acceleration_m_s2[3]; // 当前坐标系原点线加速度，单位 m/s^2
} RobotArmLinkMotion;

/**
 * @brief 单个刚性连杆的固定质量属性。
 *
 * center_of_mass_m 是从连杆坐标系原点 O_i 指向质心 C_i 的向量，表达在
 * 连杆坐标系 {i} 中。inertia_com_kg_m2 是关于质心 C_i 取矩、但矩阵分量
 * 沿 {i} 的坐标轴表达的 3x3 惯量张量。若 CAD 惯量使用主惯性轴，必须
 * 在填入本结构前将其旋转到 {i}，不需要用平行轴定理移到 O_i。
 */
typedef struct
{
    float mass_kg;                 // 连杆质量，单位 kg，必须大于或等于 0
    float center_of_mass_m[3];     // ^i(O_iC_i)，从 O_i 指向 C_i，单位 m
    float inertia_com_kg_m2[3][3]; // ^i(I_Ci)，关于 C_i、表达在 {i} 中，单位 kg*m^2
} RobotArmLinkDynamicsParam;

/**
 * @brief 单个连杆正向惯性计算结果。
 *
 * 三个向量均表达在该连杆自身坐标系 {i} 中。这里采用递归牛顿-欧拉算法
 * 的正号约定：F_i = m_i*a_Ci，N_i = I_Ci*alpha_i + omega_i x (I_Ci*omega_i)。
 * 它们表示产生当前运动所需的合力与绕质心合力矩，不是取负号的达朗贝尔惯性力。
 */
typedef struct
{
    float center_of_mass_linear_acceleration_m_s2[3]; // 质心 C_i 的线加速度，单位 m/s^2
    float inertial_force_n[3];                        // F_i = m_i*a_Ci，单位 N
    float inertial_moment_nm[3];                      // N_i，关于质心 C_i，单位 Nm
} RobotArmLinkInertia;

/**
 * @brief Newton-Euler 内推得到的单个连杆子树载荷和关节力矩。
 *
 * subtree_force_n 和 subtree_moment_nm 是父关节为驱动当前连杆及其全部
 * 子连杆所需施加的合力和合力矩，分别关于当前坐标系原点 O_i 取矩，且
 * 均表达在当前连杆坐标系 {i} 中。joint_torque_nm 是合力矩在旋转关节
 * z_i 轴上的投影，正方向与模型关节角 q_i 的正方向一致。
 */
typedef struct
{
    float subtree_force_n[3];   // 当前连杆及其子树所需合力，表达在 {i} 中，单位 N
    float subtree_moment_nm[3]; // 关于 O_i 的子树合力矩，表达在 {i} 中，单位 Nm
    float joint_torque_nm;      // tau_i = z_i^T*n_i，模型关节正方向，单位 Nm
} RobotArmLinkLoad;

/**
 * @brief 作用在某坐标系原点处的六维力。
 *
 * force_n 和 moment_nm 必须表达在调用接口指定的同一坐标系中，moment_nm
 * 是关于该坐标系原点取矩。该结构不隐含“环境作用于机械臂”或“机械臂
 * 作用于环境”的方向，具体正号约定由使用它的接口说明。
 */
typedef struct
{
    float force_n[3];   // 三维力，单位 N
    float moment_nm[3]; // 关于指定坐标系原点的三维力矩，单位 Nm
} RobotArmWrench;

/* ============================== 变换辅助接口 ================================ */

/**
 * @brief 将紧凑变换设置为单位矩阵。
 * @param transform 待写入的变换；传 NULL 时函数直接返回。
 */
void RobotArmTransform_SetIdentity(RobotArmTransform *transform);

/**
 * @brief 将紧凑变换展开为标准 4x4 齐次矩阵。
 *
 * 仅在通信、调试或其他模块明确需要 4x4 矩阵时调用，实时正运动学
 * 内部不需要执行这一步。
 * @param transform 输入的紧凑刚体变换。
 * @param matrix 输出的完整 4x4 齐次矩阵，最后一行写为 [0 0 0 1]。
 */
void RobotArmTransform_ToMatrix4x4(const RobotArmTransform *transform, float matrix[4][4]);

/* ============================== 模型初始化 ================================== */

/**
 * @brief 初始化 7 轴 Craig 修改 DH 运动学模型。
 * @param kinematics 输出的预计算上下文，生命周期必须覆盖后续计算。
 * @param dh_params 7 组参数，索引 0~6 必须依次对应 J1~J7。
 * @param joint7_to_tool 固定变换 ^7T_tool；传 NULL 表示工具系与 J7 坐标系重合。
 * @return 指针有效并完成初始化时返回 true，否则返回 false。
 */
bool RobotArmKinematics_Init(RobotArmKinematics      *kinematics,
                                     const RobotArmMdhParam   dh_params[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                     const RobotArmTransform *joint7_to_tool);

/* ============================== 步骤 1：位姿 ================================ */

/**
 * @brief 计算 7 轴正运动学。
 * 计算顺序为 ^0T_1 * ^1T_2 * ... * ^6T_7 * ^7T_tool。输入关节角
 * 必须是相对固定机械零位、方向已校正的模型角，不能直接使用未标定
 * 的电机原始角度。（留着用于单独求位姿）
 *
 * @param kinematics 已由 RobotArmKinematics_Init() 初始化的上下文。
 * @param joint_angle_rad J1 到 J7 的模型关节角，索引 0~6，单位 rad。
 * @param base_to_tool 可选输出 ^0T_tool；与 base_to_joint 不能同时为 NULL。
 * @param base_to_joint 可选输出数组：元素 0 为 ^0T_1，元素 6 为 ^0T_7；不需要时传 NULL 以减少写内存。
 * @return 参数有效且上下文已初始化时返回 true，否则返回 false。
 */
bool RobotArmKinematics_Forward(const RobotArmKinematics *kinematics,
                                        const float joint_angle_rad[ROBOT_ARM_KINEMATICS_JOINT_NUM], RobotArmTransform *base_to_tool,
                                        RobotArmTransform base_to_joint[ROBOT_ARM_KINEMATICS_JOINT_NUM]);

/* ========================== 步骤 2：速度与加速度外推 ========================== */

/**
 * @brief 同时计算 7 轴位姿及连杆速度、加速度的正向递推。
 *
 * 该函数只适用于当前模型中的旋转关节。每个关节的转轴在其自身坐标系
 * 中均为 z 轴，即 z = [0, 0, 1]^T。递推时每一级的所有运动向量都转换
 * 到当前连杆坐标系中保存。
 *
 * @param kinematics 已初始化的 Craig 修改 DH 运动学上下文。
 * @param joint_angle_rad 模型关节角 q，索引 0~6 对应 J1~J7，单位 rad。
 * @param joint_velocity_rad_s 模型关节角速度 q_dot，单位 rad/s。
 * @param joint_acceleration_rad_s2 模型关节角加速度 q_ddot，单位 rad/s^2。
 * @param base_motion 基座 {0} 的运动状态，所有向量在 {0} 中表达；传 NULL 表示静止基座。
 * @param base_to_tool 可选输出 ^0T_tool；传 NULL 时不维护累计末端位姿。
 * @param base_to_joint 可选输出 ^0T_1~^0T_7；不需要时传 NULL。
 * @param parent_to_joint 可选输出相邻变换 ^0T_1、^1T_2、...、^6T_7；不需要时传 NULL。
 * @param link_motion 输出 J1~J7 对应连杆的运动状态，不能为空。
 * @return 输入有效且模型已初始化时返回 true，否则返回 false。
 */
bool RobotArmKinematics_ForwardMotion(const RobotArmKinematics *kinematics,
                                              const float                       joint_angle_rad[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                              const float                       joint_velocity_rad_s[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                              const float                       joint_acceleration_rad_s2[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                              const RobotArmLinkMotion *base_motion, RobotArmTransform *base_to_tool,
                                              RobotArmTransform  base_to_joint[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                              RobotArmTransform  parent_to_joint[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                              RobotArmLinkMotion link_motion[ROBOT_ARM_KINEMATICS_JOINT_NUM]);

/* ========================== 步骤 3：连杆惯性载荷 ============================= */

/**
 * @brief 根据连杆运动状态计算质心加速度、惯性合力和惯性合力矩。
 *
 * 对每个连杆计算：
 *   a_C = a_O + alpha x r_OC + omega x (omega x r_OC)
 *   F   = m*a_C
 *   N_C = I_C*alpha + omega x (I_C*omega)
 *
 * @param dynamics_param J1~J7 对应连杆的固定质量、质心和质心惯量参数。
 * @param link_motion 正向递推得到的 J1~J7 连杆运动状态。
 * @param link_inertia 输出 J1~J7 的质心加速度、惯性合力和惯性合力矩。
 * @return 三个数组指针均有效时返回 true，否则返回 false。
 */
bool RobotArmJointDynamics_ComputeLinkInertia(const RobotArmLinkDynamicsParam dynamics_param[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                                 const RobotArmLinkMotion        link_motion[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                                 RobotArmLinkInertia             link_inertia[ROBOT_ARM_KINEMATICS_JOINT_NUM]);

/* ========================== 步骤 4：载荷内推与关节力矩 ========================= */

/**
 * @brief 从 J7 向 J1 执行 Newton-Euler 内推并计算各旋转关节力矩。
 *
 * 对连杆 i，先将子连杆载荷从 {i+1} 旋转到 {i}，再计算：
 *   f_i = F_i + R_i_i+1*f_i+1
 *   n_i = N_i + r_iCi x F_i + R_i_i+1*n_i+1
 *         + p_i_i+1 x (R_i_i+1*f_i+1)
 *   tau_i = z_i^T*n_i
 *
 * external_wrench_tool 表示环境作用在机械臂工具上的外部力，函数将其从
 * 工具系转换到 J7 坐标系，并取反得到执行器为抵消该外力所需提供的末端
 * 边界载荷。传 NULL 表示末端没有外部接触力。夹具和固定负载自身的质量
 * 属性仍应计入第 7 连杆参数，不能作为接触力重复加入。
 *
 * @param dynamics_param J1~J7 的质心位置等固定质量属性。
 * @param link_inertia 正向计算得到的 J1~J7 惯性合力和惯性合力矩。
 * @param parent_to_joint 相邻变换，元素 i 为 ^{i}T_{i+1}；数组元素 0 实际为 ^0T_1。
 * @param joint7_to_tool 固定工具变换 ^7T_tool，用于把工具系外力转换到 J7 系。
 * @param external_wrench_tool 环境施加到机械臂的外力，表达在工具系且关于工具原点取矩；可为 NULL。
 * @param link_load 输出每个连杆子树的合力、关于 O_i 的合力矩及关节力矩。
 * @return 必需指针和末端外力数值有效时返回 true，否则返回 false。
 */
bool RobotArmJointDynamics_Backward(const RobotArmLinkDynamicsParam dynamics_param[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                       const RobotArmLinkInertia       link_inertia[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                       const RobotArmTransform         parent_to_joint[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                       const RobotArmTransform        *joint7_to_tool,
                                       const RobotArmWrench           *external_wrench_tool,
                                       RobotArmLinkLoad                link_load[ROBOT_ARM_KINEMATICS_JOINT_NUM]);

#endif
