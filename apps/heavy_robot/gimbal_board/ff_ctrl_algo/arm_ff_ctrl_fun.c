#include "arm_ff_ctrl_fun.h"
#include "arm_math.h"
#include "arm_common_tables.h"
#include <math.h>
#include <string.h>

/*
 * 本文件只处理运动学数学运算，不直接访问电机。
 * 所有矩阵均采用左上标/右下标语义：^AT_B 把 B 系坐标转换到 A 系。
 */

/**
 * @brief 计算两个刚体变换的乘积 out = left * right。
 *
 * 若 left = ^AT_B、right = ^BT_C，则 out = ^AT_C。函数只计算
 * R_out = R_left*R_right 和 p_out = p_left + R_left*p_right，跳过
 * 恒定的齐次矩阵最后一行。result 临时变量允许 out 与输入指针相同。
 */
static void Transform_Multiply(const RobotArmTransform *left, const RobotArmTransform *right, RobotArmTransform *out)
{
    RobotArmTransform result;

    for (uint8_t row = 0U; row < 3U; ++row)
    {
        const float l0 = left->element[row][0];
        const float l1 = left->element[row][1];
        const float l2 = left->element[row][2];

        result.element[row][0] = l0 * right->element[0][0] + l1 * right->element[1][0] + l2 * right->element[2][0];
        result.element[row][1] = l0 * right->element[0][1] + l1 * right->element[1][1] + l2 * right->element[2][1];
        result.element[row][2] = l0 * right->element[0][2] + l1 * right->element[1][2] + l2 * right->element[2][2];
        result.element[row][3] = left->element[row][3] + l0 * right->element[0][3] + l1 * right->element[1][3] + l2 * right->element[2][3];
    }

    *out = result;
}

/* 计算正弦和/或余弦值；空值输出将被跳过 */
static void FastSinCos(float angle_rad, float *sin_value, float *cos_value)
{
    if ((sin_value == NULL) && (cos_value == NULL))
    {
        return;
    }

    float   phase = angle_rad * 0.159154943092f;
    int32_t turn  = (int32_t)phase;
    if (phase < 0.0f)
    {
        --turn;
    }

    phase              -= (float)turn;
    float    table_pos  = (float)FAST_MATH_TABLE_SIZE * phase;
    uint16_t sin_index  = (uint16_t)table_pos;
    if (sin_index >= FAST_MATH_TABLE_SIZE)
    {
        sin_index = 0U;
        table_pos -= (float)FAST_MATH_TABLE_SIZE;
    }

    const float fraction = table_pos - (float)sin_index;

    if (sin_value != NULL)
    {
        const float sin_a = sinTable_f32[sin_index];
        const float sin_b = sinTable_f32[sin_index + 1U];
        *sin_value        = sin_a + fraction * (sin_b - sin_a);
    }

    if (cos_value != NULL)
    {
        const uint16_t cos_index =
            (uint16_t)((sin_index + (FAST_MATH_TABLE_SIZE / 4U)) & (FAST_MATH_TABLE_SIZE - 1U));
        const float cos_a = sinTable_f32[cos_index];
        const float cos_b = sinTable_f32[cos_index + 1U];
        *cos_value        = cos_a + fraction * (cos_b - cos_a);
    }
}

/**
 * @brief 将一个 Craig 修改 DH 关节变换直接右乘到当前变换。
 *
 * DH 矩阵中的固定零元素和最后一行均被省略。与先构造 4x4 矩阵再调用
 * 通用矩阵乘法相比，这里不需要临时 DH 矩阵，也不会执行乘零和加零操作。
 *
 * current 表示 ^0T_{i-1}，函数输出 next = ^0T_i。平移列
 * [a_{i-1}, -sin(alpha_{i-1})d_i, cos(alpha_{i-1})d_i]^T
 * 已在初始化阶段拆成三个缓存分量。
 */
static void Transform_AppendMdhJoint(const RobotArmTransform *current, const RobotArmKinematics *kinematics, uint8_t joint_index,
                                     float joint_angle_rad, RobotArmTransform *next, RobotArmTransform *parent_to_child)
{
    const float theta = joint_angle_rad + kinematics->theta_offset_rad[joint_index];
    float       sin_theta;
    float       cos_theta;
    FastSinCos(theta, &sin_theta, &cos_theta);
    const float sin_alpha = kinematics->sin_alpha[joint_index];
    const float cos_alpha = kinematics->cos_alpha[joint_index];

    // theta 每周期变化；先组合 alpha 缓存，使矩阵三行共用四个乘积。
    const float sin_theta_cos_alpha = sin_theta * cos_alpha;
    const float sin_theta_sin_alpha = sin_theta * sin_alpha;
    const float cos_theta_cos_alpha = cos_theta * cos_alpha;
    const float cos_theta_sin_alpha = cos_theta * sin_alpha;

    if (parent_to_child != NULL)
    {
        // ^{i-1}T_i：R 把当前坐标系向量转换到父坐标系，p 在父坐标系中表达。
        parent_to_child->element[0][0] = cos_theta;
        parent_to_child->element[0][1] = -sin_theta;
        parent_to_child->element[0][2] = 0.0f;
        parent_to_child->element[0][3] = kinematics->a_prev_m[joint_index];
        parent_to_child->element[1][0] = sin_theta_cos_alpha;
        parent_to_child->element[1][1] = cos_theta_cos_alpha;
        parent_to_child->element[1][2] = -sin_alpha;
        parent_to_child->element[1][3] = kinematics->translate_y_m[joint_index];
        parent_to_child->element[2][0] = sin_theta_sin_alpha;
        parent_to_child->element[2][1] = cos_theta_sin_alpha;
        parent_to_child->element[2][2] = cos_alpha;
        parent_to_child->element[2][3] = kinematics->translate_z_m[joint_index];
    }

    if ((current == NULL) || (next == NULL))
    {
        return;
    }

    for (uint8_t row = 0U; row < 3U; ++row)
    {
        const float r0 = current->element[row][0];
        const float r1 = current->element[row][1];
        const float r2 = current->element[row][2];

        next->element[row][0] = r0 * cos_theta + r1 * sin_theta_cos_alpha + r2 * sin_theta_sin_alpha;
        next->element[row][1] = -r0 * sin_theta + r1 * cos_theta_cos_alpha + r2 * cos_theta_sin_alpha;
        next->element[row][2] = -r1 * sin_alpha + r2 * cos_alpha;
        next->element[row][3] = current->element[row][3] + r0 * kinematics->a_prev_m[joint_index] + r1 * kinematics->translate_y_m[joint_index] +
                                r2 * kinematics->translate_z_m[joint_index];
    }
}

/**
 * @brief 计算三维叉乘 out = left x right。
 */
static void Vector3_Cross(const float left[3], const float right[3], float out[3])
{
    const float result[3] = {
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    };

    memcpy(out, result, sizeof(result));
}

/**
 * @brief 计算三阶方阵与三维列向量的乘积 out = matrix * vector。
 *
 * 结果先保存在标量中，因此 out 可以与 vector 指向同一数组。
 */
static void Matrix3_VectorMultiply(const float matrix[3][3], const float vector[3], float out[3])
{
    const float result_0 = matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2];
    const float result_1 = matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2];
    const float result_2 = matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2];

    out[0] = result_0;
    out[1] = result_1;
    out[2] = result_2;
}

/**
 * @brief 用 ^{parent}R_{child} 的转置把父坐标系向量转换到子坐标系。
 */
static void Rotation_TransposeMultiply(const RobotArmTransform *parent_to_child, const float parent_vector[3], float child_vector[3])
{
    child_vector[0] = parent_to_child->element[0][0] * parent_vector[0] + parent_to_child->element[1][0] * parent_vector[1] +
                      parent_to_child->element[2][0] * parent_vector[2];
    child_vector[1] = parent_to_child->element[0][1] * parent_vector[0] + parent_to_child->element[1][1] * parent_vector[1] +
                      parent_to_child->element[2][1] * parent_vector[2];
    child_vector[2] = parent_to_child->element[0][2] * parent_vector[0] + parent_to_child->element[1][2] * parent_vector[1] +
                      parent_to_child->element[2][2] * parent_vector[2];
}

/**
 * @brief 用 ^{parent}R_{child} 把子坐标系向量转换到父坐标系。
 */
static void Rotation_Multiply(const RobotArmTransform *parent_to_child, const float child_vector[3], float parent_vector[3])
{
    parent_vector[0] = parent_to_child->element[0][0] * child_vector[0] + parent_to_child->element[0][1] * child_vector[1] +
                       parent_to_child->element[0][2] * child_vector[2];
    parent_vector[1] = parent_to_child->element[1][0] * child_vector[0] + parent_to_child->element[1][1] * child_vector[1] +
                       parent_to_child->element[1][2] * child_vector[2];
    parent_vector[2] = parent_to_child->element[2][0] * child_vector[0] + parent_to_child->element[2][1] * child_vector[1] +
                       parent_to_child->element[2][2] * child_vector[2];
}

/**
 * @brief 从父连杆向当前旋转关节递推速度和加速度。
 *
 * parent_to_child 的位置列 p 是当前原点相对父原点的位置，并在父坐标系
 * 中表达。Craig 修改 DH 下该位置列与 theta_i 无关，所以当前关节转动
 * 不会移动自身原点，只会增加当前连杆绕 z_i 的角速度和角加速度。
 */
static void LinkMotion_Propagate(const RobotArmLinkMotion *parent_motion, const RobotArmTransform *parent_to_child,
                                 float joint_velocity_rad_s, float joint_acceleration_rad_s2, RobotArmLinkMotion *child_motion)
{
    const float position[3] = {parent_to_child->element[0][3], parent_to_child->element[1][3], parent_to_child->element[2][3]};
    float       omega_cross_position[3];
    float       alpha_cross_position[3];
    float       omega_cross_omega_cross_position[3];
    float       vector_parent[3];
    float       parent_omega_child[3];
    float       joint_angular_velocity_child[3] = {0.0f, 0.0f, joint_velocity_rad_s};
    float       coriolis_angular_acceleration[3];

    Rotation_TransposeMultiply(parent_to_child, parent_motion->angular_velocity_rad_s, parent_omega_child);

    child_motion->angular_velocity_rad_s[0] = parent_omega_child[0];
    child_motion->angular_velocity_rad_s[1] = parent_omega_child[1];
    child_motion->angular_velocity_rad_s[2] = parent_omega_child[2] + joint_velocity_rad_s;

    Vector3_Cross(parent_omega_child, joint_angular_velocity_child, coriolis_angular_acceleration);
    Rotation_TransposeMultiply(parent_to_child, parent_motion->angular_acceleration_rad_s2, child_motion->angular_acceleration_rad_s2);
    child_motion->angular_acceleration_rad_s2[0] += coriolis_angular_acceleration[0];
    child_motion->angular_acceleration_rad_s2[1] += coriolis_angular_acceleration[1];
    child_motion->angular_acceleration_rad_s2[2] += coriolis_angular_acceleration[2] + joint_acceleration_rad_s2;

    Vector3_Cross(parent_motion->angular_velocity_rad_s, position, omega_cross_position);
    vector_parent[0] = parent_motion->origin_linear_velocity_m_s[0] + omega_cross_position[0];
    vector_parent[1] = parent_motion->origin_linear_velocity_m_s[1] + omega_cross_position[1];
    vector_parent[2] = parent_motion->origin_linear_velocity_m_s[2] + omega_cross_position[2];
    Rotation_TransposeMultiply(parent_to_child, vector_parent, child_motion->origin_linear_velocity_m_s);

    Vector3_Cross(parent_motion->angular_acceleration_rad_s2, position, alpha_cross_position);
    Vector3_Cross(parent_motion->angular_velocity_rad_s, omega_cross_position, omega_cross_omega_cross_position);
    vector_parent[0] = parent_motion->origin_linear_acceleration_m_s2[0] + alpha_cross_position[0] + omega_cross_omega_cross_position[0];
    vector_parent[1] = parent_motion->origin_linear_acceleration_m_s2[1] + alpha_cross_position[1] + omega_cross_omega_cross_position[1];
    vector_parent[2] = parent_motion->origin_linear_acceleration_m_s2[2] + alpha_cross_position[2] + omega_cross_omega_cross_position[2];
    Rotation_TransposeMultiply(parent_to_child, vector_parent, child_motion->origin_linear_acceleration_m_s2);
}

/* ============================== 变换辅助接口 ================================ */

/**
 * @brief 将紧凑刚体变换初始化为单位变换。
 * @param transform 待初始化的变换；传 NULL 时不执行任何操作。
 */
void RobotArmTransform_SetIdentity(RobotArmTransform *transform)
{
    if (transform == NULL)
    {
        return;
    }

    memset(transform, 0, sizeof(*transform));
    transform->element[0][0] = 1.0f;
    transform->element[1][1] = 1.0f;
    transform->element[2][2] = 1.0f;
}

/**
 * @brief 将内部 3x4 紧凑变换展开成标准 4x4 齐次矩阵。
 * @param transform 输入的紧凑变换 [R p]。
 * @param matrix 输出矩阵；前三行复制输入，最后一行写为 [0 0 0 1]。
 */
void RobotArmTransform_ToMatrix4x4(const RobotArmTransform *transform, float matrix[4][4])
{
    if ((transform == NULL) || (matrix == NULL))
    {
        return;
    }

    for (uint8_t row = 0U; row < 3U; ++row)
    {
        for (uint8_t column = 0U; column < 4U; ++column)
        {
            matrix[row][column] = transform->element[row][column];
        }
    }

    matrix[3][0] = 0.0f;
    matrix[3][1] = 0.0f;
    matrix[3][2] = 0.0f;
    matrix[3][3] = 1.0f;
}

/* ============================== 模型初始化 ================================== */

/**
 * @brief 预计算 7 轴 Craig 修改 DH 模型中的固定参数。
 *
 * 对每个关节缓存 sin(alpha)、cos(alpha) 和 DH 平移列中的固定分量，
 * 从而避免正运动学周期内重复计算固定三角函数。
 *
 * @param kinematics 输出的运动学上下文。
 * @param dh_params 7 组 DH 参数，索引 0~6 对应 J1~J7。
 * @param joint7_to_tool 固定变换 ^7T_tool；传 NULL 时保存单位变换。
 * @return 输入指针有效并完成初始化时返回 true，否则返回 false。
 */
bool RobotArmKinematics_Init(RobotArmKinematics      *kinematics,
                                     const RobotArmMdhParam   dh_params[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                     const RobotArmTransform *joint7_to_tool)
{
    if ((kinematics == NULL) || (dh_params == NULL))
    {
        return false;
    }

    memset(kinematics, 0, sizeof(*kinematics));

    for (uint8_t i = 0U; i < ROBOT_ARM_KINEMATICS_JOINT_NUM; ++i)
    {
        float sin_alpha;
        float cos_alpha;
        FastSinCos(dh_params[i].alpha_prev_rad, &sin_alpha, &cos_alpha);

        kinematics->a_prev_m[i]  = dh_params[i].a_prev_m;
        kinematics->sin_alpha[i] = sin_alpha;
        kinematics->cos_alpha[i] = cos_alpha;
        // Craig 修改 DH 矩阵的位置列为 [a, -sin(alpha)*d, cos(alpha)*d]^T。
        kinematics->translate_y_m[i]    = -sin_alpha * dh_params[i].d_m;
        kinematics->translate_z_m[i]    = cos_alpha * dh_params[i].d_m;
        kinematics->theta_offset_rad[i] = dh_params[i].theta_offset_rad;
    }

    if (joint7_to_tool != NULL)
    {
        kinematics->joint7_to_tool = *joint7_to_tool;
    }
    else
    {
        RobotArmTransform_SetIdentity(&kinematics->joint7_to_tool);
    }

    kinematics->initialized = true;
    return true;
}

/* ============================== 步骤 1：位姿 ================================ */

/**
 * @brief 使用缓存的 Craig 修改 DH 参数计算 7 轴正运动学。
 *
 * 依次计算 ^0T_i = ^0T_{i-1} * ^{i-1}T_i，并在最后右乘
 * ^7T_tool 得到 ^0T_tool。base_to_joint 为 NULL 时只计算末端输出，
 * 不额外保存每一级结果。
 *
 * @param kinematics 已初始化的运动学上下文。
 * @param joint_angle_rad 7 个模型关节角，索引 0~6 对应 J1~J7，单位 rad。
 * @param base_to_tool 输出的基座到工具坐标系变换 ^0T_tool。
 * @param base_to_joint 可选输出数组，依次保存 ^0T_1~^0T_7。
 * @return 参数有效且上下文已初始化时返回 true，否则返回 false。
 */
bool RobotArmKinematics_Forward(const RobotArmKinematics *kinematics,
                                        const float joint_angle_rad[ROBOT_ARM_KINEMATICS_JOINT_NUM], RobotArmTransform *base_to_tool,
                                        RobotArmTransform base_to_joint[ROBOT_ARM_KINEMATICS_JOINT_NUM])
{
    if ((kinematics == NULL) || !kinematics->initialized || (joint_angle_rad == NULL) ||
        ((base_to_tool == NULL) && (base_to_joint == NULL)))
    {
        return false;
    }

    RobotArmTransform current; // 当前累计变换，循环开始前为 ^0T_0 = I
    RobotArmTransform next;    // 当前关节右乘后的结果，避免原地计算覆盖输入
    RobotArmTransform_SetIdentity(&current);

    for (uint8_t i = 0U; i < ROBOT_ARM_KINEMATICS_JOINT_NUM; ++i)
    {
        Transform_AppendMdhJoint(&current, kinematics, i, joint_angle_rad[i], &next, NULL);
        current = next;

        if (base_to_joint != NULL)
        {
            base_to_joint[i] = current;
        }
    }

    // 工具变换只右乘一次，未配置工具偏置时 joint7_to_tool 为单位矩阵。
    if (base_to_tool != NULL)
    {
        Transform_Multiply(&current, &kinematics->joint7_to_tool, base_to_tool);
    }
    return true;
}

/* ========================== 步骤 2：速度与加速度外推 ========================== */

/**
 * @brief 使用 Craig 修改 DH 模型正向递推连杆位姿、速度和加速度。
 */
bool RobotArmKinematics_ForwardMotion(const RobotArmKinematics *kinematics,
                                              const float                       joint_angle_rad[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                              const float                       joint_velocity_rad_s[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                              const float                       joint_acceleration_rad_s2[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                              const RobotArmLinkMotion *base_motion, RobotArmTransform *base_to_tool,
                                              RobotArmTransform  base_to_joint[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                              RobotArmTransform  parent_to_joint[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                              RobotArmLinkMotion link_motion[ROBOT_ARM_KINEMATICS_JOINT_NUM])
{
    if ((kinematics == NULL) || !kinematics->initialized || (joint_angle_rad == NULL) || (joint_velocity_rad_s == NULL) ||
        (joint_acceleration_rad_s2 == NULL) || (link_motion == NULL))
    {
        return false;
    }

    const bool                        update_cumulative = (base_to_tool != NULL) || (base_to_joint != NULL);
    RobotArmTransform         current;
    RobotArmTransform         next;
    RobotArmTransform         parent_to_child;
    RobotArmLinkMotion        stationary_base = {0};
    const RobotArmLinkMotion *parent_motion   = (base_motion != NULL) ? base_motion : &stationary_base;

    if (update_cumulative)
    {
        RobotArmTransform_SetIdentity(&current);
    }

    for (uint8_t i = 0U; i < ROBOT_ARM_KINEMATICS_JOINT_NUM; ++i)
    {
        // 同一次三角函数计算同时生成累计位姿和本级递推所需的相邻变换。
        Transform_AppendMdhJoint(update_cumulative ? &current : NULL, kinematics, i, joint_angle_rad[i],
                                 update_cumulative ? &next : NULL, &parent_to_child);
        LinkMotion_Propagate(parent_motion, &parent_to_child, joint_velocity_rad_s[i], joint_acceleration_rad_s2[i], &link_motion[i]);

        if (parent_to_joint != NULL)
        {
            parent_to_joint[i] = parent_to_child;
        }

        if (update_cumulative)
        {
            current = next;
        }
        parent_motion = &link_motion[i];

        if (base_to_joint != NULL)
        {
            base_to_joint[i] = current;
        }
    }

    if (base_to_tool != NULL)
    {
        Transform_Multiply(&current, &kinematics->joint7_to_tool, base_to_tool);
    }
    return true;
}

/* ========================== 步骤 3：连杆惯性载荷 ============================= */

/**
 * @brief 批量计算七个连杆的质心加速度、惯性合力和惯性合力矩。
 */
bool RobotArmJointDynamics_ComputeLinkInertia(const RobotArmLinkDynamicsParam dynamics_param[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                                 const RobotArmLinkMotion        link_motion[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                                 RobotArmLinkInertia             link_inertia[ROBOT_ARM_KINEMATICS_JOINT_NUM])
{
    if ((dynamics_param == NULL) || (link_motion == NULL) || (link_inertia == NULL))
    {
        return false;
    }

    for (uint8_t i = 0U; i < ROBOT_ARM_KINEMATICS_JOINT_NUM; ++i)
    {
        const RobotArmLinkDynamicsParam *param  = &dynamics_param[i];
        const RobotArmLinkMotion        *motion = &link_motion[i];
        RobotArmLinkInertia             *result = &link_inertia[i];
        float                                    alpha_cross_com[3];
        float                                    omega_cross_com[3];
        float                                    centripetal_acceleration[3];
        float                                    inertia_times_alpha[3];
        float                                    inertia_times_omega[3];
        float                                    gyroscopic_moment[3];

        Vector3_Cross(motion->angular_acceleration_rad_s2, param->center_of_mass_m, alpha_cross_com);
        Vector3_Cross(motion->angular_velocity_rad_s, param->center_of_mass_m, omega_cross_com);
        Vector3_Cross(motion->angular_velocity_rad_s, omega_cross_com, centripetal_acceleration);

        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            const float acceleration = motion->origin_linear_acceleration_m_s2[axis] + alpha_cross_com[axis] + centripetal_acceleration[axis];
            result->center_of_mass_linear_acceleration_m_s2[axis] = acceleration;
            result->inertial_force_n[axis]                        = param->mass_kg * acceleration;
        }

        Matrix3_VectorMultiply(param->inertia_com_kg_m2, motion->angular_acceleration_rad_s2, inertia_times_alpha);
        Matrix3_VectorMultiply(param->inertia_com_kg_m2, motion->angular_velocity_rad_s, inertia_times_omega);
        Vector3_Cross(motion->angular_velocity_rad_s, inertia_times_omega, gyroscopic_moment);

        result->inertial_moment_nm[0] = inertia_times_alpha[0] + gyroscopic_moment[0];
        result->inertial_moment_nm[1] = inertia_times_alpha[1] + gyroscopic_moment[1];
        result->inertial_moment_nm[2] = inertia_times_alpha[2] + gyroscopic_moment[2];
    }

    return true;
}

/* ========================== 步骤 4：载荷内推与关节力矩 ========================= */

/**
 * @brief 从末端向基座内推各连杆子树载荷，并投影得到关节力矩。
 */
bool RobotArmJointDynamics_Backward(const RobotArmLinkDynamicsParam dynamics_param[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                       const RobotArmLinkInertia       link_inertia[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                       const RobotArmTransform         parent_to_joint[ROBOT_ARM_KINEMATICS_JOINT_NUM],
                                       const RobotArmTransform        *joint7_to_tool,
                                       const RobotArmWrench           *external_wrench_tool,
                                       RobotArmLinkLoad                link_load[ROBOT_ARM_KINEMATICS_JOINT_NUM])
{
    if ((dynamics_param == NULL) || (link_inertia == NULL) || (parent_to_joint == NULL) || (joint7_to_tool == NULL) || (link_load == NULL))
    {
        return false;
    }

    float child_force_current[3]  = {0.0f, 0.0f, 0.0f};
    float child_moment_current[3] = {0.0f, 0.0f, 0.0f};

    if (external_wrench_tool != NULL)
    {
        float external_force_joint7[3];
        float external_moment_joint7[3];
        float tool_offset_cross_force[3];
        const float tool_position_joint7[3] = {
            joint7_to_tool->element[0][3],
            joint7_to_tool->element[1][3],
            joint7_to_tool->element[2][3],
        };

        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            if (!isfinite(external_wrench_tool->force_n[axis]) || !isfinite(external_wrench_tool->moment_nm[axis]))
            {
                return false;
            }
        }

        // ^7F = ^7R_tool * ^toolF；力矩还需从工具原点平移到 O7。
        Rotation_Multiply(joint7_to_tool, external_wrench_tool->force_n, external_force_joint7);
        Rotation_Multiply(joint7_to_tool, external_wrench_tool->moment_nm, external_moment_joint7);
        Vector3_Cross(tool_position_joint7, external_force_joint7, tool_offset_cross_force);

        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            // 输入是环境作用于机械臂的外力；执行器抵消它所需的边界载荷取反。
            child_force_current[axis]  = -external_force_joint7[axis];
            child_moment_current[axis] = -(external_moment_joint7[axis] + tool_offset_cross_force[axis]);
        }
    }

    for (int8_t i = (int8_t)ROBOT_ARM_KINEMATICS_JOINT_NUM - 1; i >= 0; --i)
    {
        const RobotArmLinkDynamicsParam *param   = &dynamics_param[i];
        const RobotArmLinkInertia       *inertia = &link_inertia[i];
        RobotArmLinkLoad                *load    = &link_load[i];
        float                                    com_force_moment[3];
        float                                    child_force_moment[3] = {0.0f, 0.0f, 0.0f};

        if (i < ((int8_t)ROBOT_ARM_KINEMATICS_JOINT_NUM - 1))
        {
            const RobotArmTransform *current_to_child          = &parent_to_joint[i + 1];
            const float                      child_position_current[3] = {
                current_to_child->element[0][3],
                current_to_child->element[1][3],
                current_to_child->element[2][3],
            };

            Rotation_Multiply(current_to_child, link_load[i + 1].subtree_force_n, child_force_current);
            Rotation_Multiply(current_to_child, link_load[i + 1].subtree_moment_nm, child_moment_current);
            Vector3_Cross(child_position_current, child_force_current, child_force_moment);
        }

        Vector3_Cross(param->center_of_mass_m, inertia->inertial_force_n, com_force_moment);

        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            load->subtree_force_n[axis] = inertia->inertial_force_n[axis] + child_force_current[axis];
            load->subtree_moment_nm[axis] =
                inertia->inertial_moment_nm[axis] + com_force_moment[axis] + child_moment_current[axis] + child_force_moment[axis];
        }

        // 当前模型全部为旋转关节，且关节轴在自身坐标系中恒为 z_i = [0, 0, 1]^T。
        load->joint_torque_nm = load->subtree_moment_nm[2];
    }

    return true;
}
