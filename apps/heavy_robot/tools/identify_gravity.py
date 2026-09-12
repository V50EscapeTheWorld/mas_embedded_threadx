#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂重力参数辨识

固件侧在 ROBOTIC_ARM_CTRL_MODE_IDENTIFY 模式下, 经 SEGGER RTT 输出三行一组的数据:
    Q,<q1..q7>        模型关节角, rad
    D,<qd1..qd7>      电机原始角速度, rad/s
    T,<tau1..tau7>    实际下发到电机的扭矩, Nm

用法:
    python identify_gravity.py <log.txt> [--force]

辨识原理:
    机械臂在任一姿态静止时受力平衡, 电机下发的扭矩恰好抵消重力:
        tau = tau_g(q) = Y(q) @ theta
    参数 theta 每根连杆 4 个: (m, m*cx, m*cy, m*cz)。
    该式对参数严格线性, 因此收集足够多姿态后一次最小二乘即可解出。

    ⚠ 这条等式只对"静止"成立。固件因此跑的是位置环 + 扫描轨迹, 而且只在
    停稳窗口采样。绝不能拿纯重力补偿下的数据来辨识 —— 那种情况下下发的扭矩
    恒等于模型自己的输出 Y(q)@theta_current, 回归退化成恒等式, 解出来永远
    是原参数, 模型错得再离谱也"完美通过"。信息必须来自位置环的位置误差。

    但重力回归矩阵必然秩亏(见 main() 中的说明), 所以求解时向 CAD 先验做
    Tikhonov 正则化: 可辨识的方向由数据决定, 退化方向保持先验值。

辨识前会先做一次交叉验证: 用固件里当前的模型参数重算 tau, 与固件下发的 tau
逐点对比。两者必须吻合到浮点精度, 否则说明本脚本的 DH 约定与 arm_ff_ctrl_fun.c
不一致, 此时直接终止, 避免解出错误参数。
"""

import argparse
import sys

import numpy as np

# ================================================================
# 模型常量 —— 必须与固件保持一致
#
# 改固件后如果忘了同步这里, 交叉验证会立刻报错, 不会静默出错。
# 来源: apps/heavy_robot/heavy_robot_def.h
#       apps/heavy_robot/gimbal_board/arm_func/arm_func.c
# ================================================================

LENGTH_2 = 0.3400
LENGTH_3 = 0.1905
LENGTH_5 = 0.1494
D_7 = 0.0385
GRAVITY = 9.8011

# dh_params, 每行 = (a_prev, alpha_prev, d, theta_offset)
DH = np.array(
    [
        [0.0, 0.0, 0.0, 0.0],
        [0.0, np.pi / 2, 0.0, np.pi / 2],
        [LENGTH_2, np.pi, 0.0, np.pi / 2],
        [0.0, np.pi / 2, LENGTH_3, 0.0],
        [0.0, -np.pi / 2, 0.0, 0.0],
        [0.0, np.pi / 2, LENGTH_5, 0.0],
        [0.0, -np.pi / 2, D_7, 0.0],
    ],
    dtype=float,
)

# joint[].angle_direction
DIRECTION = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0], dtype=float)

# link_dynamics 当前值 (合并 4 次采集的辨识结果, 2026-09-12)
# 必须与固件同步: 交叉验证会拿这些值重算 tau 与固件的 G 行比对,
# 不一致就中止。所以重跑旧日志(用旧参数采的)会被正确拦下。
CUR_MASS = np.array([0.80, 2.40, 1.0643, 0.4542, 0.4237, 0.4092, 0.6096], dtype=float)
CUR_COM = np.array(
    [
        [0.0, 0.0, 0.07],
        [0.2233, 0.0332, 0.0],
        [-0.1658, -0.0996, 0.0],
        [0.1035, -0.0132, 0.0688],
        [-0.1210, -0.1422, -0.0142],
        [-0.0008, -0.0015, 0.0789],
        [-0.0176, -0.0234, -0.0010],
    ],
    dtype=float,
)

# robotic_arm_joint_limit[].max_torque_nm, 用于统计被限幅的样本
TAU_LIMIT = np.array([37.0, 35.0, 35.0, 12.0, 30.0, 3.0, 12.0], dtype=float)

# robotic_arm_joint_limit 的 (min, max) 模型关节角, 用于评估激励是否充分
Q_LIMIT = np.array(
    [
        [-1.0, 1.0],
        [-1.65, 1.2],
        [-2.9, 2.8],
        [-2 * np.pi, 2 * np.pi],
        [-1.8, 2.4],
        [-2 * np.pi, 2 * np.pi],
        [-1.6, 1.6],
    ],
    dtype=float,
)

# 单轴摆动幅度低于此值 (rad) 视为"几乎没动", 该轴的参数不可辨识
COVERAGE_MIN_RAD = 0.8

N_JOINT = 7
N_LINK = 7
LINK_OFFSET = 1  # 只辨识 index 1..6 (link2~7); link1 不影响任何关节力矩
N_LINK_ID = N_LINK - LINK_OFFSET
N_PARAM = N_LINK_ID * 4  # 每杆 (m, m*cx, m*cy, m*cz)

# 交叉验证门槛 (Nm)。正常浮点误差在 1e-3 以下, 约定不一致会差到 Nm 量级。
VALIDATE_TOL = 1e-2

# 角速度上限 (rad/s)。固件只在停稳窗口采样, 所以实测应远低于此值;
# 超过就说明数据不是准静态的, tau = tau_g 不成立, 结果不可信。
# 0.03 rad/s 是经验值: 更大的速度会让摩擦和惯性项混进来(实测 0.1 rad/s 时
# 残差约 0.13 Nm, 远高于日志量化精度 1e-4)。
QD_MAX = 0.03


# ================================================================
# 运动学与动力学 —— 逐行对应 ff_ctrl_algo/arm_ff_ctrl_fun.c
# ================================================================


def dh_transforms(q):
    """返回 7 个 ^{i-1}T_i, 形状 (7,3,4), 与 C 里 R+p 的紧凑形式一致。"""
    Ts = np.zeros((N_JOINT, 3, 4))
    for i in range(N_JOINT):
        a, alpha, d, off = DH[i]
        th = q[i] + off
        ct, st = np.cos(th), np.sin(th)
        ca, sa = np.cos(alpha), np.sin(alpha)
        # 对应 Transform_AppendMdhJoint
        Ts[i, 0] = [ct, -st, 0.0, a]
        Ts[i, 1] = [st * ca, ct * ca, -sa, -sa * d]
        Ts[i, 2] = [st * sa, ct * sa, ca, ca * d]
    return Ts


def link_gravity_accel(Ts):
    """各连杆原点线加速度(本体系)。ω=α=0 时 a_i = R_i^T a_{i-1}, 质心加速度 == 原点加速度。"""
    a = np.array([0.0, 0.0, GRAVITY])  # 虚拟基座加速度取 +g, 等价于重力沿 -z0
    out = np.zeros((N_LINK, 3))
    for i in range(N_LINK):
        a = Ts[i][:, :3].T @ a
        out[i] = a
    return out


def joint_torques(Ts, a_link, masses, pis):
    """
    给定各连杆质量与一阶矩 pi = m*c, 返回 7 个关节的模型力矩(已乘 angle_direction)。
    ω=α=0 使惯性合力矩恒为 0, 质心项退化为 pi x a。
    对应 ComputeLinkInertia + Backward 两个函数。
    """
    F = [masses[i] * a_link[i] for i in range(N_LINK)]
    com = [np.cross(pis[i], a_link[i]) for i in range(N_LINK)]

    sf = [np.zeros(3) for _ in range(N_LINK)]  # 子树合力
    sm = [np.zeros(3) for _ in range(N_LINK)]  # 子树合力矩

    for i in range(N_LINK - 1, -1, -1):
        if i < N_LINK - 1:
            R = Ts[i + 1][:, :3]  # ^{i}R_{i+1}
            p = Ts[i + 1][:, 3]  # O_{i+1} 在 i 系中的位置
            child_F = R @ sf[i + 1]
            child_M = R @ sm[i + 1]
            cross = np.cross(p, child_F)
        else:
            child_F = np.zeros(3)
            child_M = np.zeros(3)
            cross = np.zeros(3)

        sf[i] = F[i] + child_F
        sm[i] = com[i] + child_M + cross

    return DIRECTION * np.array(sm)[:, 2]


def build_regressor(q):
    """返回 (7, N_PARAM): Y[i, col] = 第 col 个参数取 1、其余取 0 时第 i 轴的力矩。"""
    Ts = dh_transforms(q)
    a_link = link_gravity_accel(Ts)

    Y = np.zeros((N_JOINT, N_PARAM))
    for j in range(LINK_OFFSET, N_LINK):
        base = (j - LINK_OFFSET) * 4
        for which in range(4):
            masses = np.zeros(N_LINK)
            pis = np.zeros((N_LINK, 3))
            if which == 0:
                masses[j] = 1.0
            else:
                pis[j, which - 1] = 1.0
            Y[:, base + which] = joint_torques(Ts, a_link, masses, pis)
    return Y


def current_theta():
    """固件当前参数对应的一阶矩向量, 用于交叉验证。"""
    theta = np.zeros(N_PARAM)
    for j in range(LINK_OFFSET, N_LINK):
        base = (j - LINK_OFFSET) * 4
        theta[base] = CUR_MASS[j]
        theta[base + 1 : base + 4] = CUR_MASS[j] * CUR_COM[j]
    return theta


def solve_ridge(A, b, x_prior, lam):
    """
    解 min ||A x - b||^2 + lam ||x - x_prior||^2。
    先对每列归一化, 使 lam 无量纲且不受各参数量纲差异影响。
    用增广矩阵做最小二乘而非法方程: 回归矩阵条件数约 1e16, 法方程会把它平方后丢光精度。
    返回 (解, A 的秩)。
    """
    s_col = np.linalg.norm(A, axis=0)
    s_col[s_col < 1e-12] = 1.0
    An = A / s_col

    m = A.shape[1]
    sq = np.sqrt(lam)
    A_aug = np.vstack([An, sq * np.eye(m)])
    b_aug = np.concatenate([b, sq * (x_prior * s_col)])

    sol, _, _, _ = np.linalg.lstsq(A_aug, b_aug, rcond=None)
    return sol / s_col, int(np.linalg.matrix_rank(A))


# ================================================================
# 日志解析
# ================================================================


def _normalize_line(line):
    """
    兼容两种日志来源:
      1. 裸 RTT 输出:      Q,0.12345,-1.23456,...
      2. Ozone Terminal 导出 (CSV): 123,"Q,0.12345,-1.23456,..."

    后者带行号前缀且整段被引号包住, 这里剥掉, 还原成裸文本。
    不是该格式的行原样返回。
    """
    head, sep, rest = line.partition(",")
    if sep and head.strip().isdigit():
        line = rest
    if len(line) >= 2 and line.startswith('"') and line.endswith('"'):
        line = line[1:-1].replace('""', '"')
    return line


def parse_log(path):
    """挑出 Q/D/T/G 四行重组样本, 其它日志行一律跳过。

    返回 (q, qd, tau, tau_model):
        q, qd       模型关节角 / 原始角速度
        tau         实际下发到电机的扭矩(含限幅), 用于辨识
        tau_model   限幅前的模型输出, 用于校验本脚本的 DH 约定
    """
    qs, ds, ts, gs = [], [], [], []
    pending = {}

    with open(path, "r", errors="ignore") as fp:
        for raw in fp:
            line = raw.strip()
            if not line or line.startswith("LineNo,"):
                continue
            line = _normalize_line(line)
            if "," not in line:
                continue
            tag, _, rest = line.partition(",")
            tag = tag.strip()
            if tag not in ("Q", "D", "T", "G"):
                continue
            try:
                vals = [float(x) for x in rest.split(",")]
            except ValueError:
                continue
            if len(vals) != N_JOINT:
                continue

            pending[tag] = vals
            if len(pending) == 4:
                qs.append(pending["Q"])
                ds.append(pending["D"])
                ts.append(pending["T"])
                gs.append(pending["G"])
                pending = {}

    if not qs:
        sys.exit(
            "错误: 日志里没有解析到任何完整样本 (Q/D/T/G 四行一组)。\n"
            "      常见原因: 固件没进控制循环 —— arm_func.c 里的 signal 需要非 0;\n"
            "      或者只抓到了开机日志, 没有搬动机械臂。"
        )

    return np.array(qs), np.array(ds), np.array(ts), np.array(gs)


# ================================================================
# 主流程
# ================================================================


def report_coverage(q):
    """打印各轴的角度覆盖范围, 用来判断采集是否够、激励是否充分。

    注意 J1: 它是竖直偏航轴, 转动不改变任何连杆的高度, 因此对全部关节的
    重力矩都没有贡献(实测力矩完全不变)。它只需要动起来以便辨识自身的摩擦,
    所以不纳入"激励不足"告警。
    """
    print("\n关节空间覆盖:")
    print(f"{'轴':>4} {'最小':>9} {'最大':>9} {'实际范围':>10} {'行程':>9} {'占比':>7}")
    starved = []
    for i in range(N_JOINT):
        lo, hi = float(q[:, i].min()), float(q[:, i].max())
        rng = hi - lo
        span = Q_LIMIT[i, 1] - Q_LIMIT[i, 0]
        note = ""
        if i == 0:
            note = "  (仅影响自身摩擦)"
        elif rng < COVERAGE_MIN_RAD:
            note = "  <-- 几乎没动"
            starved.append(i + 1)
        print(
            f"J{i + 1:>3} {lo:9.3f} {hi:9.3f} {rng:10.3f} {span:9.3f} {rng / span:6.0%}{note}"
        )
    if starved:
        print(
            "  警告: J" + ", J".join(str(i) for i in starved) + " 几乎没有被激励。\n"
            "        这些轴的参数解不出来, 会退化成先验值。重采时请让它们跟着摆动。"
        )
    return not starved


def main():
    ap = argparse.ArgumentParser(description="机械臂重力参数辨识")
    ap.add_argument("log", help="固件在 IDENTIFY 模式下的 RTT 日志文件")
    ap.add_argument(
        "--lam",
        type=float,
        default=1e-4,
        help="向 CAD 先验正则化的强度(默认 1e-4)。退化方向取先验值; "
        "调大更贴近先验, 调小更贴合数据",
    )
    ap.add_argument("--force", action="store_true", help="交叉验证失败时仍然继续")
    args = ap.parse_args()

    q, qd, tau, tau_model = parse_log(args.log)
    n = len(q)
    print(f"解析到 {n} 组样本")
    report_coverage(q)

    # ---------- 0. 先排除"自证数据" ----------
    # 纯重力补偿(位置环关闭)时, 下发扭矩恒等于模型自己的输出:
    #     T = tau_ff(q) = Y(q)@theta_current
    # 而辨识要解的正是 Y(q)@theta = T, 两式合并得 theta == theta_current。
    # 这是恒等式, 模型错得再离谱也会"完美成功"。必须挡在这里。
    gap = float(np.abs(tau - tau_model).max())
    print(f"T 与 G 的差距: 最大 {gap:.4f} Nm (纯重力补偿下会恒等于 0)")
    if gap < 1e-3:
        print(
            "  !! T 与 G 几乎完全相同 —— 固件跑的是纯重力补偿, 位置环没开。\n"
            "     这种数据只能解出原参数, 没有任何信息量。",
            file=sys.stderr,
        )
        sys.exit("数据无效: 请确认 ROBOTIC_ARM_CTRL_MODE_IDENTIFY 下位置环已开启")

    # ---------- 1. 交叉验证 ----------
    # 比 G(限幅前的模型输出), 它与摩擦和限幅都无关, 因此可以要求精确吻合。
    # 这一步校验的是"本脚本复刻的 DH 是否与固件一致"。
    theta_cur = current_theta()
    err = np.array([build_regressor(q[s]) @ theta_cur - tau_model[s] for s in range(n)])
    max_err = float(np.max(np.abs(err)))
    print(f"交叉验证: 最大偏差 {max_err:.3e} Nm")
    if not np.isfinite(max_err) or max_err > VALIDATE_TOL:
        print(
            "  -> 与固件模型不一致。请核对本文件顶部的 "
            "DH / DIRECTION / CUR_MASS / CUR_COM。",
            file=sys.stderr,
        )
        if not args.force:
            sys.exit("交叉验证未通过, 已中止 (确认无误可加 --force 强制继续)")
    else:
        print("  -> 通过, 本脚本的 DH 约定与固件一致")

    # ---------- 2. 组装最小二乘系统 ----------
    # 静止时受力平衡, T == tau_g_real(q)。固件只在停稳窗口采样, 拿到的
    # 都是准静态数据, 因此不需要摩擦项, 也不需要按速度筛行。
    print(f"角速度: 最大 {np.abs(qd).max():.4f} rad/s (准静态要求 < {QD_MAX})")
    if np.abs(qd).max() > QD_MAX:
        print("  ^ 数据不是准静态的, tau = tau_g 不成立, 结果不可信")

    rows = n * N_JOINT
    cols = N_PARAM
    A = np.zeros((rows, cols))
    for s in range(n):
        A[s * N_JOINT : (s + 1) * N_JOINT, :] = build_regressor(q[s])
    b = tau.reshape(-1)

    # 剔除被限幅的行: 该轴并非由重力平衡, tau = tau_g 不成立
    joint_of_row = np.tile(np.arange(N_JOINT), n)
    keep = np.abs(b) < np.tile(TAU_LIMIT, n) * 0.999
    dropped = int(rows - keep.sum())
    if dropped:
        print(f"剔除 {dropped} 行 ({dropped / rows:.1%}): 扭矩被限幅")
    A, b, joint_of_row = A[keep], b[keep], joint_of_row[keep]

    # ---------- 3. 求解: 向 CAD 先验做 Tikhonov 正则化 ----------
    #
    # 重力回归矩阵必然秩亏(例如本 DH 下 d_2=0, 使 link2 质量完全不可辨识)。
    # 纯最小二乘会落到无穷多解里的任意一个, 解出的质心可能毫无意义。
    # 因此把参数往 CAD 先验拉: 可辨识方向由数据决定, 退化方向保持先验值。
    x_prior = np.zeros(cols)
    for j in range(LINK_OFFSET, N_LINK):
        base = (j - LINK_OFFSET) * 4
        x_prior[base] = CUR_MASS[j]
        x_prior[base + 1 : base + 4] = CUR_MASS[j] * CUR_COM[j]

    sol, rank = solve_ridge(A, b, x_prior, args.lam)
    print(f"\n最小二乘: {len(b)} 个方程, {cols} 个未知量, 秩 {rank} (正则化强度 lambda={args.lam:g})")
    if rank < cols:
        print(f"  已知 {cols - rank} 个方向不可辨识, 这些方向将保持 CAD 先验值")

    theta = sol[:N_PARAM]
    resid = b - A @ sol
    per_joint = np.array(
        [np.sqrt(np.mean(resid[joint_of_row == i] ** 2)) for i in range(N_JOINT)]
    )
    print(f"残差 RMS: 总 {np.sqrt(np.mean(resid**2)):.4f} Nm, 最大 {np.max(np.abs(resid)):.4f} Nm")
    print("  各轴 RMS: " + "  ".join(f"J{i + 1}={per_joint[i]:.4f}" for i in range(N_JOINT)))

    # ---------- 判断结果有没有用: 拿同样的数据比"当前参数"和"辨识参数" ----------
    # 注意 J1 的重力矩恒为 0, 它的残差就是该轴的静摩擦, 是整套辨识的精度下限。
    resid_prior = b - A @ x_prior
    per_joint_prior = np.array(
        [np.sqrt(np.mean(resid_prior[joint_of_row == i] ** 2)) for i in range(N_JOINT)]
    )
    rms_prior = float(np.sqrt(np.mean(resid_prior**2)))
    rms_new = float(np.sqrt(np.mean(resid**2)))
    print(f"\n对比当前参数 (改善倍数 = 当前残差 / 辨识后残差):")
    print(f"  总 RMS  {rms_prior:.4f} -> {rms_new:.4f} Nm   ({rms_prior / rms_new:.2f}x)")
    print(f"  {'轴':>3} {'当前':>9} {'辨识后':>9} {'改善':>8}")
    for i in range(N_JOINT):
        ratio = per_joint_prior[i] / per_joint[i] if per_joint[i] > 1e-9 else float("inf")
        flag = "  <-- 变差" if ratio < 0.95 else ""
        print(f"  J{i + 1:<2} {per_joint_prior[i]:9.4f} {per_joint[i]:9.4f} {ratio:7.2f}x{flag}")
    if rms_new >= rms_prior:
        print("  !! 辨识后整体没有改善 —— 结果不可用, 检查数据质量或 DH 结构")

    # 限幅统计: T 被 WriteTorque 截到上限即视为该轴被限幅 (注意有摩擦时 T 本就不等于 G)
    clamped = (np.abs(tau) >= TAU_LIMIT * 0.999).sum(axis=0)
    if clamped.any():
        print("  限幅样本数: " + "  ".join(f"J{i + 1}={clamped[i]}" for i in range(N_JOINT)))

    # ---------- 4. 结果 ----------
    print("\n辨识结果:")
    print(f"{'杆':>6} {'m(kg)':>9} {'cx':>9} {'cy':>9} {'cz':>9} {'|c|':>8}   当前 |c|")
    coms = np.zeros((N_LINK, 3))
    masses = np.zeros(N_LINK)
    for j in range(LINK_OFFSET, N_LINK):
        base = (j - LINK_OFFSET) * 4
        m = theta[base]
        pi = theta[base + 1 : base + 4]
        c = pi / m if abs(m) > 1e-9 else np.zeros(3)
        masses[j] = m
        coms[j] = c
        flag = "  <-- 异常" if np.linalg.norm(c) > 0.5 else ""
        print(
            f"link{j + 1:>1} {m:9.3f} {c[0]:9.4f} {c[1]:9.4f} {c[2]:9.4f}"
            f" {np.linalg.norm(c):8.4f} {np.linalg.norm(CUR_COM[j]):8.4f}{flag}"
        )

    # ---------- 5. 生成可直接粘贴的代码 ----------
    print("\n---- 粘贴回 arm_func.c 的 link_dynamics ----")
    for j in range(N_LINK):
        if j < LINK_OFFSET:
            print(f"  /* link{j + 1}: 不参与辨识, 保持原值 */")
            continue
        print(
            f"  {{.mass_kg = {masses[j]:.4f}f, .center_of_mass_m = "
            f"{{{coms[j][0]:.4f}f, {coms[j][1]:.4f}f, {coms[j][2]:.4f}f}}}},"
            f"  /* link{j + 1} */"
        )

    print("\n提示: 残差若不是随机分布(例如对某个 q 呈波浪), 说明 DH 结构本身有误,")
    print("      最小二乘只是把结构误差吸收进参数里, 应先修 DH 再重跑。")


if __name__ == "__main__":
    main()
