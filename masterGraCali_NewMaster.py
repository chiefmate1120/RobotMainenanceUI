"""主手重力标定 MATLAB 代码的 Python 版本。

由以下 4 个 MATLAB 文件转换而来：
    Identification.m
    MasterGravity_Lagrange.m
    MasterGravityRegMat.m
    trqConstIdentify.m

对外接口：
    Identification(armIdx, cmdJntPos, actJntPos, actJntTrq)

输入数组形状均为 (n, 7)，输出与 MATLAB 版本一致。

依赖：
    numpy
    scipy
可选：
    matplotlib（仅 show_plots=True 时用于诊断绘图）
"""

import numpy as np
from scipy.signal import butter, filtfilt
from scipy.optimize import lsq_linear


# ============================================================
# 对外函数
# ============================================================
def Identification(armIdx, cmdJntPos, actJntPos, actJntTrq, show_plots=True):
    """主手重力、摩擦、力矩常数比例和齿槽转矩辨识。

    Parameters
    ----------
    armIdx : int
        0 = 左主手，1 = 右主手。
    cmdJntPos : array_like, shape (n, 7)
        指令关节位置。
    actJntPos : array_like, shape (n, 7)
        实际关节位置。
    actJntTrq : array_like, shape (n, 7)
        实际关节力矩。
    show_plots : bool, optional
        是否使用 matplotlib 显示诊断图，默认 False。

    Returns
    -------
    gravityParams : ndarray, shape (30,)
    frictionParams : ndarray, shape (7,)
    trqConstScale : ndarray, shape (2,)
    cogTrqMap5 : ndarray, shape (1064,)
    cogTrqMap6 : ndarray, shape (560,)
    cogTrqMap7 : ndarray, shape (1440,)
    """

    cmd_jnt_pos = np.asarray(cmdJntPos, dtype=float)
    act_jnt_pos_raw = np.asarray(actJntPos, dtype=float)
    act_jnt_trq_raw = np.asarray(actJntTrq, dtype=float)

    if (
        cmd_jnt_pos.ndim != 2
        or cmd_jnt_pos.shape[1] != 7
        or act_jnt_pos_raw.shape != cmd_jnt_pos.shape
        or act_jnt_trq_raw.shape != cmd_jnt_pos.shape
    ):
        raise ValueError(
            "cmdJntPos, actJntPos and actJntTrq must all have shape (n, 7)."
        )

    if len(cmd_jnt_pos) == 0:
        raise ValueError("Input data is empty.")

    arm_idx = int(armIdx)

    # --------------------------------------------------------
    # 低通滤波
    # --------------------------------------------------------
    fs = 100.0
    fc = 3.0
    b, a = butter(1, 2.0 * fc / fs)

    try:
        act_jnt_pos = filtfilt(b, a, act_jnt_pos_raw, axis=0)
        act_jnt_trq = filtfilt(b, a, act_jnt_trq_raw, axis=0)
    except ValueError as exc:
        print(f"[MasterGraCali] filtfilt failed: {exc}; use raw data.")
        act_jnt_pos = act_jnt_pos_raw.copy()
        act_jnt_trq = act_jnt_trq_raw.copy()

    plt = _get_plt(show_plots)

    # if plt is not None:
    #     plt.figure()
    #     plt.plot(act_jnt_pos)
    #     plt.legend([f"Jnt{i}" for i in range(1, 8)])
    #     plt.title("Joint Position of all the calibrated process")

    #     plt.figure()
    #     plt.plot(act_jnt_trq)
    #     plt.legend([f"Jnt{i}" for i in range(1, 8)])
    #     plt.title("Joint Torque of all the calibrated process")

    # --------------------------------------------------------
    # 按运动段切分数据
    # --------------------------------------------------------
    n = len(cmd_jnt_pos)

    # ---- joint7 ----
    peak7 = int(np.argmax(cmd_jnt_pos[:, 6])) + 1  # 1-based
    motion_length = n - peak7

    act_pos7 = _slice_around(act_jnt_pos, peak7, motion_length)
    act_trq7 = _slice_around(act_jnt_trq, peak7, motion_length)

    jnt7_peak = peak7

    # ---- joint6 ----
    motion_length6 = 2700
    tmp_peak6_1 = jnt7_peak - 17015
    tmp_peak6_2 = tmp_peak6_1 + 2 * motion_length6

    if arm_idx == 0:
        peak6_1 = _find_peak_around(
            cmd_jnt_pos, tmp_peak6_1, motion_length6, 5, find_max=False
        )
        peak6_2 = _find_peak_around(
            cmd_jnt_pos, tmp_peak6_2, motion_length6, 5, find_max=False
        )
    else:
        peak6_1 = _find_peak_around(
            cmd_jnt_pos, tmp_peak6_1, motion_length6, 5, find_max=True
        )
        peak6_2 = _find_peak_around(
            cmd_jnt_pos, tmp_peak6_2, motion_length6, 5, find_max=True
        )

    act_pos6_1 = _slice_around(act_jnt_pos, peak6_1, motion_length6)
    act_trq6_1 = _slice_around(act_jnt_trq, peak6_1, motion_length6)
    act_pos6_2 = _slice_around(act_jnt_pos, peak6_2, motion_length6)
    act_trq6_2 = _slice_around(act_jnt_trq, peak6_2, motion_length6)

    # ---- joint5：力矩常数辨识用数据 ----
    motion_length5_tc = 1800
    tmp_peak5_tc_1 = jnt7_peak - 49500
    tmp_peak5_tc_2 = tmp_peak5_tc_1 + 2 * motion_length5_tc

    if arm_idx == 0:
        peak5_tc_1 = _find_peak_around(
            cmd_jnt_pos, tmp_peak5_tc_1, motion_length5_tc, 4, find_max=True
        )
        peak5_tc_2 = _find_peak_around(
            cmd_jnt_pos, tmp_peak5_tc_2, motion_length5_tc, 4, find_max=True
        )
    else:
        peak5_tc_1 = _find_peak_around(
            cmd_jnt_pos, tmp_peak5_tc_1, motion_length5_tc, 4, find_max=False
        )
        peak5_tc_2 = _find_peak_around(
            cmd_jnt_pos, tmp_peak5_tc_2, motion_length5_tc, 4, find_max=False
        )

    act_pos5_1 = _slice_around(act_jnt_pos, peak5_tc_1, motion_length5_tc)
    act_trq5_1 = _slice_around(act_jnt_trq, peak5_tc_1, motion_length5_tc)
    act_pos5_2 = _slice_around(act_jnt_pos, peak5_tc_2, motion_length5_tc)
    act_trq5_2 = _slice_around(act_jnt_trq, peak5_tc_2, motion_length5_tc)

    # ---- joint5：重力辨识用数据 ----
    motion_length5_g = 5300
    tmp_peak5_g_1 = jnt7_peak - 36000
    tmp_peak5_g_2 = tmp_peak5_g_1 + 2 * motion_length5_g

    if arm_idx == 0:
        peak5_g_1 = _find_peak_around(
            cmd_jnt_pos, tmp_peak5_g_1, motion_length5_g, 4, find_max=True
        )
        peak5_g_2 = _find_peak_around(
            cmd_jnt_pos, tmp_peak5_g_2, motion_length5_g, 4, find_max=True
        )
    else:
        peak5_g_1 = _find_peak_around(
            cmd_jnt_pos, tmp_peak5_g_1, motion_length5_g, 4, find_max=False
        )
        peak5_g_2 = _find_peak_around(
            cmd_jnt_pos, tmp_peak5_g_2, motion_length5_g, 4, find_max=False
        )

    act_pos5_3 = _slice_around(act_jnt_pos, peak5_g_1, motion_length5_g)
    act_trq5_3 = _slice_around(act_jnt_trq, peak5_g_1, motion_length5_g)
    act_pos5_4 = _slice_around(act_jnt_pos, peak5_g_2, motion_length5_g)
    act_trq5_4 = _slice_around(act_jnt_trq, peak5_g_2, motion_length5_g)

    # MATLAB 中还计算了 rawJntPos5_3 等变量，但后续未使用，这里省略。

    # ---- joint4 ----
    motion_length4 = 1100
    tmp_peak4_1 = jnt7_peak - 55670
    tmp_peak4_2 = tmp_peak4_1 + 2 * motion_length4

    peak4_1 = _find_peak_around(
        cmd_jnt_pos, tmp_peak4_1, motion_length4, 2, find_max=True
    )
    peak4_2 = _find_peak_around(
        cmd_jnt_pos, tmp_peak4_2, motion_length4, 2, find_max=True
    )

    act_pos4_1 = _slice_around(act_jnt_pos, peak4_1, motion_length4)
    act_trq4_1 = _slice_around(act_jnt_trq, peak4_1, motion_length4)
    act_pos4_2 = _slice_around(act_jnt_pos, peak4_2, motion_length4)
    act_trq4_2 = _slice_around(act_jnt_trq, peak4_2, motion_length4)

    # ---- joint3：力矩常数辨识用数据 ----
    motion_length3_tc = 1000
    tmp_peak3_tc_1 = jnt7_peak - 61536
    tmp_peak3_tc_2 = tmp_peak3_tc_1 + 2 * motion_length3_tc

    peak3_tc_1 = _find_peak_around(
        cmd_jnt_pos, tmp_peak3_tc_1, motion_length3_tc, 2, find_max=True
    )
    peak3_tc_2 = _find_peak_around(
        cmd_jnt_pos, tmp_peak3_tc_2, motion_length3_tc, 2, find_max=True
    )

    act_pos3_1 = _slice_around(act_jnt_pos, peak3_tc_1, motion_length3_tc)
    act_trq3_1 = _slice_around(act_jnt_trq, peak3_tc_1, motion_length3_tc)
    act_pos3_2 = _slice_around(act_jnt_pos, peak3_tc_2, motion_length3_tc)
    act_trq3_2 = _slice_around(act_jnt_trq, peak3_tc_2, motion_length3_tc)

    # ---- joint3：重力辨识用数据 ----
    motion_length3_g = 1760
    tmp_peak3_g_1 = jnt7_peak - 64398

    peak3_g = _find_peak_around(
        cmd_jnt_pos, tmp_peak3_g_1, motion_length3_g, 2, find_max=True
    )

    act_pos3 = _slice_around(act_jnt_pos, peak3_g, motion_length3_g)
    act_trq3 = _slice_around(act_jnt_trq, peak3_g, motion_length3_g)

    # ---- joint2 ----
    motion_length2 = 2320
    peak2 = int(np.argmax(cmd_jnt_pos[:, 1])) + 1

    act_pos2 = _slice_around(act_jnt_pos, peak2, motion_length2)
    act_trq2 = _slice_around(act_jnt_trq, peak2, motion_length2)

    # ---- joint1 ----
    motion_length1 = 600
    if arm_idx == 0:
        peak1 = int(np.argmin(cmd_jnt_pos[:, 0])) + 1
    else:
        peak1 = int(np.argmax(cmd_jnt_pos[:, 0])) + 1

    act_pos1 = _slice_around(act_jnt_pos, peak1, motion_length1)
    act_trq1 = _slice_around(act_jnt_trq, peak1, motion_length1)

    # --------------------------------------------------------
    # 参考模型参数和上下界
    # --------------------------------------------------------
    # link1
    m1 = 7.708
    mx1 = -1.28e-3 * m1
    my1 = -0.22e-3 * m1
    mz1 = -124.25e-3 * m1

    # link2
    m2 = 0.996
    mx2 = 60.63e-3 * m2
    my2 = -0.23e-3 * m2
    mz2 = -45.12e-3 * m2

    # link3
    m3 = 0.471
    mx3 = 224.28e-3 * m3
    my3 = -1.06e-3 * m3
    mz3 = 0.48e-3 * m3

    # link4
    m4 = 0.2108
    mx4 = -0.2e-3 * m4
    my4 = 86.3e-3 * m4
    mz4 = -34.9e-3 * m4

    # link5
    m5 = 0.16417
    mx5 = -0.0e-3 * m5
    my5 = 65.1e-3 * m5
    mz5 = -19.5e-3 * m5

    # link6 & link7
    m6 = 0.105 + 0.048
    mx6 = 0e-3 * m6
    my6 = m6 * (-48.6 * 0.189 + 0.043 * 16.3) / m6 * 1e-3
    mz6 = 15.9e-3 * 0.189

    # link8
    m8 = 0.313
    mx8 = 10.7e-3 * m8
    my8 = -1.73e-3 * m8
    mz8 = 65.09e-3 * m8

    # link9
    m9 = 0.072
    mx9 = 128.3e-3 * m9
    my9 = 0.0e-3 * m9
    mz9 = 26.75e-3 * m9

    ref_param = np.array(
        [
            2.7244 * m3
            + 2.7244 * m4
            + 2.7244 * m5
            + 2.7244 * m6
            + 9.8 * mx2
            + 9.8 * mx9,
            9.8 * my2 + 9.8 * my9,
            3.577 * m4 + 3.577 * m5 + 3.577 * m6 + 0.5978 * m9 + 9.8 * mx3 + 9.8 * mx8,
            1.372 * m4 + 1.372 * m5 + 1.372 * m6 + 9.8 * my3 + 9.8 * my8 + 9.8 * mz4,
            mz5 - my4,
            my5 + mz6,
            mx4,
            mx5,
            mx6,
            my6,
        ],
        dtype=float,
    )

    robot_param_buffer = 0.5
    bound_limit = 0.0001
    lb = ref_param.copy()
    ub = ref_param.copy()

    for i in range(10):
        if ref_param[i] > bound_limit:
            lb[i] *= 1.0 - robot_param_buffer
            ub[i] *= 1.0 + robot_param_buffer
        elif ref_param[i] < -bound_limit:
            lb[i] *= 1.0 + robot_param_buffer
            ub[i] *= 1.0 - robot_param_buffer
        else:
            lb[i] = -bound_limit
            ub[i] = bound_limit

    end_buffer = 120

    # --------------------------------------------------------
    # 力矩常数比例辨识
    # --------------------------------------------------------
    trq_const_scale = trqConstIdentify(
        lb,
        ub,
        act_pos6_1,
        act_trq6_1,
        act_pos6_2,
        act_trq6_2,
        act_pos5_1,
        act_trq5_1,
        act_pos5_2,
        act_trq5_2,
        act_pos3_1,
        act_trq3_1,
        act_pos3_2,
        act_trq3_2,
    )

    act_trq5_1[:, 4] *= trq_const_scale[0]
    act_trq5_2[:, 4] *= trq_const_scale[0]
    act_trq5_3[:, 4] *= trq_const_scale[0]
    act_trq5_4[:, 4] *= trq_const_scale[0]
    act_trq6_1[:, 5] *= trq_const_scale[1]
    act_trq6_2[:, 5] *= trq_const_scale[1]

    # --------------------------------------------------------
    # joint1 摩擦
    # --------------------------------------------------------
    _, _, friction1 = _split_forward_back(act_pos1, act_trq1)
    friction1 = _trim_ends(friction1, end_buffer)
    friction_coeff1 = np.mean(np.abs(friction1[:, 0]))

    # --------------------------------------------------------
    # joint7 重力、摩擦、齿槽转矩
    # --------------------------------------------------------
    gravity_pos7, gravity_trq7, friction7 = _split_forward_back(act_pos7, act_trq7)
    friction7 = _trim_ends(friction7, end_buffer)
    gravity_trq7 = _trim_ends(gravity_trq7, end_buffer)
    gravity_pos7 = _trim_ends(gravity_pos7, end_buffer)

    friction_coeff7 = np.mean(np.abs(friction7[:, 6]))

    cog_pos_solution = np.deg2rad(0.25)
    cog_trq_len7 = 1440
    cog_trq_map7 = np.zeros(cog_trq_len7, dtype=float)

    for i in range(len(gravity_trq7)):
        jnt_pos = gravity_pos7[i, 6]
        if 0.0 <= jnt_pos <= 2.0 * np.pi:
            idx = min(
                int(np.floor(jnt_pos / cog_pos_solution)) + 1,
                cog_trq_len7,
            )
            cog_trq_map7[idx - 1] = gravity_trq7[i, 6]

    cog_trq7 = np.zeros(len(gravity_trq7), dtype=float)
    for i in range(len(gravity_trq7)):
        jnt_pos = gravity_pos7[i, 6]
        if jnt_pos > 2.0 * np.pi:
            jnt_pos -= np.floor(jnt_pos / (2.0 * np.pi)) * 2.0 * np.pi
        elif jnt_pos < 0.0:
            jnt_pos -= (np.ceil(jnt_pos / (2.0 * np.pi)) - 1.0) * 2.0 * np.pi

        idx = min(
            max(int(np.floor(jnt_pos / cog_pos_solution)) + 1, 1),
            cog_trq_len7,
        )
        cog_trq7[i] = cog_trq_map7[idx - 1]

    if plt is not None:
        plt.figure()
        plt.plot(gravity_pos7[:, 6], gravity_trq7[:, 6])
        plt.plot(gravity_pos7[:, 6], cog_trq7)
        plt.legend(["measure torque", "calibrate torque"])
        plt.title(
            "Left Arm Joint7 Calibration Gravity"
            if arm_idx == 0
            else "Right Arm Joint7 Calibration Gravity"
        )

    # --------------------------------------------------------
    # joint6 重力、摩擦、齿槽转矩
    # --------------------------------------------------------
    gravity_pos6_1, gravity_trq6_1, friction6 = _split_forward_back(
        act_pos6_1, act_trq6_1
    )
    gravity_pos6_2, gravity_trq6_2, _ = _split_forward_back(act_pos6_2, act_trq6_2)

    gravity_pos6_1 = _trim_ends(gravity_pos6_1, end_buffer)
    gravity_trq6_1 = _trim_ends(gravity_trq6_1, end_buffer)
    gravity_pos6_2 = _trim_ends(gravity_pos6_2, end_buffer)
    gravity_trq6_2 = _trim_ends(gravity_trq6_2, end_buffer)

    # 注意：MATLAB 中 joint6 的摩擦系数没有去掉前后过渡段。
    friction_coeff6 = np.mean(np.abs(friction6[:, 5]))

    reg_pos_data = gravity_pos6_1
    reg_trq_data = gravity_trq6_1

    n_reg6 = len(reg_pos_data)
    reg_mat6_all = np.zeros((n_reg6, 10), dtype=float)
    reg_trq6_all = np.zeros(n_reg6, dtype=float)
    reg_mat6 = np.zeros((n_reg6, 10), dtype=float)
    reg_trq6 = np.zeros(n_reg6, dtype=float)
    reg_pos6 = np.zeros(n_reg6, dtype=float)

    for i in range(n_reg6):
        gra_reg_mat = MasterGravityRegMat(gravity_pos6_1[i])
        gra_reg_mat2 = MasterGravityRegMat(gravity_pos6_2[i])

        reg_mat6_all[i] = gra_reg_mat[4] - gra_reg_mat2[4]
        reg_trq6_all[i] = gravity_trq6_1[i, 5] - gravity_trq6_2[i, 5]

        reg_mat6[i] = gra_reg_mat[4]
        reg_trq6[i] = reg_trq_data[i, 5]
        reg_pos6[i] = reg_pos_data[i, 5]

    lb6 = np.zeros(10, dtype=float)
    ub6 = np.zeros(10, dtype=float)
    lb6[8:10] = lb[8:10]
    ub6[8:10] = ub[8:10]

    gravity_param6 = _lsqlin(reg_mat6_all, reg_trq6_all, lb6, ub6)
    cali_gravity6 = reg_mat6 @ gravity_param6

    if arm_idx == 0:
        cog_pos6_min = np.deg2rad(-140.0)
        cog_pos6_max = np.deg2rad(0.0)
    else:
        cog_pos6_min = np.deg2rad(0.0)
        cog_pos6_max = np.deg2rad(140.0)

    cog_trq_len6 = 560
    cog_trq_map6 = np.zeros(cog_trq_len6, dtype=float)

    for i in range(len(reg_trq6_all)):
        jnt_pos = reg_pos6[i]
        if cog_pos6_min <= jnt_pos <= cog_pos6_max:
            idx = int(np.floor((jnt_pos - cog_pos6_min) / cog_pos_solution)) + 1
            idx = min(max(idx, 1), cog_trq_len6)
            cog_trq_map6[idx - 1] = reg_trq6[i] - cali_gravity6[i]

    cog_trq6 = np.zeros(len(reg_trq6_all), dtype=float)
    for i in range(len(reg_trq6_all)):
        jnt_pos = reg_pos6[i]
        idx = int(np.floor((jnt_pos - cog_pos6_min) / cog_pos_solution)) + 1
        idx = min(max(idx, 1), cog_trq_len6)
        cog_trq6[i] = cog_trq_map6[idx - 1]

    if plt is not None:
        plt.figure()
        plt.plot(reg_pos6, reg_trq6)
        plt.plot(reg_pos6, cali_gravity6)
        plt.plot(reg_pos6, cali_gravity6 + cog_trq6)
        plt.legend(["measure torque", "calibrate torque", "cali cogtrq"])
        plt.title(
            "Left Arm Joint6 Calibration Gravity"
            if arm_idx == 0
            else "Right Arm Joint6 Calibration Gravity"
        )

    # --------------------------------------------------------
    # joint5 重力、弹簧、摩擦、齿槽转矩
    # --------------------------------------------------------
    gravity_pos5, gravity_trq5, friction5 = _split_forward_back(act_pos5_4, act_trq5_4)
    gravity_pos5_2, gravity_trq5_2, _ = _split_forward_back(act_pos5_3, act_trq5_3)

    gravity_pos5 = _trim_ends(gravity_pos5, end_buffer)
    gravity_trq5 = _trim_ends(gravity_trq5, end_buffer)
    friction5 = _trim_ends(friction5, end_buffer)
    gravity_pos5_2 = _trim_ends(gravity_pos5_2, end_buffer)
    gravity_trq5_2 = _trim_ends(gravity_trq5_2, end_buffer)

    friction_coeff5 = np.mean(np.abs(friction5[:, 4]))

    n_reg5 = len(gravity_pos5)
    reg_mat5 = np.zeros((n_reg5, 10), dtype=float)
    reg_trq5 = np.zeros(n_reg5, dtype=float)

    for i in range(n_reg5):
        gra_reg_mat = MasterGravityRegMat(gravity_pos5[i])
        gra_reg_mat2 = MasterGravityRegMat(gravity_pos5_2[i])

        reg_mat5[i] = gra_reg_mat[3] - gra_reg_mat2[3]
        reg_trq5[i] = gravity_trq5[i, 4] - gravity_trq5_2[i, 4]

    lb5 = gravity_param6.copy()
    ub5 = gravity_param6.copy()
    lb5[5] = lb[5]
    lb5[7] = lb[7]
    ub5[5] = ub[5]
    ub5[7] = ub[7]

    gravity_param5 = _lsqlin(reg_mat5, reg_trq5, lb5, ub5)
    cali_trq5 = reg_mat5 @ gravity_param5

    # 弹簧参数拟合
    reg_mat5_spring = np.zeros((n_reg5, 10), dtype=float)
    reg_trq5_spring = np.zeros(n_reg5, dtype=float)
    reg_pos5 = np.zeros(n_reg5, dtype=float)

    for i in range(n_reg5):
        gra_reg_mat = MasterGravityRegMat(gravity_pos5[i])

        reg_mat5_spring[i] = gra_reg_mat[3]
        reg_trq5_spring[i] = gravity_trq5[i, 4]
        reg_pos5[i] = gravity_pos5[i, 4]

    gravity_trq5_fit = reg_mat5_spring @ gravity_param5
    spring_trq5 = reg_trq5_spring - gravity_trq5_fit
    spring5_coeffs = np.polyfit(reg_pos5, spring_trq5, 7)
    spring5_fit_trq = np.polyval(spring5_coeffs, reg_pos5)
    cali_gravity5 = spring5_fit_trq + gravity_trq5_fit

    cog_pos5_min = np.deg2rad(47.0)
    cog_pos5_max = np.deg2rad(313.0)
    cog_trq_len5 = 1064
    cog_trq_map5 = np.zeros(cog_trq_len5, dtype=float)

    for i in range(len(reg_trq5_spring)):
        jnt_pos = reg_pos5[i]
        if cog_pos5_min <= jnt_pos <= cog_pos5_max:
            idx = int(np.floor((jnt_pos - cog_pos5_min) / cog_pos_solution)) + 1
            idx = min(max(idx, 1), cog_trq_len5)
            cog_trq_map5[idx - 1] = reg_trq5_spring[i] - cali_gravity5[i]

    cog_trq5 = np.zeros(len(reg_trq5_spring), dtype=float)
    for i in range(len(reg_trq5_spring)):
        jnt_pos = reg_pos5[i]
        idx = int(np.floor((jnt_pos - cog_pos5_min) / cog_pos_solution)) + 1
        idx = min(max(idx, 1), cog_trq_len5)
        cog_trq5[i] = cog_trq_map5[idx - 1]

    if plt is not None:
        plt.figure()
        plt.plot(reg_pos5, reg_trq5_spring)
        plt.plot(reg_pos5, cali_gravity5)
        plt.plot(reg_pos5, cali_gravity5 + cog_trq5)
        plt.legend(["measure torque", "calibrate torque", "cog trq"])
        plt.title(
            "Left Arm Joint5 Calibration Gravity and Spring"
            if arm_idx == 0
            else "Right Arm Joint5 Calibration Gravity and Spring"
        )

    # --------------------------------------------------------
    # joint4 重力
    # --------------------------------------------------------
    gravity_pos4_1, gravity_trq4_1, _ = _split_forward_back(act_pos4_1, act_trq4_1)
    gravity_pos4_2, gravity_trq4_2, _ = _split_forward_back(act_pos4_2, act_trq4_2)

    gravity_pos4_1 = _trim_ends(gravity_pos4_1, end_buffer)
    gravity_trq4_1 = _trim_ends(gravity_trq4_1, end_buffer)
    gravity_pos4_2 = _trim_ends(gravity_pos4_2, end_buffer)
    gravity_trq4_2 = _trim_ends(gravity_trq4_2, end_buffer)

    friction_coeff4 = 0.0

    n_reg4 = len(gravity_pos4_1)
    reg_mat4 = np.zeros((n_reg4, 10), dtype=float)
    reg_trq4 = np.zeros(n_reg4, dtype=float)

    for i in range(n_reg4):
        gra_reg_mat = MasterGravityRegMat(gravity_pos4_1[i])
        gra_reg_mat2 = MasterGravityRegMat(gravity_pos4_2[i])

        reg_mat4[i] = gra_reg_mat[1] - gra_reg_mat2[1]
        reg_trq4[i] = gravity_trq4_1[i, 2] - gravity_trq4_2[i, 2]

    lb4 = gravity_param5.copy()
    ub4 = gravity_param5.copy()
    lb4[4] = lb[4]
    lb4[6] = lb[6]
    ub4[4] = ub[4]
    ub4[6] = ub[6]

    gravity_param4 = _lsqlin(reg_mat4, reg_trq4, lb4, ub4)
    cali_trq4 = reg_mat4 @ gravity_param4

    if plt is not None:
        plt.figure()
        plt.plot(reg_trq4)
        plt.plot(cali_trq4)
        plt.legend(["measure torque", "calibrate torque"])
        plt.title(
            "Left Arm Joint4 Calibration Gravity"
            if arm_idx == 0
            else "Right Arm Joint4 Calibration Gravity"
        )

    # --------------------------------------------------------
    # joint3 重力、弹簧、摩擦
    # --------------------------------------------------------
    gravity_pos3, gravity_trq3, friction3 = _split_forward_back(act_pos3, act_trq3)

    gravity_pos3 = _trim_ends(gravity_pos3, end_buffer)
    gravity_trq3 = _trim_ends(gravity_trq3, end_buffer)
    friction3 = _trim_ends(friction3, end_buffer)

    friction_coeff3 = np.mean(np.abs(friction3[:, 2]))

    n_reg3 = len(gravity_pos3)
    reg_mat3 = np.zeros((n_reg3, 10), dtype=float)
    reg_trq3 = np.zeros(n_reg3, dtype=float)
    reg_pos3 = np.zeros(n_reg3, dtype=float)

    for i in range(n_reg3):
        gra_reg_mat = MasterGravityRegMat(gravity_pos3[i])

        reg_mat3[i] = gra_reg_mat[1]
        reg_trq3[i] = gravity_trq3[i, 2]
        reg_pos3[i] = gravity_pos3[i, 2]

    gravity_trq3_fit = reg_mat3 @ gravity_param4
    spring_trq3 = reg_trq3 - gravity_trq3_fit
    jnt3_coeffs = np.polyfit(reg_pos3, spring_trq3, 7)
    jnt3_fit_trq = np.polyval(jnt3_coeffs, reg_pos3)
    cali_gravity3 = jnt3_fit_trq + gravity_trq3_fit

    if plt is not None:
        plt.figure()
        plt.plot(reg_pos3, reg_trq3)
        plt.plot(reg_pos3, cali_gravity3)
        plt.legend(["measure torque", "calibrate torque"])
        plt.title(
            "Left Arm Joint3 Calibration Gravity"
            if arm_idx == 0
            else "Right Arm Joint3 Calibration Gravity"
        )

    # --------------------------------------------------------
    # joint2 重力、弹簧、摩擦
    # --------------------------------------------------------
    gravity_pos2, gravity_trq2, friction2 = _split_forward_back(act_pos2, act_trq2)

    gravity_pos2 = _trim_ends(gravity_pos2, end_buffer)
    gravity_trq2 = _trim_ends(gravity_trq2, end_buffer)
    friction2 = _trim_ends(friction2, end_buffer)

    friction_coeff2 = np.mean(np.abs(friction2[:, 1]))

    n_reg2 = len(gravity_pos2)
    reg_mat2 = np.zeros((n_reg2, 10), dtype=float)
    reg_trq2 = np.zeros(n_reg2, dtype=float)
    reg_pos2 = np.zeros(n_reg2, dtype=float)

    for i in range(n_reg2):
        gra_reg_mat = MasterGravityRegMat(gravity_pos2[i])

        reg_mat2[i] = gra_reg_mat[0]
        reg_trq2[i] = gravity_trq2[i, 1]
        reg_pos2[i] = gravity_pos2[i, 1]

    gravity_trq2_fit = reg_mat2 @ gravity_param4
    spring_trq2 = reg_trq2 - gravity_trq2_fit
    jnt2_coeffs = np.polyfit(reg_pos2, spring_trq2, 7)
    jnt2_fit_trq = np.polyval(jnt2_coeffs, reg_pos2)
    cali_gravity2 = jnt2_fit_trq + gravity_trq2_fit

    if plt is not None:
        plt.figure()
        plt.plot(reg_pos2, reg_trq2)
        plt.plot(reg_pos2, cali_gravity2)
        plt.legend(["measure torque", "calibrate torque"])
        plt.title(
            "Left Arm Joint2 Calibration Gravity"
            if arm_idx == 0
            else "Right Arm Joint2 Calibration Gravity"
        )
        plt.show(block=False)

    # --------------------------------------------------------
    # 输出
    # --------------------------------------------------------
    gravity_params = np.concatenate(
        [
            gravity_param4[4:10],
            np.asarray(jnt2_coeffs, dtype=float),
            np.asarray(jnt3_coeffs, dtype=float),
            np.asarray(spring5_coeffs, dtype=float),
        ]
    )

    friction_params = np.array(
        [
            friction_coeff1,
            friction_coeff2,
            friction_coeff3,
            friction_coeff4,
            friction_coeff5,
            friction_coeff6,
            friction_coeff7,
        ],
        dtype=float,
    )

    return (
        gravity_params,
        friction_params,
        np.asarray(trq_const_scale, dtype=float),
        cog_trq_map5,
        cog_trq_map6,
        cog_trq_map7,
    )


def masterGraCali(m_graCaliData):
    """对左右手记录数据执行重力参数辨识。

    Parameters
    ----------
    m_graCaliData : ST_graCaliData
        包含左右手记录数据：
        m_leftCmdJntPos / m_leftCurJntPos / m_leftJntTrq
        m_rightCmdJntPos / m_rightCurJntPos / m_rightJntTrq

    Returns
    -------
    result : ST_RMGraCaliData
        可直接用于 UI 下发的重力配平参数结构体。
    """

    from masterRobotInterface import ST_RMGraCaliData

    # 快照数据，避免辨识过程中被清空或修改。
    left_cmd = list(m_graCaliData.m_leftCmdJntPos)
    left_cur = list(m_graCaliData.m_leftCurJntPos)
    left_trq = list(m_graCaliData.m_leftJntTrq)
    right_cmd = list(m_graCaliData.m_rightCmdJntPos)
    right_cur = list(m_graCaliData.m_rightCurJntPos)
    right_trq = list(m_graCaliData.m_rightJntTrq)

    lengths = [
        len(left_cmd),
        len(left_cur),
        len(left_trq),
        len(right_cmd),
        len(right_cur),
        len(right_trq),
    ]

    if min(lengths) == 0:
        raise ValueError("No recorded gravity calibration data.")

    if len(set(lengths)) != 1:
        raise ValueError(f"Recorded data lengths mismatch: {lengths}.")

    left_result = Identification(0, left_cmd, left_cur, left_trq)
    right_result = Identification(1, right_cmd, right_cur, right_trq)

    (
        gravity_left,
        friction_left,
        scale_left,
        cog5_left,
        cog6_left,
        cog7_left,
    ) = left_result

    (
        gravity_right,
        friction_right,
        scale_right,
        cog5_right,
        cog6_right,
        cog7_right,
    ) = right_result

    result = ST_RMGraCaliData()

    # g_gravityParam: [2, 30]
    for i in range(30):
        result.g_gravityParam[0][i] = float(gravity_left[i])
        result.g_gravityParam[1][i] = float(gravity_right[i])

    # g_jntFrictionParam: [2, 7]
    for i in range(7):
        result.g_jntFrictionParam[0][i] = float(friction_left[i])
        result.g_jntFrictionParam[1][i] = float(friction_right[i])

    # g_trqConstScale: [2, 2]
    for i in range(2):
        result.g_trqConstScale[0][i] = float(scale_left[i])
        result.g_trqConstScale[1][i] = float(scale_right[i])

    # g_left/rightJnt5CogTrq: [1064]
    for i in range(1064):
        result.g_leftJnt5CogTrq[i] = float(cog5_left[i])
        result.g_rightJnt5CogTrq[i] = float(cog5_right[i])

    # g_left/rightJnt6CogTrq: [560]
    for i in range(560):
        result.g_leftJnt6CogTrq[i] = float(cog6_left[i])
        result.g_rightJnt6CogTrq[i] = float(cog6_right[i])

    # g_left/rightJnt7CogTrq: [1440]
    for i in range(1440):
        result.g_leftJnt7CogTrq[i] = float(cog7_left[i])
        result.g_rightJnt7CogTrq[i] = float(cog7_right[i])

    return result


def MasterGravity_Lagrange():
    """MATLAB 符号推导结果（文档用途）。

    该函数保留原始 MATLAB 文件中的 DH 参数和推导得到的重力力矩表达式。
    实际数值计算请使用 MasterGravityRegMat()。
    """

    return {
        "deg90": np.deg2rad(90.0),
        "d1": 0.261,
        "a3": 0.278,
        "a4": 0.365,
        "d4": 0.140,
        "a9": 0.061,
        "t2": (
            "cos(q2)*(2.7244*m3 + 2.7244*m4 + 2.7244*m5 + 2.7244*m6 "
            "+ 9.8*mx2 + 9.8*mx9) "
            "- sin(q2)*(9.8*my2 + 9.8*my9)"
        ),
        "t8": (
            "cos(q8)*(3.577*m4 + 3.577*m5 + 3.577*m6 + 0.5978*m9 "
            "+ 9.8*mx3 + 9.8*mx8) "
            "- sin(q8)*(1.372*m4 + 1.372*m5 + 1.372*m6 "
            "+ 9.8*my3 + 9.8*my8 + 9.8*mz4) "
            "- mx6*(9.8*cos(q8)*sin(q4)*sin(q6) "
            "+ 9.8*cos(q6)*sin(q5)*sin(q8) "
            "- 9.8*cos(q4)*cos(q5)*cos(q6)*cos(q8)) "
            "- mx5*(9.8*sin(q5)*sin(q8) "
            "- 9.8*cos(q4)*cos(q5)*cos(q8)) "
            "- (my5+mz6)*(9.8*cos(q5)*sin(q8) "
            "+ 9.8*cos(q4)*cos(q8)*sin(q5)) "
            "- my6*(9.8*cos(q6)*cos(q8)*sin(q4) "
            "- 9.8*sin(q5)*sin(q6)*sin(q8) "
            "+ 9.8*cos(q4)*cos(q5)*cos(q8)*sin(q6)) "
            "+ 9.8*mx4*cos(q4)*cos(q8) "
            "+ 9.8*(mz5-my4)*cos(q8)*sin(q4)"
        ),
        "t4": (
            "9.8*(mz5-my4)*cos(q4)*sin(q8) "
            "- 9.8*my6*sin(q8)*(cos(q4)*cos(q6) "
            "- cos(q5)*sin(q4)*sin(q6)) "
            "- 9.8*mx6*sin(q8)*(cos(q4)*sin(q6) "
            "+ cos(q5)*cos(q6)*sin(q4)) "
            "- 9.8*mx4*sin(q4)*sin(q8) "
            "- 9.8*mx5*cos(q5)*sin(q4)*sin(q8) "
            "+ 9.8*(my5+mz6)*sin(q4)*sin(q5)*sin(q8)"
        ),
        "t5": (
            "mx6*(9.8*cos(q5)*cos(q6)*cos(q8) "
            "- 9.8*cos(q4)*cos(q6)*sin(q5)*sin(q8)) "
            "- my6*(9.8*cos(q5)*cos(q8)*sin(q6) "
            "- 9.8*cos(q4)*sin(q5)*sin(q6)*sin(q8)) "
            "+ mx5*(9.8*cos(q5)*cos(q8) "
            "- 9.8*cos(q4)*sin(q5)*sin(q8)) "
            "- (my5+mz6)*(9.8*cos(q8)*sin(q5) "
            "+ 9.8*cos(q4)*cos(q5)*sin(q8))"
        ),
        "t6": (
            "-my6*(9.8*cos(q6)*cos(q8)*sin(q5) "
            "- 9.8*sin(q4)*sin(q6)*sin(q8) "
            "+ 9.8*cos(q4)*cos(q5)*cos(q6)*sin(q8)) "
            "- mx6*(9.8*cos(q6)*sin(q4)*sin(q8) "
            "+ 9.8*cos(q8)*sin(q5)*sin(q6) "
            "+ 9.8*cos(q4)*cos(q5)*sin(q6)*sin(q8))"
        ),
    }


def MasterGravityRegMat(q):
    """主手重力回归矩阵。

    Parameters
    ----------
    q : array_like, shape (7,)
        关节位置 [q1, q2, q3, q4, q5, q6, q7]，单位 rad。

    Returns
    -------
    regMat : ndarray, shape (5, 10)
        第 1~5 行分别对应 joint2、joint8(joint3)、joint4、joint5、joint6。
    """

    q = np.asarray(q, dtype=float).reshape(-1)
    if q.size < 7:
        raise ValueError("q must contain 7 joint positions.")

    q2 = q[1]
    q8 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]

    c2, s2 = np.cos(q2), np.sin(q2)
    c4, s4 = np.cos(q4), np.sin(q4)
    c5, s5 = np.cos(q5), np.sin(q5)
    c6, s6 = np.cos(q6), np.sin(q6)
    c8, s8 = np.cos(q8), np.sin(q8)

    reg_mat = np.zeros((5, 10), dtype=float)

    # joint2
    reg_mat[0, 0] = c2
    reg_mat[0, 1] = -s2

    # joint8 / joint3
    reg_mat[1, 2] = c8
    reg_mat[1, 3] = -s8
    reg_mat[1, 4] = 9.8 * c8 * s4
    reg_mat[1, 5] = -(9.8 * c5 * s8 + 9.8 * c4 * c8 * s5)
    reg_mat[1, 6] = 9.8 * c4 * c8
    reg_mat[1, 7] = -(9.8 * s5 * s8 - 9.8 * c4 * c5 * c8)
    reg_mat[1, 8] = -(9.8 * c8 * s4 * s6 + 9.8 * c6 * s5 * s8 - 9.8 * c4 * c5 * c6 * c8)
    reg_mat[1, 9] = -(9.8 * c6 * c8 * s4 - 9.8 * s5 * s6 * s8 + 9.8 * c4 * c5 * c8 * s6)

    # joint4
    reg_mat[2, 4] = 9.8 * c4 * s8
    reg_mat[2, 5] = 9.8 * s4 * s5 * s8
    reg_mat[2, 6] = -9.8 * s4 * s8
    reg_mat[2, 7] = -9.8 * c5 * s4 * s8
    reg_mat[2, 8] = -9.8 * s8 * (c4 * s6 + c5 * c6 * s4)
    reg_mat[2, 9] = -9.8 * s8 * (c4 * c6 - c5 * s4 * s6)

    # joint5
    reg_mat[3, 5] = -(9.8 * c8 * s5 + 9.8 * c4 * c5 * s8)
    reg_mat[3, 7] = 9.8 * c5 * c8 - 9.8 * c4 * s5 * s8
    reg_mat[3, 8] = 9.8 * c5 * c6 * c8 - 9.8 * c4 * c6 * s5 * s8
    reg_mat[3, 9] = -(9.8 * c5 * c8 * s6 - 9.8 * c4 * s5 * s6 * s8)

    # joint6
    reg_mat[4, 8] = -(9.8 * c6 * s4 * s8 + 9.8 * c8 * s5 * s6 + 9.8 * c4 * c5 * s6 * s8)
    reg_mat[4, 9] = -(9.8 * c6 * c8 * s5 - 9.8 * s4 * s6 * s8 + 9.8 * c4 * c5 * c6 * s8)

    return reg_mat


def trqConstIdentify(
    lb,
    ub,
    actJntPos6_1,
    actJntTrq6_1,
    actJntPos6_2,
    actJntTrq6_2,
    actJntPos5_1,
    actJntTrq5_1,
    actJntPos5_2,
    actJntTrq5_2,
    actJntPos3_1,
    actJntTrq3_1,
    actJntPos3_2,
    actJntTrq3_2,
):
    """辨识 joint5/joint6 的力矩常数比例。"""

    end_buffer = 150

    # ---- joint6 ----
    pos6_1, trq6_1, _ = _split_forward_back(
        np.asarray(actJntPos6_1, dtype=float),
        np.asarray(actJntTrq6_1, dtype=float),
    )
    pos6_2, trq6_2, _ = _split_forward_back(
        np.asarray(actJntPos6_2, dtype=float),
        np.asarray(actJntTrq6_2, dtype=float),
    )

    pos6_1 = _trim_ends(pos6_1, end_buffer)
    trq6_1 = _trim_ends(trq6_1, end_buffer)
    pos6_2 = _trim_ends(pos6_2, end_buffer)
    trq6_2 = _trim_ends(trq6_2, end_buffer)

    reg_mat6, reg_trq6 = _build_joint_regression(
        pos6_1, trq6_1, pos6_2, trq6_2, row_index=4, trq_col=5
    )

    lb6 = np.zeros(10, dtype=float)
    ub6 = np.zeros(10, dtype=float)
    lb6[8:10] = lb[8:10]
    ub6[8:10] = ub[8:10]

    gravity_param6 = _lsqlin(reg_mat6, reg_trq6, lb6, ub6)

    # ---- joint5 ----
    pos5_1, trq5_1, _ = _split_forward_back(
        np.asarray(actJntPos5_1, dtype=float),
        np.asarray(actJntTrq5_1, dtype=float),
    )
    pos5_2, trq5_2, _ = _split_forward_back(
        np.asarray(actJntPos5_2, dtype=float),
        np.asarray(actJntTrq5_2, dtype=float),
    )

    pos5_1 = _trim_ends(pos5_1, end_buffer)
    trq5_1 = _trim_ends(trq5_1, end_buffer)
    pos5_2 = _trim_ends(pos5_2, end_buffer)
    trq5_2 = _trim_ends(trq5_2, end_buffer)

    reg_mat5, reg_trq5 = _build_joint_regression(
        pos5_1, trq5_1, pos5_2, trq5_2, row_index=3, trq_col=4
    )

    lb5 = np.zeros(10, dtype=float)
    ub5 = np.zeros(10, dtype=float)
    lb5[8:10] = lb[8:10]
    ub5[8:10] = ub[8:10]

    gravity_param5 = _lsqlin(reg_mat5, reg_trq5, lb5, ub5)

    # ---- joint3 ----
    pos3_1, trq3_1, _ = _split_forward_back(
        np.asarray(actJntPos3_1, dtype=float),
        np.asarray(actJntTrq3_1, dtype=float),
    )
    pos3_2, trq3_2, _ = _split_forward_back(
        np.asarray(actJntPos3_2, dtype=float),
        np.asarray(actJntTrq3_2, dtype=float),
    )

    pos3_1 = _trim_ends(pos3_1, end_buffer)
    trq3_1 = _trim_ends(trq3_1, end_buffer)
    pos3_2 = _trim_ends(pos3_2, end_buffer)
    trq3_2 = _trim_ends(trq3_2, end_buffer)

    reg_mat3, reg_trq3 = _build_joint_regression(
        pos3_1, trq3_1, pos3_2, trq3_2, row_index=1, trq_col=2
    )

    lb3 = np.zeros(10, dtype=float)
    ub3 = np.zeros(10, dtype=float)
    lb3[8:10] = lb[8:10]
    ub3[8:10] = ub[8:10]

    gravity_param3 = _lsqlin(reg_mat3, reg_trq3, lb3, ub3)

    return np.array(
        [
            gravity_param3[9] / gravity_param5[9],
            gravity_param3[9] / gravity_param6[9],
        ],
        dtype=float,
    )


# ============================================================
# 内部辅助函数
# ============================================================
def _lsqlin(a_mat, b_vec, lb, ub):
    """带上下界的线性最小二乘，对应 MATLAB lsqlin。

    MATLAB lsqlin 允许 lb == ub（固定变量）；scipy.lsq_linear 要求
    下界严格小于上界，因此这里先分离固定变量，再对自由变量求解。
    """
    a_mat = np.asarray(a_mat, dtype=float)
    b_vec = np.asarray(b_vec, dtype=float).reshape(-1)
    lb = np.asarray(lb, dtype=float).reshape(-1)
    ub = np.asarray(ub, dtype=float).reshape(-1)

    if np.any(ub < lb):
        raise ValueError("Invalid bounds: upper bound is smaller than lower bound.")

    n_vars = a_mat.shape[1]
    x = np.zeros(n_vars, dtype=float)

    fixed = ub <= lb
    free = ~fixed

    if np.any(fixed):
        x[fixed] = lb[fixed]

    if not np.any(free):
        return x

    rhs = b_vec - a_mat[:, fixed] @ x[fixed]
    a_free = a_mat[:, free]
    lb_free = lb[free]
    ub_free = ub[free]

    try:
        result = lsq_linear(
            a_free,
            rhs,
            bounds=(lb_free, ub_free),
            method="bvls",
            tol=1e-12,
        )
    except Exception:
        result = lsq_linear(
            a_free,
            rhs,
            bounds=(lb_free, ub_free),
            method="trf",
            tol=1e-12,
        )

    x[free] = result.x
    return x


def _split_forward_back(pos, trq):
    """把往复运动数据拆成前半程和后半程，并计算平均重力和摩擦力矩。"""
    pos = np.asarray(pos, dtype=float)
    trq = np.asarray(trq, dtype=float)

    n = len(trq)
    half_f = n // 2
    start_back = int(np.ceil(n / 2.0))

    forth_trq = trq[:half_f, :]
    back_trq = np.flip(trq[start_back:, :], axis=0)

    m = min(len(forth_trq), len(back_trq))
    if m == 0:
        raise ValueError("Segment is too short for forward/back splitting.")

    forth_trq = forth_trq[:m, :]
    back_trq = back_trq[:m, :]
    gravity_pos = pos[:half_f, :][:m, :]

    gravity_trq = (forth_trq + back_trq) / 2.0
    friction_trq = (forth_trq - back_trq) / 2.0

    return gravity_pos, gravity_trq, friction_trq


def _trim_ends(data, end_buffer):
    """对应 MATLAB 的 data(endBuffer:end-endBuffer, :)。"""
    data = np.asarray(data, dtype=float)
    start = end_buffer - 1
    end = len(data) - end_buffer

    if start < 0 or end <= start:
        raise ValueError(
            f"Segment length {len(data)} is too short for " f"endBuffer={end_buffer}."
        )

    return data[start:end, :]


def _slice_around(data, peak_1based, motion_length):
    """对应 MATLAB 的 data(peak-motionLength:peak+motionLength, :)。"""
    start0 = peak_1based - motion_length - 1
    end0 = peak_1based + motion_length

    if start0 < 0 or end0 > len(data):
        raise ValueError(f"Slice [{start0}:{end0}] is outside data length {len(data)}.")

    return data[start0:end0, :]


def _find_peak_around(cmd_jnt_pos, tmp_peak_1based, motion_length, jnt_index, find_max):
    """在指定关节的局部区间内寻找峰值，返回 1-based 全局索引。"""
    start1 = tmp_peak_1based - motion_length
    start0 = start1 - 1
    end0 = start0 + 2 * motion_length + 1

    if start0 < 0 or end0 > len(cmd_jnt_pos):
        raise ValueError(
            f"Peak search slice [{start0}:{end0}] is outside "
            f"data length {len(cmd_jnt_pos)}."
        )

    segment = cmd_jnt_pos[start0:end0, jnt_index]

    if find_max:
        local0 = int(np.argmax(segment))
    else:
        local0 = int(np.argmin(segment))

    return start1 + local0


def _build_joint_regression(pos1, trq1, pos2, trq2, row_index, trq_col):
    """构建 trqConstIdentify 中使用的差分回归矩阵。"""
    n = len(pos1)
    reg_mat = np.zeros((n, 10), dtype=float)
    reg_trq = np.zeros(n, dtype=float)

    for i in range(n):
        reg_mat1 = MasterGravityRegMat(pos1[i])
        reg_mat2 = MasterGravityRegMat(pos2[i])

        reg_mat[i] = reg_mat1[row_index] - reg_mat2[row_index]
        reg_trq[i] = trq1[i, trq_col] - trq2[i, trq_col]

    return reg_mat, reg_trq


def _get_plt(show_plots):
    """按需导入 matplotlib。"""
    if not show_plots:
        return None

    try:
        import matplotlib.pyplot as plt

        return plt
    except ImportError:
        print("[MasterGraCali] matplotlib is not installed; " "skip diagnostic plots.")
        return None
