"""主手（old master）重力标定 MATLAB 代码的 Python 版本。

由以下 MATLAB 文件转换而来：
    Identification.m
    MasterGravityRegMat.m
    springTorque2.m
    springTorque3.m
    springTorque5.m

对外接口：
    Identification(armIdx, cmdJntPos, actJntPos, actJntTrq, show_plots=True)
    masterGraCali(m_graCaliData)

标定算法本身保持 MATLAB 原始实现，不做算法逻辑上的修改。

绘图与 masterGraCali_NewMaster 保持一致：每只主手 6 张诊断图，共 12 张。
"""

import numpy as np
from scipy.signal import butter, filtfilt
from scipy.optimize import lsq_linear

# ============================================================
# 对外函数
# ============================================================
def Identification(armIdx, cmdJntPos, actJntPos, actJntTrq, show_plots=True):
    """主手重力、摩擦辨识（old master MATLAB 版本转换）。

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
        是否使用 matplotlib 显示诊断图，默认 True。

    Returns
    -------
    gravityParams : ndarray, shape (30,)
    frictionParams : ndarray, shape (14,)
        每个关节为 (A, B) 两个摩擦参数。
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
    if arm_idx not in (0, 1):
        raise ValueError("armIdx must be 0 (left) or 1 (right).")

    # --------------------------------------------------------
    # 滤波
    # --------------------------------------------------------
    decimate_rate = 1
    fs = 100.0
    fc = 5.0
    b, a = butter(1, 2.0 * fc / fs)

    try:
        act_jnt_pos = filtfilt(b, a, act_jnt_pos_raw, axis=0)
        act_jnt_trq = filtfilt(b, a, act_jnt_trq_raw, axis=0)
    except ValueError as exc:
        print(f"[MasterGraCaliOld] filtfilt failed: {exc}; use raw data.")
        act_jnt_pos = act_jnt_pos_raw.copy()
        act_jnt_trq = act_jnt_trq_raw.copy()

    plt = _get_plt(show_plots)

    # --------------------------------------------------------
    # 按各关节运动段切分数据
    # --------------------------------------------------------
    # joint6
    search_jnt_idx = 5
    motion_length = 2000
    if arm_idx == 0:
        peak_idx = int(np.argmin(cmd_jnt_pos[:, search_jnt_idx])) + 1
    else:
        peak_idx = int(np.argmax(cmd_jnt_pos[:, search_jnt_idx])) + 1

    act_jnt_pos6 = _slice_around(act_jnt_pos, peak_idx, motion_length)
    act_jnt_trq6 = _slice_around(act_jnt_trq, peak_idx, motion_length)

    # joint5
    search_jnt_idx = 4
    motion_length = 5360
    if arm_idx == 0:
        peak_idx = int(np.argmax(cmd_jnt_pos[:, search_jnt_idx])) + 1
    else:
        peak_idx = int(np.argmin(cmd_jnt_pos[:, search_jnt_idx])) + 1

    act_jnt_pos5 = _slice_around(act_jnt_pos, peak_idx, motion_length)
    act_jnt_trq5 = _slice_around(act_jnt_trq, peak_idx, motion_length)

    # joint3
    search_jnt_idx = 2
    motion_length3 = 2400
    peak_idx3 = int(np.argmax(cmd_jnt_pos[:, search_jnt_idx])) + 1

    act_jnt_pos3 = _slice_around(act_jnt_pos, peak_idx3, motion_length3)
    act_jnt_trq3 = _slice_around(act_jnt_trq, peak_idx3, motion_length3)

    # joint3 motion for joint5 calibration
    motion_length = 1200
    segment = cmd_jnt_pos[: peak_idx3 - motion_length3, search_jnt_idx]
    peak_idx = int(np.argmax(segment)) + 1

    act_jnt_pos3_1 = _slice_around(act_jnt_pos, peak_idx, motion_length)
    act_jnt_trq3_1 = _slice_around(act_jnt_trq, peak_idx, motion_length)

    segment1 = cmd_jnt_pos[: peak_idx - 10, search_jnt_idx]
    max_val1 = float(np.max(segment1))
    peak_idx1 = int(np.argmax(segment1)) + 1

    segment2 = cmd_jnt_pos[peak_idx + 9 : peak_idx3 - motion_length3, search_jnt_idx]
    max_val2 = float(np.max(segment2))
    peak_idx2 = int(np.argmax(segment2)) + 1
    peak_idx2 = peak_idx + 9 + peak_idx2

    if max_val1 > max_val2:
        peak_idx2 = peak_idx1

    act_jnt_pos3_2 = _slice_around(act_jnt_pos, peak_idx2, motion_length)
    act_jnt_trq3_2 = _slice_around(act_jnt_trq, peak_idx2, motion_length)

    # joint3 motion for joint4 calibration
    peak_idx4 = min(peak_idx, peak_idx2)
    motion_length = 1200

    segment = cmd_jnt_pos[: peak_idx4 - 1200, search_jnt_idx]
    peak_idx = int(np.argmax(segment)) + 1

    act_jnt_pos3_3 = _slice_around(act_jnt_pos, peak_idx, motion_length)
    act_jnt_trq3_3 = _slice_around(act_jnt_trq, peak_idx, motion_length)

    segment1 = cmd_jnt_pos[: peak_idx - 10, search_jnt_idx]
    max_val1 = float(np.max(segment1))
    peak_idx1 = int(np.argmax(segment1)) + 1

    segment2 = cmd_jnt_pos[peak_idx + 9 : peak_idx4 - 1200, search_jnt_idx]
    max_val2 = float(np.max(segment2))
    peak_idx2 = int(np.argmax(segment2)) + 1
    peak_idx2 = peak_idx + 9 + peak_idx2

    if max_val1 > max_val2:
        peak_idx2 = peak_idx1

    act_jnt_pos3_4 = _slice_around(act_jnt_pos, peak_idx2, motion_length)
    act_jnt_trq3_4 = _slice_around(act_jnt_trq, peak_idx2, motion_length)

    # joint2
    search_jnt_idx = 1
    motion_length = 2400
    peak_idx = int(np.argmax(cmd_jnt_pos[:, search_jnt_idx])) + 1

    act_jnt_pos2 = _slice_around(act_jnt_pos, peak_idx, motion_length)
    act_jnt_trq2 = _slice_around(act_jnt_trq, peak_idx, motion_length)

    # joint1/7
    search_jnt_idx = 6
    motion_length = 500
    peak_idx = int(np.argmin(cmd_jnt_pos[:, search_jnt_idx])) + 1

    act_jnt_pos1 = _slice_around(act_jnt_pos, peak_idx, motion_length)
    act_jnt_trq1 = _slice_around(act_jnt_trq, peak_idx, motion_length)

    # --------------------------------------------------------
    # 参考模型参数
    # --------------------------------------------------------
    # link1
    m1 = 7.708
    mx1 = -1.28e-3 * m1
    my1 = -0.22e-3 * m1
    mz1 = -124.25e-3 * m1

    # link2
    m2 = 1.162
    mx2 = 77.8e-3 * m2
    my2 = -0.12e-3 * m2
    mz2 = -28.5e-3 * m2

    # link3
    m3 = 0.3594
    mx3 = 216.28e-3 * m3
    my3 = -1.77e-3 * m3
    mz3 = 0.12e-3 * m3

    # link4
    m4 = 0.2334
    mx4 = -0.2e-3 * m4
    my4 = 93.2e-3 * m4
    mz4 = -54.8e-3 * m4

    # link5
    m5 = 0.1824
    mx5 = -0.1e-3 * m5
    my5 = 81.3e-3 * m5
    mz5 = -41.1e-3 * m5

    # link6 & link7
    m6 = 0.189 + 0.043
    mx6 = 0e-3 * m6
    my6 = m6 * (-54.5 * 0.189 + 0.043 * 20) / m6 * 1e-3
    mz6 = 30.07e-3 * 0.189

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

    # spring
    preF2 = 12.9
    k2 = 1.59e3
    preF3 = 12.5
    k3 = 2.8e3
    preF5 = 5.7
    k5 = 0.687e3

    ref_param = np.array(
        [
            2.7244 * m3 + 2.7244 * m4 + 2.7244 * m5 + 2.7244 * m6 + 9.8 * mx2 + 9.8 * mx9,
            9.8 * my2 + 9.8 * my9,
            3.577 * m4 + 3.577 * m5 + 3.577 * m6 + 0.5978 * m9 + 9.8 * mx3 + 9.8 * mx8,
            1.5827 * m4 + 1.5827 * m5 + 1.5827 * m6 + 9.8 * my3 + 9.8 * my8 + 9.8 * mz4,
            mz5 - my4,
            my5 + mz6,
            mx4,
            mx5,
            mx6,
            my6,
            preF2,
            k2,
            preF3,
            k3,
            preF5,
            k5,
        ],
        dtype=float,
    )

    robot_param_buffer = 0.5
    spring_param_buffer = 0.5
    lb = ref_param.copy()
    ub = ref_param.copy()
    bound_limit = 0.001

    for i in range(10):
        if ref_param[i] > bound_limit:
            lb[i] = lb[i] * (1 - robot_param_buffer)
            ub[i] = ub[i] * (1 + robot_param_buffer)
        elif ref_param[i] < -bound_limit:
            lb[i] = lb[i] * (1 + robot_param_buffer)
            ub[i] = ub[i] * (1 - robot_param_buffer)
        else:
            lb[i] = -bound_limit
            ub[i] = bound_limit

    lb[10:16] = lb[10:16] * (1 - spring_param_buffer)
    ub[10:16] = ub[10:16] * (1 + spring_param_buffer)

    end_buffer = 100

    # --------------------------------------------------------
    # joint1/7
    # --------------------------------------------------------
    gravity_pos1, gravity_trq1, friction1 = _split_forward_back(
        act_jnt_pos1, act_jnt_trq1
    )

    gravity_pos1 = _trim_ends(gravity_pos1, end_buffer)
    gravity_trq1 = _trim_ends(gravity_trq1, end_buffer)
    friction1 = _trim_ends(friction1, end_buffer)

    friction_coeff1 = float(np.mean(np.abs(friction1[:, 0])))
    friction_coeff7 = float(np.mean(np.abs(friction1[:, 6])))

    # --------------------------------------------------------
    # joint6
    # --------------------------------------------------------
    gravity_pos6, gravity_trq6, friction6 = _split_forward_back(
        act_jnt_pos6, act_jnt_trq6
    )

    gravity_pos6 = _trim_ends(gravity_pos6, end_buffer)
    gravity_trq6 = _trim_ends(gravity_trq6, end_buffer)
    friction6 = _trim_ends(friction6, end_buffer)

    act_gravity_trq = np.abs(gravity_trq6[:, 5])
    act_friction_trq = np.abs(friction6[:, 5])
    friction_coeff6 = np.polyfit(act_gravity_trq, act_friction_trq, 1)

    reg_mat6, reg_trq6, reg_pos6 = _build_regression(
        gravity_pos6,
        gravity_trq6,
        row_index=4,
        trq_col=5,
        arm_idx=arm_idx,
        decimate_rate=decimate_rate,
    )

    lb6 = np.zeros(16, dtype=float)
    ub6 = np.zeros(16, dtype=float)
    lb6[8:10] = lb[8:10]
    ub6[8:10] = ub[8:10]

    gravity_param6 = _lsqlin(reg_mat6, reg_trq6, lb6, ub6)
    cali_gravity6 = reg_mat6 @ gravity_param6

    if plt is not None:
        plt.figure()
        plt.plot(reg_pos6, reg_trq6)
        plt.plot(reg_pos6, cali_gravity6)
        plt.legend(["measure torque", "calibrate torque"])
        plt.title(
            "Left Arm Joint6 Calibration Gravity"
            if arm_idx == 0
            else "Rigth Arm Joint6 Calibration Gravity"
        )

    # --------------------------------------------------------
    # joint5 重力
    # --------------------------------------------------------
    gravity_pos5, gravity_trq5, friction5 = _split_forward_back(
        act_jnt_pos5, act_jnt_trq5
    )

    gravity_pos5 = _trim_ends(gravity_pos5, end_buffer)
    gravity_trq5 = _trim_ends(gravity_trq5, end_buffer)
    friction5 = _trim_ends(friction5, end_buffer)

    act_gravity_trq = np.abs(gravity_trq5[:, 4])
    act_friction_trq = np.abs(friction5[:, 4])
    friction_coeff5 = np.polyfit(act_gravity_trq, act_friction_trq, 1)

    gravity_pos3_1, gravity_trq3_1, _ = _split_forward_back(
        act_jnt_pos3_1, act_jnt_trq3_1
    )
    gravity_pos3_2, gravity_trq3_2, _ = _split_forward_back(
        act_jnt_pos3_2, act_jnt_trq3_2
    )

    gravity_pos3_1 = _trim_ends(gravity_pos3_1, end_buffer)
    gravity_trq3_1 = _trim_ends(gravity_trq3_1, end_buffer)
    gravity_pos3_2 = _trim_ends(gravity_pos3_2, end_buffer)
    gravity_trq3_2 = _trim_ends(gravity_trq3_2, end_buffer)

    reg_mat5, reg_trq5 = _build_diff_regression(
        gravity_pos3_1,
        gravity_trq3_1,
        gravity_pos3_2,
        gravity_trq3_2,
        row_index=1,
        trq_col=2,
        arm_idx=arm_idx,
        decimate_rate=decimate_rate,
    )

    lb5 = gravity_param6.copy()
    ub5 = gravity_param6.copy()
    lb5[5] = lb[5]
    lb5[7] = lb[7]
    ub5[5] = ub[5]
    ub5[7] = ub[7]

    gravity_param5 = _lsqlin(reg_mat5, reg_trq5, lb5, ub5)
    cali_trq5 = reg_mat5 @ gravity_param5

    if plt is not None:
        plt.figure()
        plt.plot(reg_trq5)
        plt.plot(cali_trq5)
        plt.legend(["measure torque", "calibrate torque"])
        plt.title(
            "Left Arm Joint5 Calibration Gravity"
            if arm_idx == 0
            else "Rigth Arm Joint5 Calibration Gravity"
        )

    # --------------------------------------------------------
    # joint5 弹簧
    # --------------------------------------------------------
    reg_mat5_spring, reg_trq5_spring, reg_pos5 = _build_regression(
        gravity_pos5,
        gravity_trq5,
        row_index=3,
        trq_col=4,
        arm_idx=arm_idx,
        decimate_rate=decimate_rate,
    )

    gravity_trq = reg_mat5_spring[:, 0:10] @ gravity_param5[0:10]
    spring_trq = reg_trq5_spring - gravity_trq
    spring5_coeffs = np.polyfit(reg_pos5, spring_trq, 7)
    spring5_fit_trq = np.polyval(spring5_coeffs, reg_pos5)
    cali_gravity5 = spring5_fit_trq + gravity_trq

    if plt is not None:
        plt.figure()
        plt.plot(reg_pos5, reg_trq5_spring)
        plt.plot(reg_pos5, cali_gravity5)
        plt.legend(["measure torque", "calibrate torque"])
        plt.title(
            "Left Arm Joint5 Calibration Gravity and Spring"
            if arm_idx == 0
            else "Rigth Arm Joint5 Calibration Gravity and Spring"
        )

    # --------------------------------------------------------
    # joint4
    # --------------------------------------------------------
    friction_coeff4 = np.array([0.0, 0.0], dtype=float)

    gravity_pos3_3, gravity_trq3_3, _ = _split_forward_back(
        act_jnt_pos3_3, act_jnt_trq3_3
    )
    gravity_pos3_4, gravity_trq3_4, _ = _split_forward_back(
        act_jnt_pos3_4, act_jnt_trq3_4
    )

    gravity_pos3_3 = _trim_ends(gravity_pos3_3, end_buffer)
    gravity_trq3_3 = _trim_ends(gravity_trq3_3, end_buffer)
    gravity_pos3_4 = _trim_ends(gravity_pos3_4, end_buffer)
    gravity_trq3_4 = _trim_ends(gravity_trq3_4, end_buffer)

    reg_mat4, reg_trq4 = _build_diff_regression(
        gravity_pos3_3,
        gravity_trq3_3,
        gravity_pos3_4,
        gravity_trq3_4,
        row_index=1,
        trq_col=2,
        arm_idx=arm_idx,
        decimate_rate=decimate_rate,
    )

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
            else "Rigth Arm Joint4 Calibration Gravity"
        )

    # --------------------------------------------------------
    # joint3
    # --------------------------------------------------------
    gravity_pos3, gravity_trq3, friction3 = _split_forward_back(
        act_jnt_pos3, act_jnt_trq3
    )

    gravity_pos3 = _trim_ends(gravity_pos3, end_buffer)
    gravity_trq3 = _trim_ends(gravity_trq3, end_buffer)
    friction3 = _trim_ends(friction3, end_buffer)

    friction_coeff3 = float(np.mean(np.abs(friction3[:, 2])))

    reg_mat3, reg_trq3, reg_pos3 = _build_regression(
        gravity_pos3,
        gravity_trq3,
        row_index=1,
        trq_col=2,
        arm_idx=arm_idx,
        decimate_rate=decimate_rate,
    )

    gravity_trq = reg_mat3[:, 0:10] @ gravity_param4[0:10]
    spring_trq = reg_trq3 - gravity_trq
    jnt3_coeffs = np.polyfit(reg_pos3, spring_trq, 7)
    jnt3_fit_trq = np.polyval(jnt3_coeffs, reg_pos3)
    cali_gravity3 = jnt3_fit_trq + gravity_trq

    if plt is not None:
        plt.figure()
        plt.plot(reg_pos3, reg_trq3)
        plt.plot(reg_pos3, cali_gravity3)
        plt.legend(["measure torque", "calibrate torque"])
        plt.title(
            "Left Arm Joint3 Calibration Gravity"
            if arm_idx == 0
            else "Rigth Arm Joint3 Calibration Gravity"
        )

    # --------------------------------------------------------
    # joint2
    # --------------------------------------------------------
    gravity_pos2, gravity_trq2, friction2 = _split_forward_back(
        act_jnt_pos2, act_jnt_trq2
    )

    gravity_pos2 = _trim_ends(gravity_pos2, end_buffer)
    gravity_trq2 = _trim_ends(gravity_trq2, end_buffer)
    friction2 = _trim_ends(friction2, end_buffer)

    friction_coeff2 = float(np.mean(np.abs(friction2[:, 1])))

    reg_mat2, reg_trq2, reg_pos2 = _build_regression(
        gravity_pos2,
        gravity_trq2,
        row_index=0,
        trq_col=1,
        arm_idx=arm_idx,
        decimate_rate=decimate_rate,
    )

    gravity_trq = reg_mat2[:, 0:10] @ gravity_param4[0:10]
    spring_trq = reg_trq2 - gravity_trq
    jnt2_coeffs = np.polyfit(reg_pos2, spring_trq, 7)
    jnt2_fit_trq = np.polyval(jnt2_coeffs, reg_pos2)
    cali_gravity2 = jnt2_fit_trq + gravity_trq

    if plt is not None:
        plt.figure()
        plt.plot(reg_pos2, reg_trq2)
        plt.plot(reg_pos2, cali_gravity2)
        plt.legend(["measure torque", "calibrate torque"])
        plt.title(
            "Left Arm Joint2 Calibration Gravity"
            if arm_idx == 0
            else "Rigth Arm Joint2 Calibration Gravity"
        )

    if plt is not None:
        plt.show(block=False)

    # --------------------------------------------------------
    # 输出
    # --------------------------------------------------------
    gravity_params = np.concatenate(
        [
            gravity_param4[4:10],
            jnt2_coeffs,
            jnt3_coeffs,
            spring5_coeffs,
        ]
    )

    friction_params = np.concatenate(
        [
            np.array(
                [
                    0.0,
                    friction_coeff1,
                    0.0,
                    friction_coeff2,
                    0.0,
                    friction_coeff3,
                ],
                dtype=float,
            ),
            np.asarray(friction_coeff4, dtype=float).reshape(-1),
            np.asarray(friction_coeff5, dtype=float).reshape(-1),
            np.asarray(friction_coeff6, dtype=float).reshape(-1),
            np.array([0.0, friction_coeff7], dtype=float),
        ]
    )

    return gravity_params, friction_params


def masterGraCali(m_graCaliData):
    """对左右手记录数据执行 old master 重力参数辨识。

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

    left_gravity, left_friction = Identification(0, left_cmd, left_cur, left_trq)
    right_gravity, right_friction = Identification(1, right_cmd, right_cur, right_trq)

    result = ST_RMGraCaliData()

    # g_gravityParam: [2, 30]（PLC 类型 REAL）
    for i in range(30):
        result.g_gravityParam[0][i] = float(left_gravity[i])
        result.g_gravityParam[1][i] = float(right_gravity[i])

    # old master 的摩擦参数写入独立字段 g_jntFrictionParamOldArm: [2, 14]
    # 每个关节有 (A, B) 两个参数，共 7 * 2 = 14 个。
    for i in range(14):
        result.g_jntFrictionParamOldArm[0][i] = float(left_friction[i])
        result.g_jntFrictionParamOldArm[1][i] = float(right_friction[i])

    # old master 算法不辨识力矩常数比例，使用中性值 1.0。
    for arm_index in range(2):
        result.g_trqConstScale[arm_index][0] = 1.0
        result.g_trqConstScale[arm_index][1] = 1.0

    return result


# ============================================================
# MATLAB 转换辅助函数
# ============================================================
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
            f"Segment length {len(data)} is too short for endBuffer={end_buffer}."
        )

    return data[start:end, :]


def _slice_around(data, peak_1based, motion_length):
    """对应 MATLAB 的 data(peak-motionLength:peak+motionLength, :)。"""
    start0 = peak_1based - motion_length - 1
    end0 = peak_1based + motion_length

    if start0 < 0 or end0 > len(data):
        raise ValueError(f"Slice [{start0}:{end0}] is outside data length {len(data)}.")

    return data[start0:end0, :]


def _spring_regression_row(position, arm_idx):
    """构造 springTorque2/3/5 对应的 5x6 弹簧回归矩阵。"""
    spring_reg_mat = np.zeros((5, 6), dtype=float)

    a, b = springTorque2(position[1])
    spring_reg_mat[0, 0] = a
    spring_reg_mat[0, 1] = b

    a, b = springTorque3(position[2])
    spring_reg_mat[1, 2] = a
    spring_reg_mat[1, 3] = b

    a, b = springTorque5(position[4], arm_idx)
    spring_reg_mat[3, 4] = a
    spring_reg_mat[3, 5] = b

    return spring_reg_mat


def _build_regression(pos, trq, row_index, trq_col, arm_idx, decimate_rate):
    """构造 [重力项, 弹簧项] 的回归矩阵。"""
    n_reg = len(pos) // decimate_rate
    reg_mat = np.zeros((n_reg, 16), dtype=float)
    reg_trq = np.zeros(n_reg, dtype=float)
    reg_pos = np.zeros(n_reg, dtype=float)

    for k in range(1, n_reg + 1):
        i = k * decimate_rate - 1
        position = pos[i]

        gra_reg_mat = MasterGravityRegMat(position)
        spring_reg_mat = _spring_regression_row(position, arm_idx)

        reg_mat[k - 1, :] = np.concatenate(
            [gra_reg_mat[row_index, :], -spring_reg_mat[row_index, :]]
        )
        reg_trq[k - 1] = trq[i, trq_col]
        reg_pos[k - 1] = position[trq_col]

    return reg_mat, reg_trq, reg_pos


def _build_diff_regression(
    pos1,
    trq1,
    pos2,
    trq2,
    row_index,
    trq_col,
    arm_idx,
    decimate_rate,
):
    """构造两段轨迹之间的差分回归矩阵。"""
    n_reg = len(pos1) // decimate_rate
    reg_mat = np.zeros((n_reg, 16), dtype=float)
    reg_trq = np.zeros(n_reg, dtype=float)

    for k in range(1, n_reg + 1):
        i = k * decimate_rate - 1
        position1 = pos1[i]
        position2 = pos2[i]

        gra_reg_mat = MasterGravityRegMat(position1)
        spring_reg_mat = _spring_regression_row(position1, arm_idx)

        gra_reg_mat2 = MasterGravityRegMat(position2)
        spring_reg_mat2 = _spring_regression_row(position2, arm_idx)

        reg_mat[k - 1, :] = np.concatenate(
            [gra_reg_mat[row_index, :], -spring_reg_mat[row_index, :]]
        ) - np.concatenate(
            [gra_reg_mat2[row_index, :], -spring_reg_mat2[row_index, :]]
        )
        reg_trq[k - 1] = trq1[i, trq_col] - trq2[i, trq_col]

    return reg_mat, reg_trq


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


def _get_plt(show_plots):
    """按需导入 matplotlib。"""
    if not show_plots:
        return None

    try:
        import matplotlib.pyplot as plt

        return plt
    except ImportError:
        print("[MasterGraCaliOld] matplotlib is not installed; skip diagnostic plots.")
        return None


# ============================================================
# 重力回归矩阵（MATLAB MasterGravityRegMat.m 转换）
# ============================================================
def MasterGravityRegMat(q):
    """old master 重力回归矩阵。"""
    q2 = q[1]
    q8 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]

    reg_mat = np.zeros((5, 10), dtype=float)

    reg_mat[0, :] = [
        np.cos(q2),
        -np.sin(q2),
        0.0,
        0.0,
        -4.9 * (np.cos(q2 - q4) - 1.0 * np.cos(q4 - q2 + 2.0 * q8)),
        9.8 * np.cos(q4 - q2 + q8) * np.sin(q5) * np.sin(q8),
        -4.9 * (np.sin(q2 - 1.0 * q4) + np.sin(q4 - 1.0 * q2 + 2.0 * q8)),
        -9.8 * np.cos(q4 - q2 + q8) * np.cos(q5) * np.sin(q8),
        9.8
        * np.sin(q8)
        * (
            np.sin(q4 - q2 + q8) * np.sin(q6)
            - 1.0 * np.cos(q4 - q2 + q8) * np.cos(q5) * np.cos(q6)
        ),
        9.8
        * np.sin(q8)
        * (
            np.sin(q4 - q2 + q8) * np.cos(q6)
            + np.cos(q4 - q2 + q8) * np.cos(q5) * np.sin(q6)
        ),
    ]

    reg_mat[1, :] = [
        0.0,
        0.0,
        np.cos(q8),
        -np.sin(q8),
        -(
            9.8 * np.cos(2.0 * q8) * np.cos(q2) * np.cos(q4)
            + 9.8 * np.cos(2.0 * q8) * np.sin(q2) * np.sin(q4)
            - 9.8 * np.sin(2.0 * q8) * np.cos(q2) * np.sin(q4)
            + 9.8 * np.sin(2.0 * q8) * np.cos(q4) * np.sin(q2)
        ),
        -(
            9.8 * np.cos(q5) * np.sin(q8)
            + 9.8 * np.cos(2.0 * q8) * np.cos(q2) * np.sin(q4) * np.sin(q5)
            - 9.8 * np.cos(2.0 * q8) * np.cos(q4) * np.sin(q2) * np.sin(q5)
            + 9.8 * np.sin(2.0 * q8) * np.cos(q2) * np.cos(q4) * np.sin(q5)
            + 9.8 * np.sin(2.0 * q8) * np.sin(q2) * np.sin(q4) * np.sin(q5)
        ),
        (
            9.8 * np.cos(2.0 * q8) * np.cos(q2) * np.sin(q4)
            - 9.8 * np.cos(2.0 * q8) * np.cos(q4) * np.sin(q2)
            + 9.8 * np.sin(2.0 * q8) * np.cos(q2) * np.cos(q4)
            + 9.8 * np.sin(2.0 * q8) * np.sin(q2) * np.sin(q4)
        ),
        (
            9.8 * np.cos(2.0 * q8) * np.cos(q2) * np.cos(q5) * np.sin(q4)
            - 9.8 * np.sin(q5) * np.sin(q8)
            - 9.8 * np.cos(2.0 * q8) * np.cos(q4) * np.cos(q5) * np.sin(q2)
            + 9.8 * np.sin(2.0 * q8) * np.cos(q2) * np.cos(q4) * np.cos(q5)
            + 9.8 * np.sin(2.0 * q8) * np.cos(q5) * np.sin(q2) * np.sin(q4)
        ),
        (
            9.8 * np.cos(2.0 * q8) * np.cos(q2) * np.cos(q4) * np.sin(q6)
            - 9.8 * np.cos(q6) * np.sin(q5) * np.sin(q8)
            + 9.8 * np.cos(2.0 * q8) * np.sin(q2) * np.sin(q4) * np.sin(q6)
            - 9.8 * np.sin(2.0 * q8) * np.cos(q2) * np.sin(q4) * np.sin(q6)
            + 9.8 * np.sin(2.0 * q8) * np.cos(q4) * np.sin(q2) * np.sin(q6)
            + 9.8
            * np.cos(2.0 * q8)
            * np.cos(q2)
            * np.cos(q5)
            * np.cos(q6)
            * np.sin(q4)
            - 9.8
            * np.cos(2.0 * q8)
            * np.cos(q4)
            * np.cos(q5)
            * np.cos(q6)
            * np.sin(q2)
            + 9.8
            * np.sin(2.0 * q8)
            * np.cos(q2)
            * np.cos(q4)
            * np.cos(q5)
            * np.cos(q6)
            + 9.8 * np.sin(2.0 * q8) * np.cos(q5) * np.cos(q6) * np.sin(q2) * np.sin(q4)
        ),
        (
            9.8 * np.sin(q5) * np.sin(q6) * np.sin(q8)
            + 9.8 * np.cos(2.0 * q8) * np.cos(q2) * np.cos(q4) * np.cos(q6)
            + 9.8 * np.cos(2.0 * q8) * np.cos(q6) * np.sin(q2) * np.sin(q4)
            - 9.8 * np.sin(2.0 * q8) * np.cos(q2) * np.cos(q6) * np.sin(q4)
            + 9.8 * np.sin(2.0 * q8) * np.cos(q4) * np.cos(q6) * np.sin(q2)
            - 9.8
            * np.cos(2.0 * q8)
            * np.cos(q2)
            * np.cos(q5)
            * np.sin(q4)
            * np.sin(q6)
            + 9.8
            * np.cos(2.0 * q8)
            * np.cos(q4)
            * np.cos(q5)
            * np.sin(q2)
            * np.sin(q6)
            - 9.8
            * np.sin(2.0 * q8)
            * np.cos(q2)
            * np.cos(q4)
            * np.cos(q5)
            * np.sin(q6)
            - 9.8 * np.sin(2.0 * q8) * np.cos(q5) * np.sin(q2) * np.sin(q4) * np.sin(q6)
        ),
    ]

    reg_mat[2, :] = [
        0.0,
        0.0,
        0.0,
        0.0,
        9.8 * np.sin(q4 - 1.0 * q2 + q8) * np.sin(q8),
        -9.8 * np.cos(q4 - 1.0 * q2 + q8) * np.sin(q5) * np.sin(q8),
        9.8 * np.cos(q4 - 1.0 * q2 + q8) * np.sin(q8),
        9.8 * np.cos(q4 - 1.0 * q2 + q8) * np.cos(q5) * np.sin(q8),
        -9.8
        * np.sin(q8)
        * (
            np.sin(q4 - 1.0 * q2 + q8) * np.sin(q6)
            - 1.0 * np.cos(q4 - 1.0 * q2 + q8) * np.cos(q5) * np.cos(q6)
        ),
        -9.8
        * np.sin(q8)
        * (
            np.sin(q4 - 1.0 * q2 + q8) * np.cos(q6)
            + np.cos(q4 - 1.0 * q2 + q8) * np.cos(q5) * np.sin(q6)
        ),
    ]

    reg_mat[3, :] = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -(9.8 * np.cos(q8) * np.sin(q5) + 9.8 * np.sin(q4 - 1.0 * q2 + q8) * np.cos(q5) * np.sin(q8)),
        0.0,
        (9.8 * np.cos(q5) * np.cos(q8) - 9.8 * np.sin(q4 - 1.0 * q2 + q8) * np.sin(q5) * np.sin(q8)),
        9.8
        * np.cos(q6)
        * (
            np.cos(q5) * np.cos(q8)
            - 1.0 * np.sin(q4 - 1.0 * q2 + q8) * np.sin(q5) * np.sin(q8)
        ),
        -9.8
        * np.sin(q6)
        * (
            np.cos(q5) * np.cos(q8)
            - 1.0 * np.sin(q4 - 1.0 * q2 + q8) * np.sin(q5) * np.sin(q8)
        ),
    ]

    reg_mat[4, :] = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -(
            9.8 * np.cos(q8) * np.sin(q5) * np.sin(q6)
            - 9.8 * np.cos(q4 - 1.0 * q2 + q8) * np.cos(q6) * np.sin(q8)
            + 9.8
            * np.sin(q4 - 1.0 * q2 + q8)
            * np.cos(q5)
            * np.sin(q6)
            * np.sin(q8)
        ),
        -(
            9.8 * np.cos(q4 - 1.0 * q2 + q8) * np.sin(q6) * np.sin(q8)
            + 9.8 * np.cos(q6) * np.cos(q8) * np.sin(q5)
            + 9.8
            * np.sin(q4 - 1.0 * q2 + q8)
            * np.cos(q5)
            * np.cos(q6)
            * np.sin(q8)
        ),
    ]

    return reg_mat


# ============================================================
# 弹簧力矩回归项（MATLAB springTorque*.m 转换）
# ============================================================
def springTorque2(q):
    """对应 MATLAB springTorque2.m。"""
    L0 = 114.3e-3
    b1 = 20e-3
    b2 = 16e-3
    l0 = 200e-3
    r = 30e-3
    L0 = L0 + b1 + b2

    theta = q + np.deg2rad(90.0)
    L = np.sqrt(l0**2 + r**2 + 2.0 * l0 * r * np.cos(theta))
    t = r * l0 * np.sin(theta) / L

    a = t
    b = t * (L - L0)
    return a, b


def springTorque3(q):
    """对应 MATLAB springTorque3.m。"""
    L0 = 127.5e-3
    b1 = 20e-3
    b2 = 23e-3
    l0 = 200e-3
    r = 32e-3
    d = 23e-3
    L0 = L0 + b1 + b2

    alpha = q + np.deg2rad(90.0)
    tan_beta = (r * np.sin(alpha) + d) / (l0 + r * np.cos(alpha))
    beta = np.arctan(tan_beta)
    L = d / np.sin(beta) + r * np.sin(alpha) / np.sin(beta)
    t = (l0 - d / np.tan(beta)) * np.sin(beta)

    a = t
    b = t * (L - L0)
    return a, b


def springTorque5(q, armIdx):
    """对应 MATLAB springTorque5.m。"""
    L0 = 70e-3
    s1 = 17e-3
    L1 = 128e-3
    R = 7.5e-3
    beta = np.deg2rad(28.0)
    L0 = L0 + s1

    if armIdx == 0:
        theta = q - np.pi
    elif armIdx == 1:
        theta = np.pi - q
    else:
        raise ValueError("armIdx must be 0 (left) or 1 (right).")

    L = np.sqrt(L1**2 + R**2 + 2.0 * L1 * R * np.cos(theta + beta))
    t = L1 * R * np.sin(theta + beta) / L

    if armIdx == 0:
        a = t
        b = t * (L - L0)
    else:
        a = -t
        b = -t * (L - L0)

    return a, b
