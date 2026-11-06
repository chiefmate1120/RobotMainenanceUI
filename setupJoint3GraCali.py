"""Setup joint 3 gravity calibration.

由以下 MATLAB 文件合并转换而来：
    caliDataProcess.m
    setupJoint3GravityFit.m

对外接口：
    run(graCaliData, armIdx, show_plots=True)

graCaliData 为 slaveRobotInterface.ST_graCaliData，包含：
    m_cmdJntPos : n x 4 x 13，指令关节位置
    m_curJntPos : n x 4 x 13，实际关节位置
    m_curJntTrq : n x 4 x 13，实际关节力矩

armIdx 为 1~4 号臂。run 内部提取该臂的 setup joint 3
（TwinCAT 关节 [3]，Python 0-based 索引 2）后交给 caliDataProcess。
数据默认按 1000 Hz 采集。
"""

import numpy as np
from scipy.signal import butter, filtfilt

# 当前数据采集频率（Hz）。
DATA_FS = 1000.0

# 原始 MATLAB 参数：fs=1000 Hz，singleMotionLength=13000。
SINGLE_MOTION_LENGTH = 13000

# 滤波截止频率（Hz），与采样率无关。
FC = 5.0

# 首尾过渡段比例。
END_BUFFER_RATIO = 0.05

# setup joint 3 在 PLC 中为关节 [3]（1-based），对应 Python 0-based 索引 2。
SETUP_JNT3_INDEX = 2


def _trim_ends(data, end_buffer):
    """对应 MATLAB 的 data(endBuffer:end-endBuffer)。"""
    if end_buffer <= 0:
        return data

    start = end_buffer - 1
    end = len(data) - end_buffer
    if start < 0 or end <= start:
        return data

    return data[start:end]


def caliDataProcess(cmdJntPos, curJntPos, curJntTrq):
    """处理原始采样数据，返回单向运动段的关节位置、重力力矩和摩擦力矩。

    Parameters
    ----------
    cmdJntPos : sequence
        指令关节位置 1D 序列。
    curJntPos : sequence
        实际关节位置 1D 序列。
    curJntTrq : sequence
        实际关节力矩 1D 序列。
    show_plots : bool, optional
        是否生成诊断图，默认 True。

    Returns
    -------
    jnt_pos : ndarray
    jnt_trq : ndarray
    jnt_friction : ndarray
    """
    # 滤波
    b, a = butter(1, 2.0 * FC / DATA_FS)
    try:
        jnt_trq_filtered = filtfilt(b, a, curJntTrq)
    except ValueError as exc:
        print(f"[SetupJoint3GraCali] filtfilt failed: {exc}; use raw data.")
        jnt_trq_filtered = curJntTrq.copy()

    # 找往返运动峰值，并截取一个完整运动段。
    single_motion_length = SINGLE_MOTION_LENGTH
    peak_idx = int(np.argmax(cmdJntPos))
    start = peak_idx - single_motion_length
    end = peak_idx + single_motion_length + 1

    if start < 0 or end > len(curJntPos):
        raise ValueError(
            f"Motion window [{start}:{end}] is outside data length "
            f"{len(curJntPos)}."
        )

    jnt_pos = curJntPos[start:end]
    jnt_trq_segment = jnt_trq_filtered[start:end]

    # 前半程与后半程（后半程翻转）
    n = len(jnt_trq_segment)
    half = n // 2
    forth_trq = jnt_trq_segment[:half]
    back_trq = np.flip(jnt_trq_segment[(n + 1) // 2 :])

    m = min(len(forth_trq), len(back_trq))
    if m == 0:
        raise ValueError("Segment is too short for forward/back splitting.")

    forth_trq = forth_trq[:m]
    back_trq = back_trq[:m]
    jnt_pos = jnt_pos[:m]

    # 平均得到重力项，差值得到摩擦项。
    jnt_trq = (forth_trq + back_trq) / 2.0
    jnt_friction = np.abs(forth_trq - back_trq) / 2.0

    # 去掉首尾过渡段。
    end_buffer = int(END_BUFFER_RATIO * len(jnt_pos))
    jnt_pos = _trim_ends(jnt_pos, end_buffer)
    jnt_trq = _trim_ends(jnt_trq, end_buffer)
    jnt_friction = _trim_ends(jnt_friction, end_buffer)

    return jnt_pos, jnt_trq, jnt_friction


def _plot_cali_process(
    cmd_jnt_pos,
    cur_jnt_pos,
    cur_jnt_trq,
    filt_jnt_trq,
    forth_trq,
    back_trq,
    jnt_trq,
    jnt_friction,
):
    """生成 caliDataProcess 对应的诊断图。"""
    import matplotlib.pyplot as plt

    plt.figure()
    plt.plot(cmd_jnt_pos)
    plt.plot(cur_jnt_pos)
    plt.legend(["cmdJnt3", "curJnt3"])
    plt.title("Joint Position of the Calibration Process")

    plt.figure()
    plt.plot(cur_jnt_trq)
    plt.plot(filt_jnt_trq)
    plt.legend(["curJntTrq", "filtJntTrq"])
    plt.title("Joint Torque of Valid Calibration Process")

    plt.figure()
    plt.plot(forth_trq)
    plt.plot(back_trq)
    plt.legend(["forthJnt3", "backJnt3"])
    plt.title("Joint Torque of Forward and Backward Motion")

    plt.figure()
    plt.plot(jnt_trq)
    plt.plot(jnt_friction)
    plt.legend(["gravityJnt3", "frictionJnt3"])
    plt.title("Joint Gravity and Friction")


def run(graCaliData, armIdx, show_plots=True):
    """Setup joint 3 gravity/friction fitting.

    Parameters
    ----------
    graCaliData : slaveRobotInterface.ST_graCaliData
        包含 m_cmdJntPos / m_curJntPos / m_curJntTrq，
        每个采样为 4 x 13 的二维数组。
    armIdx : int
        手臂编号，1~4。
    show_plots : bool, optional
        是否生成诊断图，默认 True。

    Returns
    -------
    result : dict
        gravityCoeffs / frictionCoeffs
    """

    armIdx = int(armIdx)
    if armIdx not in (1, 2, 3, 4):
        raise ValueError(f"Invalid armIdx: {armIdx}. Expected 1~4.")

    # 提取指定手臂 setup joint 3 的 1D 序列。
    arm_index = armIdx - 1
    cmdJntPos = [
        sample[arm_index][SETUP_JNT3_INDEX] for sample in graCaliData.m_cmdJntPos
    ]
    curJntPos = [
        sample[arm_index][SETUP_JNT3_INDEX] for sample in graCaliData.m_curJntPos
    ]
    curJntTrq = [
        sample[arm_index][SETUP_JNT3_INDEX] for sample in graCaliData.m_curJntTrq
    ]

    jnt_pos, jnt_trq, jnt_friction = caliDataProcess(
        cmdJntPos,
        curJntPos,
        curJntTrq,
    )

    # 重力项：三次多项式拟合
    gravity_coeffs = np.polyfit(jnt_pos, jnt_trq, 3)
    fit_gravity = np.polyval(gravity_coeffs, jnt_pos)

    # 摩擦项：三次多项式拟合
    friction_coeffs = np.polyfit(jnt_pos, jnt_friction, 3)
    fit_friction = np.polyval(friction_coeffs, jnt_pos)

    result = {
        "gravityCoeffs": gravity_coeffs,
        "frictionCoeffs": friction_coeffs,
    }

    if show_plots:
        _plot_fit_results(
            jnt_pos,
            jnt_trq,
            jnt_friction,
            fit_gravity,
            fit_friction,
        )
        import matplotlib.pyplot as plt

        plt.show(block=False)

    return result


def _plot_fit_results(
    jnt_pos,
    jnt_trq,
    jnt_friction,
    fit_gravity,
    fit_friction,
):
    """生成 setupJoint3GravityFit.m 对应的拟合图。"""
    import matplotlib.pyplot as plt

    plt.figure()
    plt.plot(jnt_pos, jnt_trq)
    plt.plot(jnt_pos, fit_gravity)
    plt.legend(["meaGravity", "fitGravity"])
    plt.title("Measured and Fitted Gravity")

    plt.figure()
    plt.plot(jnt_pos, jnt_friction)
    plt.plot(jnt_pos, fit_friction)
    plt.legend(["meaFriction", "fitFriction"])
    plt.title("Measured and Fitted Friction")
