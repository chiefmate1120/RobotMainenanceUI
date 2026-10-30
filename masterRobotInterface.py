import ctypes
import time
import pyads
import queue


# slave robot status
class ST_RMRobotStatusData(ctypes.Structure):
    _fields_ = [
        ("m_cartJntPos", ctypes.c_double * 4),
        ("m_cartStatusWord", ctypes.c_int32 * 4),
        ("m_cartCmdWord", ctypes.c_int32 * 4),
        ("m_cartCmdMotorEncoder", ctypes.c_int32 * 4),
        ("m_cartCurMotorEncoder", ctypes.c_int32 * 4),
        ("m_cartcurLinkEncoder", ctypes.c_int32 * 4),
        ("m_armJntPos", (ctypes.c_double * 7) * 2),
        ("m_armCmdJntPos", (ctypes.c_double * 7) * 2),
        ("m_armJntTrq", (ctypes.c_double * 7) * 2),
        ("m_armStatusWord", (ctypes.c_int32 * 7) * 2),
        ("m_armCmdWord", (ctypes.c_int32 * 7) * 2),
        ("m_armOPMode", (ctypes.c_int16 * 7) * 2),
        ("m_armCmdMotorEncoder", (ctypes.c_int32 * 7) * 2),
        ("m_armCurMotorEncoder", (ctypes.c_int32 * 7) * 2),
        ("m_armCurLinkEncoder", (ctypes.c_int32 * 7) * 2),
        ("m_armCmdMotorCurrent", (ctypes.c_int32 * 7) * 2),
        ("m_armCurMotorCurrent", (ctypes.c_int32 * 7) * 2),
    ]


class ST_RMJntCtrlData(ctypes.Structure):
    _fields_ = [
        ("m_cartJntEnable", ctypes.c_bool * 4),
        ("m_cartCmdJntPos", ctypes.c_double * 4),
        ("m_armJntEnable", (ctypes.c_bool * 7) * 2),
        ("m_armCmdOPMode", (ctypes.c_int16 * 7) * 2),
        ("m_armCmdJntPos", (ctypes.c_double * 7) * 2),
        ("m_armCmdMotorCurrent", (ctypes.c_int32 * 7) * 2),
    ]


# 与 GVL_ArmDynamics 中的 g_jntXCogTrqMapLen 保持一致。
JNT5_COG_TRQ_MAP_LEN = 1064
JNT6_COG_TRQ_MAP_LEN = 560
JNT7_COG_TRQ_MAP_LEN = 1440


class ST_RMGraCaliData(ctypes.Structure):
    """主手重力配平数据。

    TwinCAT 类型：
    REAL  -> ctypes.c_float
    LREAL -> ctypes.c_double
    BOOL  -> ctypes.c_bool
    """

    _fields_ = [
        ("m_homeFlag", ctypes.c_bool),
        ("m_caliFlag", ctypes.c_bool),
        ("m_verifyFlag", ctypes.c_bool),
        ("m_paramDownload", ctypes.c_bool),
        ("g_gravityParam", (ctypes.c_double * 30) * 2),
        ("g_jntFrictionParam", (ctypes.c_double * 7) * 2),
        ("g_trqConstScale", (ctypes.c_double * 2) * 2),
        ("g_leftJnt5CogTrq", ctypes.c_double * JNT5_COG_TRQ_MAP_LEN),
        ("g_leftJnt6CogTrq", ctypes.c_double * JNT6_COG_TRQ_MAP_LEN),
        ("g_leftJnt7CogTrq", ctypes.c_double * JNT7_COG_TRQ_MAP_LEN),
        ("g_rightJnt5CogTrq", ctypes.c_double * JNT5_COG_TRQ_MAP_LEN),
        ("g_rightJnt6CogTrq", ctypes.c_double * JNT6_COG_TRQ_MAP_LEN),
        ("g_rightJnt7CogTrq", ctypes.c_double * JNT7_COG_TRQ_MAP_LEN),
        ("g_jntFrictionParamOldArm", (ctypes.c_double * 14) * 2),
    ]


class ST_graCaliData:
    """重力标定采样数据，每个成员均为 n x 7。"""

    def __init__(self):
        self.m_leftCmdJntPos = []
        self.m_leftCurJntPos = []
        self.m_leftJntTrq = []
        self.m_rightCmdJntPos = []
        self.m_rightCurJntPos = []
        self.m_rightJntTrq = []

    def clear(self):
        self.m_leftCmdJntPos.clear()
        self.m_leftCurJntPos.clear()
        self.m_leftJntTrq.clear()
        self.m_rightCmdJntPos.clear()
        self.m_rightCurJntPos.clear()
        self.m_rightJntTrq.clear()


# enable UI control, transit PLC state machine
# set by UI
m_UICtrlEnable = False

# TwinCAT TYPE RMModule
RMModule_UNKNOWN = -1
RMModule_Cart = 0
RMModule_Arm1 = 1
RMModule_Arm2 = 2
RMModule_ArmAll = 3

# mode. set by UI
# 0: joint control 1:gravity calibraiton
m_RMMode = 0

ADSDataConfig = {
    0: (
        "GVL_RMInterfaceData.g_RMJntCtrlData",
        ST_RMJntCtrlData,
    ),
    1: (
        "GVL_RMInterfaceData.g_RMGraCaliData",
        ST_RMGraCaliData,
    ),
}

# ==========================================
# 2. 全局变量定义
# ==========================================
plc = None
m_robotStatusDataQueue = queue.Queue(maxsize=10 * 1000)
m_UIHeartBeat = 0  # Python 端的心跳变量
m_robotConnected = False
m_robotConnectCounts = 0
m_robotDataBufferSize = 100
m_robotCmdsSeq = 1

# 重力标定记录抽点：每 10 个缓存数据记录 1 个。
GRA_CALI_RECORD_EVERY = 10

# 主手重力配平状态（由 PLC 的 g_masterGraCaliState 通知更新）。
# -1 表示尚未收到 PLC 状态（未连接 / 首次读取失败），UI 指示灯保持灰色。
m_masterGraCaliState = -1

# PLC 机器人模块类型（GVL_RMInterfaceData.g_robotModule）。
# 1: slave, 2: new master, 3: new4567 master, 4: new567 master, 5: old master
# -1 表示尚未从 PLC 读取。
m_robotModule = -1

# 重力标定采样数据。
m_graCaliData = ST_graCaliData()


lastTime = time.perf_counter()


def setRMMode(mode):
    """设置当前模式；切换到重力配平模式（1）时清空采样数据。"""
    global m_RMMode

    mode = int(mode)

    if mode == 1 and m_RMMode != 1:
        m_graCaliData.clear()
        print("[GraCali] Cleared m_graCaliData on entering mode 1.")

    m_RMMode = mode


def connect(ams_net_id):
    res = False
    """
    负责和 TwinCAT 建立通信，并注册 g_robotStatusData 变量的读取回调
    """
    global plc, m_robotConnected, m_robotStatusDataQueue, lastTime
    global m_masterGraCaliState, m_robotModule

    # 已经连接时不要重复创建连接、重复注册 notification。
    if plc is not None and plc.is_open and m_robotConnected:
        print(
            "[Connect] Already connected, " "skip duplicate notification registration."
        )
        return True

    try:
        ADS_PORT = 851
        # get ip
        parts = ams_net_id.split(".")
        PLC_IP = ".".join(parts[:4]) if len(parts) >= 4 else ams_net_id
        plc = pyads.Connection(ams_net_id, ADS_PORT, PLC_IP)
        plc.open()
        print(f"[Connect] Successfully connected to TwinCAT, AMS Net ID: {ams_net_id}")

        # 连接成功后先读取机器人模块类型，供重力标定选择算法。
        try:
            m_robotModule = int(
                plc.read_by_name(
                    "GVL_RMInterfaceData.g_robotModule",
                    pyads.PLCTYPE_INT,
                )
            )
            print(f"[Connect] g_robotModule = {m_robotModule}")

        except Exception as e:
            m_robotModule = -1
            print(f"[Connect] Read g_robotModule failed: {e}")

        # 定义带有 pyads.notification 装饰器的回调函数
        # 传入整个 ARRAY[1..100] 的 ctypes 类型，pyads 会自动解析并传给 value
        @plc.notification(ST_RMRobotStatusData * m_robotDataBufferSize)
        def readRobotData(handle, name, timestamp, value):
            global m_robotStatusDataQueue, m_robotConnected, m_robotConnectCounts, lastTime
            try:
                # 重力配平模式下，PLC 状态为 4 时开始记录。
                recording = m_RMMode == 1 and m_masterGraCaliState == 2

                # 遍历这 100 个结构体，逐个放入队列
                for i in range(m_robotDataBufferSize):
                    single_item = value[
                        i
                    ]  # 每一个元素就是一个单独的 ST_RMRobotStatusData 结构体

                    if m_robotStatusDataQueue.full():
                        m_robotStatusDataQueue.get_nowait()  # 队列满时丢弃最老的

                    m_robotStatusDataQueue.put(single_item)

                    # 每 GRA_CALI_RECORD_EVERY 个缓存数据记录 1 个。
                    if recording and (i % GRA_CALI_RECORD_EVERY == 0):
                        # 第一行 = 左主手，第二行 = 右主手。
                        m_graCaliData.m_leftCurJntPos.append(
                            list(single_item.m_armJntPos[0])
                        )
                        m_graCaliData.m_rightCurJntPos.append(
                            list(single_item.m_armJntPos[1])
                        )
                        m_graCaliData.m_leftCmdJntPos.append(
                            list(single_item.m_armCmdJntPos[0])
                        )
                        m_graCaliData.m_rightCmdJntPos.append(
                            list(single_item.m_armCmdJntPos[1])
                        )
                        m_graCaliData.m_leftJntTrq.append(
                            list(single_item.m_armJntTrq[0])
                        )
                        m_graCaliData.m_rightJntTrq.append(
                            list(single_item.m_armJntTrq[1])
                        )

            except Exception as e:
                print(f"[Error] Failed to put data to queue: {e}")

            m_robotConnected = True
            m_robotConnectCounts = 0

            current_time = time.perf_counter()
            interval_ms = (current_time - lastTime) * 1000.0  # 转换为毫秒
            # print(f"[Callback Interval] Time since last callback: {interval_ms:.2f} ms")

            lastTime = current_time

        # 示例：访问第一个机器人的第一个关节位置
        # print(m_robotStatusData[0].m_cartJntPos[0])

        # 定义通知属性
        attr = pyads.NotificationAttrib(
            length=ctypes.sizeof(ST_RMRobotStatusData * m_robotDataBufferSize),
            trans_mode=pyads.ADSTRANS_SERVERONCHA,
            max_delay=0,
            cycle_time=10,  # 1000*100ns,
        )

        # 注册变量变化监听通知（直接绑定上面定义好的带有装饰器的函数）
        plc.add_device_notification(
            "GVL_RMInterfaceData.g_masterRobotStatusData", attr, readRobotData
        )
        print(
            "[Connect] Successfully registered notification for 'g_robotStatusData' with decorator."
        )

        # 注册主手重力配平状态通知。
        @plc.notification(pyads.PLCTYPE_INT)
        def readMasterGraCaliState(handle, name, timestamp, value):
            global m_masterGraCaliState
            m_masterGraCaliState = int(value)
            print(f"[Callback] g_masterGraCaliState = {m_masterGraCaliState}")

        gra_cali_attr = pyads.NotificationAttrib(
            length=ctypes.sizeof(pyads.PLCTYPE_INT),
            trans_mode=pyads.ADSTRANS_SERVERONCHA,
            max_delay=0,
            cycle_time=10,  # 1000*100ns,
        )

        plc.add_device_notification(
            "GVL_RMInterfaceData.g_masterGraCaliState",
            gra_cali_attr,
            readMasterGraCaliState,
        )
        print(
            "[Connect] Successfully registered notification for "
            "'g_masterGraCaliState'."
        )

        # 先读取一次当前值，避免 notification 尚未触发时仍为默认值。
        try:
            m_masterGraCaliState = int(
                plc.read_by_name(
                    "GVL_RMInterfaceData.g_masterGraCaliState",
                    pyads.PLCTYPE_INT,
                )
            )
        except Exception as e:
            print(f"[Connect] Read g_masterGraCaliState failed: {e}")

        m_robotConnected = True
        res = True

    except Exception as e:
        print(f"[Error] Connection or notification registration failed: {e}")

    return res


def writeHeartBeat():
    """
    负责向 PLC 中的 GVL_RMInterfaceData.g_UIHeartBeat 写入 Python 变量 m_UIHeartBeat
    """
    global plc, m_UIHeartBeat, m_robotConnectCounts, m_robotConnected
    if plc is not None:
        try:
            plc.write_by_name(
                "GVL_RMInterfaceData.g_UIHeartBeat", m_UIHeartBeat, pyads.PLCTYPE_INT
            )
            print(f"[Heartbeat] Written g_UIHeartBeat = {m_UIHeartBeat}")
            m_UIHeartBeat = (m_UIHeartBeat + 1) % 100
        except Exception as e:
            print(f"[Error] Failed to write heartbeat: {e}")
    else:
        print("[Warning] PLC is not connected yet.")

    m_robotConnectCounts = min(20, (m_robotConnectCounts + 1))
    if m_robotConnectCounts > 5:
        m_robotConnected = False

        # 连接已判定失效：关闭并清除旧连接，
        # 避免下次 connect 时旧端口和 notification 仍然存在。
        if plc is not None:
            try:
                plc.close()
            except Exception as e:
                print(f"[Heartbeat] Failed to close stale connection: {e}")
            plc = None


def writeUIEnable(enableFlag, module_idx):
    global plc, m_UICtrlEnable

    if plc is None or not plc.is_open:
        print("[Warning] PLC is not connected yet.")
        return False

    try:
        plc.write_by_name(
            "GVL_RMInterfaceData.g_UICtrlModule",
            int(module_idx),
            pyads.PLCTYPE_INT,
        )
        plc.write_by_name(
            "GVL_RMInterfaceData.g_RMMode",
            m_RMMode,
            pyads.PLCTYPE_INT,
        )

        enable = bool(enableFlag)
        plc.write_by_name(
            "GVL_RMInterfaceData.g_UICtrlEnable",
            enable,
            pyads.PLCTYPE_BOOL,
        )
        m_UICtrlEnable = enable

        return True
    except Exception as e:
        print(f"[Error] Failed to write UI enable: {e}")
        return False


def writeModuleIdx(module_idx):
    """
    将当前分页编号写入 PLC 变量 GVL_RMInterfaceData.g_UICtrlModule。
    0=患者台车，1~4=1~4号臂。
    """
    global plc

    if plc is None or not plc.is_open:
        print("[Warning] PLC is not connected yet.")
        return False

    try:
        plc.write_by_name(
            "GVL_RMInterfaceData.g_UICtrlModule",
            int(module_idx),
            pyads.PLCTYPE_INT,
        )
        plc.write_by_name(
            "GVL_RMInterfaceData.g_RMMode",
            m_RMMode,
            pyads.PLCTYPE_INT,
        )
        print(f"[UICtrlModule] Written g_UICtrlModule = {module_idx}")
        return True
    except Exception as e:
        print(f"[Error] Failed to write UI control module index: {e}")
        return False


def writeUICmds(cmds):
    global plc, m_robotCmdsSeq, m_RMMode

    if plc is None or not plc.is_open:
        print("[Warning] PLC is not connected yet.")
        return False

    config = ADSDataConfig.get(m_RMMode)
    if config is None:
        print(f"[Error] Invalid command mode: {m_RMMode}")
        return False

    plc_var_name, struct_type = config

    try:
        plc.write_by_name(
            plc_var_name,
            cmds,
            struct_type,
        )

        plc.write_by_name(
            "GVL_RMInterfaceData.g_RMMode",
            m_RMMode,
            pyads.PLCTYPE_INT,
        )

        m_robotCmdsSeq = (m_robotCmdsSeq + 1) % 10
        plc.write_by_name(
            "GVL_RMInterfaceData.g_UICmdsDataSeq",
            m_robotCmdsSeq,
            pyads.PLCTYPE_INT,
        )
        print("[UICmds] Written g_UICmdsData")
        return True
    except Exception as e:
        print(f"[Error] Failed to write UI commands: {e}")
        return False
