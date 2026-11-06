import ctypes
import time
import pyads
import queue


# slave robot status
class ST_RMRobotStatusData(ctypes.Structure):
    _fields_ = [
        ("m_cartJntPos", ctypes.c_double * 8),
        ("m_cartStatusWord", ctypes.c_int32 * 8),
        ("m_cartCmdWord", ctypes.c_int32 * 8),
        ("m_cartCmdMotorEncoder", ctypes.c_int32 * 8),
        ("m_cartCurMotorEncoder", ctypes.c_int32 * 8),
        ("m_cartcurLinkEncoder", ctypes.c_int32 * 8),
        ("m_armJntPos", (ctypes.c_double * 13) * 4),
        ("m_armCmdJntPos", (ctypes.c_double * 13) * 4),
        ("m_armJntTrq", (ctypes.c_double * 13) * 4),
        ("m_armStatusWord", (ctypes.c_int32 * 13) * 4),
        ("m_armCmdWord", (ctypes.c_int32 * 13) * 4),
        ("m_armOPMode", (ctypes.c_int16 * 13) * 4),
        ("m_armCmdMotorEncoder", (ctypes.c_int32 * 13) * 4),
        ("m_armCurMotorEncoder", (ctypes.c_int32 * 13) * 4),
        ("m_armCurLinkEncoder", (ctypes.c_int32 * 13) * 4),
        ("m_armCmdMotorCurrent", (ctypes.c_int32 * 13) * 4),
        ("m_armCurMotorCurrent", (ctypes.c_int32 * 13) * 4),
    ]


class ST_RMJntCtrlData(ctypes.Structure):
    _fields_ = [
        ("m_cartJntEnable", ctypes.c_bool * 8),
        ("m_cartCmdJntPos", ctypes.c_double * 8),
        ("m_armJntEnable", (ctypes.c_bool * 13) * 4),
        ("m_armCmdOPMode", (ctypes.c_int16 * 13) * 4),
        ("m_armCmdJntPos", (ctypes.c_double * 13) * 4),
        ("m_armCmdMotorCurrent", (ctypes.c_int32 * 13) * 4),
    ]


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
        ("m_setupJnt3GravityParams", (ctypes.c_double * 4) * 4),
        ("m_setupJnt3FrictionParams", (ctypes.c_double * 4) * 4),
    ]


class ST_graCaliData:
    """从手重力标定采样数据。

    每个成员为 n 个采样，每个采样是 4 x 13 的二维数组：
        第一维：手臂编号（0~3 对应 1~4 号臂）
        第二维：关节编号（0~12 对应 1~13 号关节）
    """

    def __init__(self):
        self.m_cmdJntPos = []
        self.m_curJntPos = []
        self.m_curJntTrq = []

    def clear(self):
        self.m_cmdJntPos.clear()
        self.m_curJntPos.clear()
        self.m_curJntTrq.clear()


# enable UI control, transit PLC state machine
# set by UI
m_UICtrlEnable = False

# TwinCAT TYPE RMModule
RMModule_UNKNOWN = -1
RMModule_Cart = 0
RMModule_Arm1 = 1
RMModule_Arm2 = 2
RMModule_Arm3 = 3
RMModule_Arm4 = 4
RMModule_ArmAll = 5

# mode. set by UI
# 0: joint control 1: gravity calibration
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
    # 2: (
    #     "GVL_RMInterfaceData.g_UIArmCmdsData",
    #     ST_RMArmCtrlData,
    # ),
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

# 从手重力标定状态（由 PLC 的 g_slaveGraCaliState 通知更新）。
# -1 表示尚未收到 PLC 状态。
m_slaveGraCaliState = -1

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
    global m_slaveGraCaliState, m_robotModule

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
                # 重力配平模式下，PLC 状态为 2（标定进行中）时开始记录。
                recording = m_RMMode == 1 and m_slaveGraCaliState == 2

                # 遍历这 100 个结构体，逐个放入队列
                for i in range(m_robotDataBufferSize):
                    single_item = value[
                        i
                    ]  # 每一个元素就是一个单独的 ST_RMRobotStatusData 结构体

                    if m_robotStatusDataQueue.full():
                        m_robotStatusDataQueue.get_nowait()  # 队列满时丢弃最老的

                    m_robotStatusDataQueue.put(single_item)

                    # 每 GRA_CALI_RECORD_EVERY 个缓存数据记录 1 个。
                    if recording:
                        # 每个采样保存 4 条臂 x 13 个关节的二维数据。
                        m_graCaliData.m_cmdJntPos.append(
                            [list(arm) for arm in single_item.m_armCmdJntPos]
                        )
                        m_graCaliData.m_curJntPos.append(
                            [list(arm) for arm in single_item.m_armJntPos]
                        )
                        m_graCaliData.m_curJntTrq.append(
                            [list(arm) for arm in single_item.m_armJntTrq]
                        )

            except Exception as e:
                print(f"[Error] Failed to put data to queue: {e}")

            m_robotConnected = True
            m_robotConnectCounts = 0

            current_time = time.perf_counter()
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
            "GVL_RMInterfaceData.g_slaveRobotStatusData", attr, readRobotData
        )
        print(
            "[Connect] Successfully registered notification for 'g_robotStatusData' with decorator."
        )

        # 注册从手重力标定状态通知；旧 PLC 没有该变量时不影响主连接。
        @plc.notification(pyads.PLCTYPE_INT)
        def readSlaveGraCaliState(handle, name, timestamp, value):
            global m_slaveGraCaliState
            m_slaveGraCaliState = int(value)
            print(f"[Callback] g_slaveGraCaliState = {m_slaveGraCaliState}")

        try:
            gra_cali_attr = pyads.NotificationAttrib(
                length=ctypes.sizeof(pyads.PLCTYPE_INT),
                trans_mode=pyads.ADSTRANS_SERVERONCHA,
                max_delay=0,
                cycle_time=10,  # 1000*100ns,
            )
            plc.add_device_notification(
                "GVL_RMInterfaceData.g_slaveGraCaliState",
                gra_cali_attr,
                readSlaveGraCaliState,
            )
            print(
                "[Connect] Successfully registered notification for "
                "'g_slaveGraCaliState'."
            )
        except Exception as e:
            print(f"[Connect] Register g_slaveGraCaliState failed: {e}")

        # 先读取一次当前值，避免 notification 尚未触发时仍为默认值。
        try:
            m_slaveGraCaliState = int(
                plc.read_by_name(
                    "GVL_RMInterfaceData.g_slaveGraCaliState",
                    pyads.PLCTYPE_INT,
                )
            )
        except Exception as e:
            print(f"[Connect] Read g_slaveGraCaliState failed: {e}")

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


def writeGraCaliCmds(cmds):
    """写入从手重力标定命令结构体。

    结构体类型由调用方提供（slaveDynaCaliModule.ST_RMGraCaliCmds），
    因此这里不需要在 slaveRobotInterface 中固定标定结果结构。
    """
    global plc, m_robotCmdsSeq, m_RMMode

    if plc is None or not plc.is_open:
        print("[Warning] PLC is not connected yet.")
        return False

    try:
        plc.write_by_name(
            "GVL_RMInterfaceData.g_RMGraCaliData",
            cmds,
            type(cmds),
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
        print("[UICmds] Written g_RMGraCaliData")
        return True
    except Exception as e:
        print(f"[Error] Failed to write gravity calibration commands: {e}")
        return False
