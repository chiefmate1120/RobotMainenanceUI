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


# enable UI control, transit PLC state machine
# set by UI
m_UICtrlEnable = False

# mode. set by UI
# 1: joint control
m_RMMode = 0
ADSDataConfig = {
    0: (
        "GVL_RMInterfaceData.g_RMJntCtrlData",
        ST_RMJntCtrlData,
    ),
    # 1: (
    #     "GVL_RMInterfaceData.g_UICartCmdsData",
    #     ST_RMCartCtrlData,
    # ),
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


lastTime = time.perf_counter()


def connect(ams_net_id):
    res = False
    """
    负责和 TwinCAT 建立通信，并注册 g_robotStatusData 变量的读取回调
    """
    global plc, m_robotConnected, m_robotStatusDataQueue, lastTime
    try:
        ADS_PORT = 851
        # get ip
        parts = ams_net_id.split(".")
        PLC_IP = ".".join(parts[:4]) if len(parts) >= 4 else ams_net_id

        plc = pyads.Connection(ams_net_id, ADS_PORT, PLC_IP)
        plc.open()
        print(f"[Connect] Successfully connected to TwinCAT, AMS Net ID: {ams_net_id}")

        # 定义带有 pyads.notification 装饰器的回调函数
        # 传入整个 ARRAY[1..100] 的 ctypes 类型，pyads 会自动解析并传给 value
        @plc.notification(ST_RMRobotStatusData * m_robotDataBufferSize)
        def readRobotData(handle, name, timestamp, value):
            global m_robotStatusDataQueue, m_robotConnected, m_robotConnectCounts, lastTime
            try:
                # 遍历这 100 个结构体，逐个放入队列
                for i in range(m_robotDataBufferSize):
                    single_item = value[
                        i
                    ]  # 每一个元素就是一个单独的 ST_RMRobotStatusData 结构体

                    if m_robotStatusDataQueue.full():
                        m_robotStatusDataQueue.get_nowait()  # 队列满时丢弃最老的

                    m_robotStatusDataQueue.put(single_item)

                print(
                    f"[Callback] Successfully split and put 100 robot structures into queue at {timestamp}"
                )
            except Exception as e:
                print(f"[Error] Failed to put data to queue: {e}")

            m_robotConnected = True
            m_robotConnectCounts = 0
            print(
                f"[Callback] g_robotStatusData updated automatically at timestamp: {timestamp}"
            )

            current_time = time.perf_counter()
            interval_ms = (current_time - lastTime) * 1000.0  # 转换为毫秒
            print(f"[Callback Interval] Time since last callback: {interval_ms:.2f} ms")
            print(
                f"[first pos]: {value[0].m_armJntPos[0][6]}, [last Pos]: {value[99].m_armJntPos[0][6]}"
            )

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


def writeUIEnable(enableFlag):
    global plc, m_UICtrlEnable

    if plc is None or not plc.is_open:
        print("[Warning] PLC is not connected yet.")
        return False

    try:
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
        print(f"[UICtrlModule] Written g_UICtrlModule = {module_idx}")
        return True
    except Exception as e:
        print(f"[Error] Failed to write UI control module index: {e}")
        return False


def writeUICmds(cmds, mode):
    global plc, m_robotCmdsSeq, m_RMMode

    if plc is None or not plc.is_open:
        print("[Warning] PLC is not connected yet.")
        return False

    # mode check
    config = ADSDataConfig.get(mode)
    if config is None:
        print(f"[Error] Invalid command mode: {mode}")
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
