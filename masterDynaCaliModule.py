import os
import threading
from io import BytesIO

from PyQt5.QtWidgets import (
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPixmap

import masterRobotInterface

# 标定诊断图布局：2 行，每行 6 张。
CALI_PLOT_ROWS = 2
CALI_PLOT_COLUMNS = 6

# 原始标定采样数据文件，放在本脚本所在目录。
GRA_CALI_RAW_DATA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "graCaliData.txt",
)

# 0: 使用 PLC 在线采集的 m_graCaliData。
# 1: 读取 graCaliData.txt 回放，用于离线复现标定解算问题。
testFlag = 0


def readRawData(path=GRA_CALI_RAW_DATA_PATH):
    """读取 graCaliData.txt，并转换为 ST_graCaliData 格式。

    文件每行 42 个数据，顺序为：
    左手指令位置(7) + 左手当前位置(7) + 左手力矩(7) +
    右手指令位置(7) + 右手当前位置(7) + 右手力矩(7)。
    """
    data = masterRobotInterface.ST_graCaliData()

    with open(path, "r", encoding="utf-8") as fp:
        for line_no, line in enumerate(fp, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            values = [float(value) for value in line.split()]
            if len(values) != 42:
                raise ValueError(
                    f"{path}: line {line_no} has {len(values)} values, " "expected 42."
                )

            data.m_leftCmdJntPos.append(values[0:7])
            data.m_leftCurJntPos.append(values[7:14])
            data.m_leftJntTrq.append(values[14:21])
            data.m_rightCmdJntPos.append(values[21:28])
            data.m_rightCurJntPos.append(values[28:35])
            data.m_rightJntTrq.append(values[35:42])

    if not data.m_leftCmdJntPos:
        raise ValueError(f"{path}: no calibration data found.")

    return data


class PlotImageLabel(QLabel):
    """按控件大小等比显示一张静态标定图。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._source_pixmap = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(160, 110)
        self.setStyleSheet("border: 1px solid #d0d7de;" "background-color: #ffffff;")

    def set_plot_pixmap(self, pixmap):
        self._source_pixmap = pixmap
        self._update_pixmap()

    def clear_plot(self):
        self._source_pixmap = None
        self.clear()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_pixmap()

    def _update_pixmap(self):
        if self._source_pixmap is None or self._source_pixmap.isNull():
            return
        self.setPixmap(
            self._source_pixmap.scaled(
                self.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )


class MasterDynaCaliPage(QWidget):
    """主手重力配平分页。"""

    calibration_result_ready = pyqtSignal(object)
    calibration_failed = pyqtSignal(str)
    calibration_plots_ready = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)

        # 重力配平参数结构体。
        self.gra_cali_data = masterRobotInterface.ST_RMGraCaliData()
        for i in range(2):
            for j in range(2):
                self.gra_cali_data.g_trqConstScale[i][j] = 1.0

        self._start_calibration_pressed = False
        self._last_gra_cali_state = int(masterRobotInterface.m_masterGraCaliState)
        self._cali_thread = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(14)

        title = QLabel("<b>主手重力配平</b>")
        title.setStyleSheet("font-size: 16px; color: #2c3e50;")
        layout.addWidget(title)

        button_layout = QHBoxLayout()
        button_layout.setSpacing(12)

        self.btn_home = self._create_button("回零")
        self.btn_start_calibration = self._create_button("开始标定")
        self.btn_verify_calibration = self._create_button("效果验证")
        self.btn_download_calibration = self._create_button("下载标定结果")

        # 按钮按下/松开时，把对应的 BOOL 成员写入结构体并下发。
        self._bind_flag_button(self.btn_home, "m_homeFlag")
        self._bind_flag_button(self.btn_start_calibration, "m_caliFlag")
        self._bind_flag_button(self.btn_verify_calibration, "m_verifyFlag")
        self._bind_flag_button(self.btn_download_calibration, "m_paramDownload")

        # 按下“开始标定”后，开始监控 g_masterGraCaliState 的 2 -> 3 变化。
        self.btn_start_calibration.pressed.connect(self._on_start_calibration_pressed)

        self.calibration_result_ready.connect(self._on_calibration_result_ready)
        self.calibration_failed.connect(self._on_calibration_failed)
        self.calibration_plots_ready.connect(self._on_calibration_plots_ready)

        # 每个按钮下方放置一个状态指示灯。
        self.home_light = self._create_light()
        self.cali_light = self._create_light()
        self.verify_light = self._create_light()
        self.download_light = self._create_light()
        self._download_light_color = "gray"

        button_layout.addWidget(
            self._make_button_column(self.btn_home, self.home_light)
        )
        button_layout.addWidget(
            self._make_button_column(self.btn_start_calibration, self.cali_light)
        )
        button_layout.addWidget(
            self._make_button_column(self.btn_verify_calibration, self.verify_light)
        )
        button_layout.addWidget(
            self._make_button_column(self.btn_download_calibration, self.download_light)
        )
        button_layout.addStretch()

        layout.addLayout(button_layout)

        # 预留后续标定状态显示区域。
        self.lbl_message = QLabel("")
        self.lbl_message.setStyleSheet("color: #7f8c8d;")
        layout.addWidget(self.lbl_message)

        # 标定诊断图：2 行，每行 6 张。
        self.plot_grid = QGridLayout()
        self.plot_grid.setContentsMargins(0, 0, 0, 0)
        self.plot_grid.setSpacing(4)
        self.plot_labels = []

        for row in range(CALI_PLOT_ROWS):
            for col in range(CALI_PLOT_COLUMNS):
                label = PlotImageLabel()
                self.plot_grid.addWidget(label, row, col)
                self.plot_labels.append(label)

        for col in range(CALI_PLOT_COLUMNS):
            self.plot_grid.setColumnStretch(col, 1)
        for row in range(CALI_PLOT_ROWS):
            self.plot_grid.setRowStretch(row, 1)

        layout.addLayout(self.plot_grid, 1)

        self._update_calibration_lights(self._last_gra_cali_state)

        # 监控 PLC 的 g_masterGraCaliState。
        self.state_monitor_timer = QTimer(self)
        self.state_monitor_timer.setInterval(100)
        self.state_monitor_timer.timeout.connect(self._monitor_gra_cali_state)
        self.state_monitor_timer.start()

    @staticmethod
    def _create_button(text):
        button = QPushButton(text)
        button.setCheckable(False)
        button.setCursor(Qt.PointingHandCursor)
        button.setMinimumWidth(130)
        button.setMinimumHeight(40)
        button.setStyleSheet(
            "QPushButton {"
            " padding: 6px 18px;"
            " font-weight: bold;"
            " border: 1px solid #7f8c8d;"
            " border-radius: 4px;"
            " background-color: #ecf0f1;"
            " color: #2c3e50;"
            "}"
            "QPushButton:hover { background-color: #dfe6e9; }"
            "QPushButton:pressed { background-color: #bdc3c7; }"
        )
        return button

    @staticmethod
    def _create_light():
        light = QLabel()
        light.setFixedSize(20, 20)
        light.setStyleSheet(
            "background-color: #bdc3c7;"
            "border-radius: 10px;"
            "border: 1px solid #7f8c8d;"
        )
        return light

    @staticmethod
    def _make_button_column(button, light):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(button)
        layout.addWidget(light, alignment=Qt.AlignHCenter)
        return container

    @staticmethod
    def _set_light(light, color):
        if color == "blue":
            background = "#3498db"
            border = "#2980b9"
        elif color == "green":
            background = "#2ecc71"
            border = "#27ae60"
        elif color == "red":
            background = "#e74c3c"
            border = "#c0392b"
        else:
            background = "#bdc3c7"
            border = "#7f8c8d"

        light.setStyleSheet(
            f"background-color: {background};"
            f"border-radius: 10px;"
            f"border: 1px solid {border};"
        )

    def _update_calibration_lights(self, state):
        """根据 m_masterGraCaliState 更新 4 个按钮指示灯。"""
        state = int(state)

        if state == -1:
            colors = ("gray", "gray", "gray", "gray")
        elif state == 0:
            # 回零过程中 PLC 也会返回 0：只保留上一次下载成功/失败的
            # 绿色/红色结果，其他情况下恢复灰色，不能把下载灯误点亮。
            download_color = (
                self._download_light_color
                if self._download_light_color in ("green", "red")
                else "gray"
            )
            colors = ("blue", "gray", "gray", download_color)
        elif state == 1:
            colors = ("green", "gray", "gray", "gray")
        elif state == 2:
            colors = ("green", "blue", "gray", "gray")
        elif state == 3:
            colors = ("green", "green", "gray", "gray")
        elif state in (4, 5):
            colors = ("green", "green", "green", "gray")
        elif 6 <= state < 100:
            colors = ("green", "green", "green", "blue")
        elif state == 100:
            colors = ("green", "green", "green", "green")
        elif state == 200:
            # 下载标定结果失败。
            colors = ("green", "green", "green", "red")
        else:
            colors = ("gray", "gray", "gray", "gray")

        for index, (light, color) in enumerate(
            zip(
                (
                    self.home_light,
                    self.cali_light,
                    self.verify_light,
                    self.download_light,
                ),
                colors,
            )
        ):
            self._set_light(light, color)
            if index == 3:
                self._download_light_color = color

    def _bind_flag_button(self, button, flag_name):
        """把按钮的按下/松开映射为结构体 BOOL 成员的 True/False。"""
        button.pressed.connect(
            lambda name=flag_name: self._on_button_value_changed(name, True)
        )
        button.released.connect(
            lambda name=flag_name: self._on_button_value_changed(name, False)
        )

    def _on_button_value_changed(self, flag_name, value):
        setattr(self.gra_cali_data, flag_name, bool(value))

        if flag_name == "m_homeFlag" and value:
            # 开始新的回零流程时，清除上一次下载结果指示灯。
            self._download_light_color = "gray"
            self._set_light(self.download_light, "gray")

        # 模式 1：写入 GVL_RMInterfaceData.g_RMGraCaliData。
        if not masterRobotInterface.writeUICmds(self.gra_cali_data):
            print(
                f"[DynaCali] Failed to write {flag_name}={value} " "to g_RMGraCaliData"
            )

    def _on_start_calibration_pressed(self):
        """按下开始标定后，开始监控状态 2 -> 3。"""
        self._start_calibration_pressed = True
        self._last_gra_cali_state = int(masterRobotInterface.m_masterGraCaliState)
        self._update_calibration_lights(self._last_gra_cali_state)
        self.set_message("等待标定状态进入 3 ...")

    def reset_calibration_lights(self):
        """切换到本页时，把所有指示灯重置为灰色。"""
        self._last_gra_cali_state = int(masterRobotInterface.m_masterGraCaliState)
        self._update_calibration_lights(-1)

    def _monitor_gra_cali_state(self):
        current_state = int(masterRobotInterface.m_masterGraCaliState)
        previous_state = self._last_gra_cali_state
        self._last_gra_cali_state = current_state

        # 状态未变化时不重复刷新，保证页面切换后的灰色重置能保持到下一次状态变化。
        if current_state == previous_state:
            return

        self._update_calibration_lights(current_state)

        if (
            self._start_calibration_pressed
            and previous_state == 2
            and current_state == 3
            and masterRobotInterface.m_RMMode == 1
        ):
            self._start_calibration_pressed = False
            self._start_master_gra_cali()

    def _start_master_gra_cali(self):
        if self._cali_thread is not None and self._cali_thread.is_alive():
            self.set_message("上一次重力参数辨识尚未完成。")
            return

        self._clear_plot_images()
        self.set_message("正在辨识重力参数 ...")

        self._cali_thread = threading.Thread(
            target=self._run_master_gra_cali,
            name="MasterGraCaliUI",
            daemon=True,
        )
        self._cali_thread.start()

    @staticmethod
    def _get_master_gra_cali():
        """根据 PLC 的 g_robotModule 选择主手重力标定算法。"""
        robot_module = int(masterRobotInterface.m_robotModule)

        if robot_module in (2, 3):
            from masterGraCali_NewMaster import masterGraCali

            print(
                f"[DynaCali] robotModule={robot_module}, "
                "using masterGraCali_NewMaster."
            )
            return masterGraCali

        if robot_module == 4:
            from masterGraCali_NewWrist import masterGraCali

            print(
                f"[DynaCali] robotModule={robot_module}, "
                "using masterGraCali_NewWrist."
            )
            return masterGraCali

        if robot_module == 5:
            from masterGraCali_OldMaster import masterGraCali

            print(
                f"[DynaCali] robotModule={robot_module}, "
                "using masterGraCali_OldMaster."
            )
            return masterGraCali

        raise ValueError(
            "Unsupported robotModule for master gravity calibration: "
            f"{robot_module}. Supported: 2/3 -> NewMaster, "
            "4 -> NewWrist, 5 -> OldMaster."
        )

    def _run_master_gra_cali(self):
        try:
            master_gra_cali = self._get_master_gra_cali()

            if testFlag == 0:
                # 在线模式：先落盘原始采样数据，便于复现和定位解算错误。
                self.writeRawData()
                cali_data = masterRobotInterface.m_graCaliData
            elif testFlag == 1:
                # 离线回放模式：读取已保存的原始数据，不能覆盖该文件。
                cali_data = readRawData()
                print(
                    f"[DynaCali] Loaded {len(cali_data.m_leftCmdJntPos)} "
                    f"raw samples from {GRA_CALI_RAW_DATA_PATH}"
                )
            else:
                raise ValueError(f"Invalid testFlag: {testFlag}")

            result, plot_images = self._call_master_gra_cali(
                master_gra_cali,
                cali_data,
            )

            if plot_images:
                self.calibration_plots_ready.emit(plot_images)
            self.calibration_result_ready.emit(result)
        except Exception as exc:
            self.calibration_failed.emit(str(exc))

    def _call_master_gra_cali(self, master_gra_cali, cali_data):
        """在线程中生成标定诊断图，并返回结果和 PNG 图像列表。"""
        import matplotlib

        # 后台线程里必须使用非交互后端，否则 QtAgg 无法创建窗口。
        matplotlib.use("Agg", force=True)

        import matplotlib.pyplot as plt

        plt.close("all")
        original_show = plt.show
        plt.show = lambda *args, **kwargs: None

        try:
            result = master_gra_cali(cali_data)
            figures = [plt.figure(num) for num in plt.get_fignums()]
            images = [self._figure_to_png(figure) for figure in figures]
        finally:
            plt.show = original_show
            plt.close("all")

        return result, images

    @staticmethod
    def _figure_to_png(figure):
        """把 matplotlib Figure 渲染成 PNG 字节数据。"""
        figure.set_size_inches(4.0, 3.0)
        try:
            figure.tight_layout()
        except Exception:
            pass

        buffer = BytesIO()
        figure.savefig(
            buffer,
            format="png",
            dpi=100,
            bbox_inches="tight",
        )
        return buffer.getvalue()

    def _on_calibration_plots_ready(self, images):
        """在主线程中把标定诊断图显示到页面网格里。"""
        self._clear_plot_images()

        if not images:
            return

        if len(images) != len(self.plot_labels):
            print(
                f"[DynaCali] Got {len(images)} calibration plots, "
                f"expected {len(self.plot_labels)}."
            )

        for label, png in zip(self.plot_labels, images):
            pixmap = QPixmap()
            if pixmap.loadFromData(png, "PNG"):
                label.set_plot_pixmap(pixmap)
            else:
                print("[DynaCali] Failed to decode a calibration plot.")

    def _clear_plot_images(self):
        for label in self.plot_labels:
            label.clear_plot()

    def writeRawData(self):
        """把 m_graCaliData 的原始采样写入 graCaliData.txt。

        每行 42 个数据，顺序为：
        左手指令位置(7) + 左手当前位置(7) + 左手力矩(7) +
        右手指令位置(7) + 右手当前位置(7) + 右手力矩(7)。
        """
        data = masterRobotInterface.m_graCaliData

        # 与 masterGraCali() 中一样先做快照，避免写入过程中采样数据继续追加。
        series = [
            list(data.m_leftCmdJntPos),
            list(data.m_leftCurJntPos),
            list(data.m_leftJntTrq),
            list(data.m_rightCmdJntPos),
            list(data.m_rightCurJntPos),
            list(data.m_rightJntTrq),
        ]
        lengths = [len(item) for item in series]
        sample_count = min(lengths)

        if sample_count == 0:
            print("[DynaCali] No raw calibration data to write.")
            return False

        if len(set(lengths)) != 1:
            # 采样回调可能正在追加最后一条，只写各数组都完整存在的前 n 条。
            print(
                f"[DynaCali] Raw data lengths mismatch: {lengths}; "
                f"writing first {sample_count} samples."
            )

        try:
            with open(
                GRA_CALI_RAW_DATA_PATH,
                "w",
                encoding="utf-8",
                newline="\n",
            ) as fp:
                for i in range(sample_count):
                    row = []
                    for item in series:
                        values = item[i]
                        if len(values) != 7:
                            raise ValueError(
                                f"Sample {i} has {len(values)} values, " "expected 7."
                            )
                        row.extend(float(value) for value in values)

                    fp.write(" ".join(repr(float(value)) for value in row))
                    fp.write("\n")
        except Exception as exc:
            print(f"[DynaCali] Failed to write raw data: {exc}")
            return False

        print(
            f"[DynaCali] Wrote {sample_count} raw samples "
            f"to {GRA_CALI_RAW_DATA_PATH}"
        )
        return True

    def _on_calibration_result_ready(self, result):
        self.gra_cali_data = result
        self._update_calibration_lights(int(masterRobotInterface.m_masterGraCaliState))
        self.set_message("重力参数辨识完成。")

    def _on_calibration_failed(self, message):
        self._update_calibration_lights(int(masterRobotInterface.m_masterGraCaliState))
        self.set_message(f"重力参数辨识失败：{message}")

    def set_message(self, text):
        self.lbl_message.setText(text)
