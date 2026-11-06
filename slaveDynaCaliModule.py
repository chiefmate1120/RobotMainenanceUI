import threading
from io import BytesIO

from PyQt5.QtWidgets import (
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QTabWidget,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPixmap

import slaveRobotInterface
from slaveJntCtrlModule import apply_tab_bar_style

# 标定诊断图布局：1 行 2 张，仅显示 _plot_fit_results 生成的
# 重力拟合图和摩擦拟合图。
CALI_PLOT_ROWS = 1
CALI_PLOT_COLUMNS = 2


class PlotImageLabel(QLabel):
    """按控件大小等比显示一张静态标定图。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._source_pixmap = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(160, 110)
        self.setStyleSheet(
            "border: 1px solid #d0d7de;"
            "background-color: #ffffff;"
        )

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


class ArmGraCaliPage(QWidget):
    """单条从手臂的重力标定子页面。"""

    calibration_result_ready = pyqtSignal(object)
    calibration_failed = pyqtSignal(str)
    calibration_plots_ready = pyqtSignal(object)

    def __init__(self, arm_idx, parent=None):
        super().__init__(parent)

        # 手臂编号（1~4）。
        self.arm_idx = int(arm_idx)

        # 重力标定参数结构体：4 个按钮标志 + 4 臂 x 4 标定系数。
        # 切换子页面或点击按钮时，整个结构体写入 g_RMGraCaliData。
        self.gra_cali_data = slaveRobotInterface.ST_RMGraCaliData()
        self.cali_result = None

        self._start_calibration_pressed = False
        self._last_gra_cali_state = int(
            getattr(slaveRobotInterface, "m_slaveGraCaliState", -1)
        )
        self._cali_thread = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(14)

        button_layout = QHBoxLayout()
        button_layout.setSpacing(12)

        self.btn_home = self._create_button("回零")
        self.btn_start_calibration = self._create_button("开始标定")
        self.btn_verify_calibration = self._create_button("效果验证")
        self.btn_download_calibration = self._create_button("下载标定结果")

        self._bind_flag_button(self.btn_home, "m_homeFlag")
        self._bind_flag_button(self.btn_start_calibration, "m_caliFlag")
        self._bind_flag_button(self.btn_verify_calibration, "m_verifyFlag")
        self._bind_flag_button(self.btn_download_calibration, "m_paramDownload")

        self.btn_start_calibration.pressed.connect(self._on_start_calibration_pressed)

        self.calibration_result_ready.connect(self._on_calibration_result_ready)
        self.calibration_failed.connect(self._on_calibration_failed)
        self.calibration_plots_ready.connect(self._on_calibration_plots_ready)

        self.home_light = self._create_light()
        self.cali_light = self._create_light()
        self.verify_light = self._create_light()
        self.download_light = self._create_light()
        self._download_light_color = "gray"

        button_layout.addWidget(self._make_button_column(self.btn_home, self.home_light))
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

        self.lbl_message = QLabel("")
        self.lbl_message.setStyleSheet("color: #7f8c8d;")
        layout.addWidget(self.lbl_message)

        # 标定诊断图：1 行 2 张（重力拟合图 / 摩擦拟合图）。
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
        """根据 m_slaveGraCaliState 更新 4 个按钮指示灯。"""
        state = int(state)

        if state == -1:
            colors = ("gray", "gray", "gray", "gray")
        elif state == 0:
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
        button.pressed.connect(
            lambda name=flag_name: self._on_button_value_changed(name, True)
        )
        button.released.connect(
            lambda name=flag_name: self._on_button_value_changed(name, False)
        )

    def write_gra_cali_data(self):
        """把当前手臂的 g_RMGraCaliData 结构体写入 PLC。"""
        return slaveRobotInterface.writeGraCaliCmds(self.gra_cali_data)

    def reset_calibration_commands(self):
        """清零上位机标定按钮对应的 4 个 PLC 命令标志。"""
        for flag_name in (
            "m_homeFlag",
            "m_caliFlag",
            "m_verifyFlag",
            "m_paramDownload",
        ):
            setattr(self.gra_cali_data, flag_name, False)

        self._start_calibration_pressed = False
        self._download_light_color = "gray"
        self._set_light(self.download_light, "gray")

    def _on_button_value_changed(self, flag_name, value):
        setattr(self.gra_cali_data, flag_name, bool(value))

        if flag_name == "m_homeFlag" and value:
            self._download_light_color = "gray"
            self._set_light(self.download_light, "gray")

        if not self.write_gra_cali_data():
            print(
                f"[SlaveDynaCali] Failed to write {flag_name}={value} "
                "to g_RMGraCaliData"
            )

    def _on_start_calibration_pressed(self):
        """按下开始标定后，开始监控状态 2 -> 3。"""
        self._start_calibration_pressed = True
        self._last_gra_cali_state = int(
            getattr(slaveRobotInterface, "m_slaveGraCaliState", -1)
        )
        self._update_calibration_lights(self._last_gra_cali_state)
        self.set_message("等待标定状态进入 3 ...")

        # 如果当前 PLC 还没有 g_slaveGraCaliState 接口，直接触发一次标定，
        # 便于单独调试 setupJoint3GraCali.run。
        if self._last_gra_cali_state < 0:
            self._start_calibration_pressed = False
            self.set_message("未检测到 PLC 标定状态，直接开始标定 ...")
            self._start_slave_gra_cali()

    def reset_calibration_lights(self):
        """切换到本页时，把所有指示灯重置为灰色。"""
        self._last_gra_cali_state = int(
            getattr(slaveRobotInterface, "m_slaveGraCaliState", -1)
        )
        self._update_calibration_lights(-1)

    def _monitor_gra_cali_state(self):
        current_state = int(
            getattr(slaveRobotInterface, "m_slaveGraCaliState", -1)
        )
        previous_state = self._last_gra_cali_state
        self._last_gra_cali_state = current_state

        if current_state == previous_state:
            return

        self._update_calibration_lights(current_state)

        if (
            self._start_calibration_pressed
            and previous_state == 2
            and current_state == 3
            and slaveRobotInterface.m_RMMode == 1
        ):
            self._start_calibration_pressed = False
            self._start_slave_gra_cali()

    def _start_slave_gra_cali(self):
        if self._cali_thread is not None and self._cali_thread.is_alive():
            self.set_message("上一次重力参数辨识尚未完成。")
            return

        self._clear_plot_images()
        self.set_message("正在辨识重力参数 ...")

        self._cali_thread = threading.Thread(
            target=self._run_slave_gra_cali,
            name="SlaveDynaCaliUI",
            daemon=True,
        )
        self._cali_thread.start()

    @staticmethod
    def _get_slave_gra_cali():
        """返回 setupJoint3GraCali.run。"""
        try:
            from setupJoint3GraCali import run
        except ImportError as exc:
            raise RuntimeError(
                "setupJoint3GraCali.run is not available: "
                f"{exc}"
            ) from exc

        return run

    @staticmethod
    def _get_gra_cali_data():
        """获取 setup joint 3 的标定采样数据。"""
        provider = getattr(slaveRobotInterface, "getGraCaliData", None)
        if callable(provider):
            return provider()

        data = getattr(slaveRobotInterface, "m_graCaliData", None)
        if data is None:
            raise RuntimeError(
                "slaveRobotInterface.m_graCaliData is not available; "
                "cannot run setup joint 3 gravity calibration."
            )
        return data

    def _run_slave_gra_cali(self):
        try:
            cali_func = self._get_slave_gra_cali()
            result, plot_images = self._call_slave_gra_cali(cali_func)

            if plot_images:
                self.calibration_plots_ready.emit(plot_images)
            self.calibration_result_ready.emit(result)
        except Exception as exc:
            self.calibration_failed.emit(str(exc))

    def _call_slave_gra_cali(self, cali_func):
        """在线程中调用 setupJoint3GraCali.run，并返回结果和 PNG 图像列表。"""
        import matplotlib

        matplotlib.use("Agg", force=True)

        import matplotlib.pyplot as plt

        plt.close("all")
        original_show = plt.show
        plt.show = lambda *args, **kwargs: None

        try:
            gra_cali_data = self._get_gra_cali_data()
            result = cali_func(gra_cali_data, self.arm_idx, show_plots=True)
            figures = [plt.figure(num) for num in plt.get_fignums()]

            # run 中先由 caliDataProcess 生成过程图，最后才是
            # _plot_fit_results 生成的重力/摩擦拟合图；只取最后两张。
            figures = figures[-2:]
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
        """在主线程中把两张拟合图显示到当前手臂子页面。"""
        self._clear_plot_images()

        if not images:
            return

        if len(images) != len(self.plot_labels):
            print(
                f"[SlaveDynaCali] Got {len(images)} calibration plots, "
                f"expected {len(self.plot_labels)}."
            )

        for label, png in zip(self.plot_labels, images):
            pixmap = QPixmap()
            if pixmap.loadFromData(png, "PNG"):
                label.set_plot_pixmap(pixmap)
            else:
                print("[SlaveDynaCali] Failed to decode a calibration plot.")

    def _clear_plot_images(self):
        for label in self.plot_labels:
            label.clear_plot()

    def _on_calibration_result_ready(self, result):
        self.cali_result = result

        gravity_coeffs = list(result.get("gravityCoeffs", []))
        friction_coeffs = list(result.get("frictionCoeffs", []))
        if len(gravity_coeffs) != 4 or len(friction_coeffs) != 4:
            self.set_message("标定结果长度异常，未写入 PLC。")
            return

        # 只保留重力/摩擦系数，并写入当前手臂对应的行。
        arm_row = self.arm_idx - 1
        for i in range(4):
            self.gra_cali_data.m_setupJnt3GravityParams[arm_row][i] = float(
                gravity_coeffs[i]
            )
            self.gra_cali_data.m_setupJnt3FrictionParams[arm_row][i] = float(
                friction_coeffs[i]
            )

        if self.write_gra_cali_data():
            self.set_message("重力参数辨识完成，标定结果已写入 PLC。")
        else:
            self.set_message("重力参数辨识完成，但写入 PLC 失败。")

        self._update_calibration_lights(
            int(getattr(slaveRobotInterface, "m_slaveGraCaliState", -1))
        )

    def _on_calibration_failed(self, message):
        self._update_calibration_lights(
            int(getattr(slaveRobotInterface, "m_slaveGraCaliState", -1))
        )
        self.set_message(f"重力参数辨识失败：{message}")

    def set_message(self, text):
        self.lbl_message.setText(text)


class SlaveDynaCaliPage(QWidget):
    """从手重力标定分页：1~4 号臂子页面。"""

    # 切换手臂子页面时请求主窗口关闭“上位机控制”。
    control_disable_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.arm_tabs = QTabWidget(self)
        self.arm_tabs.setDocumentMode(True)
        self.arm_tabs.setMovable(False)

        self.arm_pages = []
        for arm_idx in range(1, 5):
            page = ArmGraCaliPage(arm_idx, self)
            self.arm_pages.append(page)
            self.arm_tabs.addTab(page, f"{arm_idx}号臂")

        self.arm_tabs.currentChanged.connect(self.on_arm_tab_changed)

        # 与关节控制分页保持相同的子分页按钮风格。
        apply_tab_bar_style(self.arm_tabs)

        layout.addWidget(self.arm_tabs)

    def on_arm_tab_changed(self, index):
        """切换手臂子页面时，关闭上位机控制并更新 PLC 手臂号。"""
        # 与关节控制分页一样，切换手臂时清除“上位机控制”。
        self.control_disable_requested.emit()

        module_idx = slaveRobotInterface.RMModule_Arm1 + int(index)
        slaveRobotInterface.writeUIEnable(
            slaveRobotInterface.m_UICtrlEnable,
            module_idx,
        )

        # 切换手臂时清零上位机标定按钮，避免把上一次的命令标志带到新手臂。
        if 0 <= index < len(self.arm_pages):
            self.arm_pages[index].reset_calibration_commands()

        self.write_current_gra_cali_data()

    def write_current_gra_cali_data(self):
        """把当前手臂子页面的结构体写入 PLC。"""
        index = self.arm_tabs.currentIndex()
        if 0 <= index < len(self.arm_pages):
            if not self.arm_pages[index].write_gra_cali_data():
                print(
                    f"[SlaveDynaCali] Failed to write g_RMGraCaliData "
                    f"for arm {index + 1}"
                )

    def current_module_index(self):
        """返回当前子页面对应的 RMModule：1~4 号臂。"""
        return slaveRobotInterface.RMModule_Arm1 + int(
            self.arm_tabs.currentIndex()
        )

    def reset_calibration_lights(self):
        """切换到重力配平分页时，把所有手臂子页的指示灯重置为灰色。"""
        for page in self.arm_pages:
            page.reset_calibration_lights()
