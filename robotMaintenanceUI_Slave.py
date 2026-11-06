import sys

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QMessageBox,
    QTabWidget,
)
from PyQt5.QtCore import QTimer, Qt

import slaveRobotInterface
from slaveJntCtrlModule import SlaveJntCtrlPage, apply_tab_bar_style
from slaveDynaCaliModule import SlaveDynaCaliPage

# =========================================================
# Configuration
# =========================================================

DEFAULT_AMS_NET_ID = "169.254.191.207.1.1"
UI_INTERVAL_MS = 20
HEARTBEAT_INTERVAL_MS = 1000


class RobotMainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        self._closing = False
        self._connecting = False
        self.jnt_page = None
        self.btn_upper_control = None

        self.setWindowTitle("Robot TwinCAT Control & Monitor Interface")

        screen = QApplication.primaryScreen()
        if screen is not None:
            self.setGeometry(screen.availableGeometry())

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # -------------------------------------------------
        # 1. ADS 连接
        # -------------------------------------------------
        top_layout = QHBoxLayout()

        top_layout.addWidget(QLabel("AMS Net ID:"))

        self.input_ams = QLineEdit(DEFAULT_AMS_NET_ID)
        self.input_ams.setFixedWidth(270)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.on_connect_clicked)

        self.indicator_light = QLabel()
        self.indicator_light.setFixedSize(25, 25)

        top_layout.addWidget(self.input_ams)
        top_layout.addWidget(self.btn_connect)
        top_layout.addWidget(self.indicator_light)
        top_layout.addStretch()

        main_layout.addLayout(top_layout)

        # -------------------------------------------------
        # 2. 分页
        # -------------------------------------------------
        self.tabs = QTabWidget(self)
        self.tabs.setDocumentMode(True)
        self.tabs.setMovable(False)

        self.jnt_page = SlaveJntCtrlPage(self)
        self.dyna_cali_page = SlaveDynaCaliPage(self)

        self.tabs.addTab(self.jnt_page, "关节状态和控制")
        self.tabs.addTab(self.dyna_cali_page, "从手重力配平")

        apply_tab_bar_style(self.tabs)

        # 子分页切换时，自动关闭上位机控制。
        self.jnt_page.control_disable_requested.connect(self._disable_upper_control)
        self.dyna_cali_page.control_disable_requested.connect(
            self._disable_upper_control
        )
        self.tabs.currentChanged.connect(self._on_main_tab_changed)

        # 初始分页为关节控制。
        slaveRobotInterface.setRMMode(0)

        # “上位机控制”按键保留在主窗口。
        self.btn_upper_control = QPushButton("上位机控制")
        self.btn_upper_control.setCheckable(True)
        self.btn_upper_control.setChecked(False)
        self.btn_upper_control.setCursor(Qt.PointingHandCursor)
        self.btn_upper_control.setMinimumWidth(120)
        self.btn_upper_control.setStyleSheet(
            "QPushButton {"
            " padding: 5px 16px;"
            " font-weight: bold;"
            " border: 1px solid #7f8c8d;"
            " border-radius: 4px;"
            " background-color: #ecf0f1;"
            " color: #2c3e50;"
            "}"
            "QPushButton:hover { background-color: #dfe6e9; }"
            "QPushButton:pressed { background-color: #bdc3c7; }"
            "QPushButton:checked {"
            " background-color: #2ecc71;"
            " color: white;"
            " border: 1px solid #27ae60;"
            "}"
        )
        self.btn_upper_control.toggled.connect(self.on_upper_control_toggled)

        corner_widget = QWidget()
        corner_layout = QHBoxLayout(corner_widget)
        corner_layout.setContentsMargins(0, 0, 8, 0)
        corner_layout.addWidget(self.btn_upper_control)
        self.tabs.setCornerWidget(corner_widget, Qt.TopRightCorner)

        base_button_height = self.btn_upper_control.sizeHint().height()
        self.btn_upper_control.setMinimumHeight(max(base_button_height * 2, 40))

        main_layout.addWidget(self.tabs)

        # -------------------------------------------------
        # 3. 定时器
        # -------------------------------------------------
        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.setInterval(HEARTBEAT_INTERVAL_MS)
        self.heartbeat_timer.timeout.connect(self.write_heartbeat)

        self.ui_timer = QTimer(self)
        self.ui_timer.setTimerType(Qt.PreciseTimer)
        self.ui_timer.setInterval(UI_INTERVAL_MS)
        self.ui_timer.timeout.connect(self.update_indicator)
        self.ui_timer.start()

        # 初始连接状态。
        self.btn_upper_control.setEnabled(bool(slaveRobotInterface.m_robotConnected))
        self.set_indicator_status(bool(slaveRobotInterface.m_robotConnected))

    # =====================================================
    # 上位机控制
    # =====================================================

    def on_upper_control_toggled(self, checked):
        if self.jnt_page is not None:
            self.jnt_page.set_control_enabled(bool(checked))
        slaveRobotInterface.writeUIEnable(bool(checked), self._current_module_index())

    def _disable_upper_control(self):
        if self.btn_upper_control is not None and self.btn_upper_control.isChecked():
            self.btn_upper_control.setChecked(False)

    def _on_main_tab_changed(self, index):
        # 关节控制：m_RMMode=0；重力配平：m_RMMode=1。
        if index == 0:
            slaveRobotInterface.setRMMode(0)
        elif index == 1:
            slaveRobotInterface.setRMMode(1)

            # 每次切换到重力配平分页时，指示灯先全部置灰，
            # 并把当前手臂的结构体写入 PLC。
            self.dyna_cali_page.reset_calibration_lights()
            self.dyna_cali_page.write_current_gra_cali_data()

        # 任何页面切换都同步当前模块编号。
        slaveRobotInterface.writeModuleIdx(self._current_module_index())

        # 切换到重力配平分页时，关闭关节控制。
        self._disable_upper_control()

    def _current_module_index(self):
        """返回当前页面使用的 RMModule 枚举值。"""
        if self.tabs.currentIndex() == 0 and self.jnt_page is not None:
            return self.jnt_page.current_module_index()

        # 从手重力配平分页跟随当前手臂子页面。
        if self.dyna_cali_page is not None:
            return self.dyna_cali_page.current_module_index()

        return slaveRobotInterface.RMModule_ArmAll

    # =====================================================
    # 连接指示灯
    # =====================================================

    def set_indicator_status(self, is_connected):
        """指示灯与 m_robotConnected 绑定；未连接时禁用上位机控制。"""
        if is_connected:
            self.indicator_light.setStyleSheet(
                "background-color: #2ecc71;"
                "border-radius: 12px;"
                "border: 1px solid #27ae60;"
            )
        else:
            self.indicator_light.setStyleSheet(
                "background-color: #bdc3c7;"
                "border-radius: 12px;"
                "border: 1px solid #7f8c8d;"
            )

        if self.btn_upper_control is not None:
            if is_connected:
                self.btn_upper_control.setEnabled(True)
            else:
                if self.btn_upper_control.isChecked():
                    self.btn_upper_control.setChecked(False)
                self.btn_upper_control.setEnabled(False)

    # =====================================================
    # ADS 连接
    # =====================================================

    def on_connect_clicked(self):
        if self._connecting:
            return

        if slaveRobotInterface.m_robotConnected:
            QMessageBox.information(
                self,
                "Information",
                "Already connected to TwinCAT.",
            )
            return

        ams_id = self.input_ams.text().strip()

        if not ams_id:
            QMessageBox.warning(
                self,
                "Warning",
                "Please enter a valid AMS Net ID!",
            )
            return

        self._connecting = True
        self.btn_connect.setEnabled(False)

        try:
            success = slaveRobotInterface.connect(ams_id)
        except Exception as exc:
            success = False
            print(f"[UI] Connection exception: {exc}")
        finally:
            self._connecting = False
            self.btn_connect.setEnabled(True)

        if success:
            self.set_indicator_status(bool(slaveRobotInterface.m_robotConnected))

            # 连接成功后同步当前页面模块编号。
            slaveRobotInterface.writeModuleIdx(self._current_module_index())

            self.heartbeat_timer.start()

            QMessageBox.information(
                self,
                "Success",
                "Connected to TwinCAT successfully!",
            )
        else:
            self.set_indicator_status(False)
            QMessageBox.critical(
                self,
                "Error",
                "Failed to connect to TwinCAT. " "Check AMS Net ID and PLC status.",
            )

    # =====================================================
    # 心跳
    # =====================================================

    def write_heartbeat(self):
        if self._closing:
            return

        if not slaveRobotInterface.m_robotConnected:
            return

        try:
            slaveRobotInterface.writeHeartBeat()
        except Exception as exc:
            print(f"[UI] Heartbeat write failed: {exc}")

    # =====================================================
    # UI 更新
    # =====================================================

    def update_indicator(self):
        if self._closing:
            return

        self.set_indicator_status(bool(slaveRobotInterface.m_robotConnected))

    # =====================================================
    # 关闭窗口
    # =====================================================

    def closeEvent(self, event):
        self._closing = True
        self.ui_timer.stop()
        self.heartbeat_timer.stop()

        if self.jnt_page is not None:
            self.jnt_page.stop_timer()

        disconnect_func = getattr(
            slaveRobotInterface,
            "disconnect",
            None,
        )

        if callable(disconnect_func):
            try:
                result = disconnect_func()
                if result is False:
                    self._closing = False
                    QMessageBox.warning(
                        self,
                        "Warning",
                        "ADS resources have not been released "
                        "successfully. Please try closing again.",
                    )
                    self.ui_timer.start(UI_INTERVAL_MS)
                    if slaveRobotInterface.m_robotConnected:
                        self.heartbeat_timer.start(HEARTBEAT_INTERVAL_MS)
                    event.ignore()
                    return
            except Exception as exc:
                self._closing = False
                QMessageBox.critical(
                    self,
                    "Error",
                    f"Failed to disconnect from TwinCAT:\n{exc}",
                )
                self.ui_timer.start(UI_INTERVAL_MS)
                if slaveRobotInterface.m_robotConnected:
                    self.heartbeat_timer.start(HEARTBEAT_INTERVAL_MS)
                event.ignore()
                return

        event.accept()


# =========================================================
# Program entry
# =========================================================

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotMainWindow()
    window.showMaximized()
    sys.exit(app.exec_())
