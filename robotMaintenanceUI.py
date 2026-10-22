import sys
import queue
import time
from collections import deque

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
)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg

import robotInterface

# =========================================================
# Configuration
# =========================================================

DEFAULT_AMS_NET_ID = "192.168.0.30.1.1"

UI_INTERVAL_MS = 20
HEARTBEAT_INTERVAL_MS = 1000

MAX_PLOT_POINTS = 10000
MAX_SAMPLES_PER_UPDATE = 500

ROBOT_INDEX = 0
JOINT_INDEX = 6

SCOPE_WINDOW_POINTS = MAX_PLOT_POINTS


class RobotMainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        self.slaveRobotData = robotInterface.ST_RMRobotStatusData()

        self.setWindowTitle("Robot TwinCAT Control & Monitor Interface")
        self.resize(1100, 750)

        self._closing = False
        self._connecting = False
        self.scope_running = False

        self.time_data = deque(maxlen=MAX_PLOT_POINTS)
        self.value_data = deque(maxlen=MAX_PLOT_POINTS)
        self.counter_x = 0

        # 用户是否手动缩放或平移过视图
        self._manual_view = False

        # =================================================
        # 1. Main layout
        # =================================================

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)

        # =================================================
        # 2. ADS connection
        # =================================================

        top_layout = QHBoxLayout()

        top_layout.addWidget(QLabel("AMS Net ID:"))

        self.input_ams = QLineEdit(DEFAULT_AMS_NET_ID)
        self.input_ams.setFixedWidth(180)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.on_connect_clicked)

        self.lbl_status = QLabel("Disconnected")

        self.indicator_light = QLabel()
        self.indicator_light.setFixedSize(25, 25)
        self.set_indicator_status(False)

        top_layout.addWidget(self.input_ams)
        top_layout.addWidget(self.btn_connect)
        top_layout.addStretch()
        top_layout.addWidget(QLabel("ADS Status:"))
        top_layout.addWidget(self.lbl_status)
        top_layout.addWidget(self.indicator_light)

        main_layout.addLayout(top_layout)

        # =================================================
        # 3. Scope controls
        # =================================================

        main_layout.addWidget(
            QLabel(
                f"<b>Real-time Oscilloscope "
                f"(Robot {ROBOT_INDEX}, "
                f"Joint {JOINT_INDEX})</b>"
            )
        )

        scope_layout = QHBoxLayout()

        self.btn_start = QPushButton("Start")
        self.btn_stop = QPushButton("Stop")

        self.btn_x_zoom_in = QPushButton("X Zoom +")
        self.btn_x_zoom_out = QPushButton("X Zoom -")
        self.btn_auto_range = QPushButton("Auto Range")

        self.btn_start.clicked.connect(self.start_scope)
        self.btn_stop.clicked.connect(self.stop_scope)

        self.btn_x_zoom_in.clicked.connect(lambda: self.zoom_x(0.5))
        self.btn_x_zoom_out.clicked.connect(lambda: self.zoom_x(2.0))
        self.btn_auto_range.clicked.connect(self.auto_range)

        self.btn_stop.setEnabled(False)

        self.lbl_scope_status = QLabel("Scope stopped")

        scope_layout.addWidget(self.btn_start)
        scope_layout.addWidget(self.btn_stop)
        scope_layout.addSpacing(15)
        scope_layout.addWidget(self.btn_x_zoom_in)
        scope_layout.addWidget(self.btn_x_zoom_out)
        scope_layout.addWidget(self.btn_auto_range)
        scope_layout.addStretch()
        scope_layout.addWidget(self.lbl_scope_status)

        main_layout.addLayout(scope_layout)

        # =================================================
        # 4. Plot widget
        # =================================================

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("w")

        plot_item = self.plot_widget.getPlotItem()
        self.view_box = plot_item.getViewBox()

        self.plot_widget.showGrid(
            x=True,
            y=True,
            alpha=0.3,
        )

        self.plot_widget.setLabel("bottom", "Sample Index")
        self.plot_widget.setLabel("left", "Joint Position")

        # 允许鼠标交互
        self.plot_widget.setMouseEnabled(x=True, y=True)

        # 左键拖动矩形框选，松开后放大选区
        self.view_box.setMouseMode(pg.ViewBox.RectMode)

        plot_item.setMenuEnabled(True)

        self.plot_widget.getAxis("bottom").setStyle(tickTextOffset=8)

        # 曲线及可点击的圆形数据点
        self.curve = self.plot_widget.plot(
            [],
            [],
            pen=pg.mkPen(color="b", width=1.5),
            name="Joint Position",
            symbol="o",
            symbolSize=6,
            symbolBrush="b",
            symbolPen=pg.mkPen(color="b", width=1),
        )

        self.curve.setClipToView(True)
        self.curve.setDownsampling(
            auto=True,
            method="peak",
        )

        self.curve.sigPointsClicked.connect(self.on_points_clicked)

        main_layout.addWidget(self.plot_widget)

        # =================================================
        # 5. Heartbeat timer
        # =================================================

        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.setInterval(HEARTBEAT_INTERVAL_MS)
        self.heartbeat_timer.timeout.connect(self.write_heartbeat)

        # =================================================
        # 6. UI timer
        # =================================================

        self.ui_timer = QTimer(self)
        self.ui_timer.setTimerType(Qt.PreciseTimer)
        self.ui_timer.setInterval(UI_INTERVAL_MS)
        self.ui_timer.timeout.connect(self.update_ui_and_plot)
        self.ui_timer.start()

    # =====================================================
    # ADS connection indicator
    # =====================================================

    def set_indicator_status(self, is_connected):

        if is_connected:
            self.indicator_light.setStyleSheet(
                "background-color: #2ecc71;"
                "border-radius: 12px;"
                "border: 1px solid #27ae60;"
            )
            self.lbl_status.setText("Connected")

        else:
            self.indicator_light.setStyleSheet(
                "background-color: #bdc3c7;"
                "border-radius: 12px;"
                "border: 1px solid #7f8c8d;"
            )
            self.lbl_status.setText("Disconnected")

    # =====================================================
    # Connect to TwinCAT
    # =====================================================

    def on_connect_clicked(self):

        if self._connecting:
            return

        if robotInterface.m_robotConnected:
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
        self.lbl_status.setText("Connecting...")

        try:
            success = robotInterface.connect(ams_id)

        except Exception as exc:
            success = False
            print(f"[UI] Connection exception: {exc}")

        finally:
            self._connecting = False
            self.btn_connect.setEnabled(True)

        if success:
            self.set_indicator_status(bool(robotInterface.m_robotConnected))

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
    # Data queue
    # =====================================================

    def get_data_queue(self):

        data_queue = getattr(
            robotInterface,
            "m_robotStatusDataQueue",
            None,
        )

        if data_queue is None:
            data_queue = getattr(
                robotInterface,
                "m_robotStatusDataList",
                None,
            )

        return data_queue

    def clear_data_queue(self):
        """
        丢弃本次采集开始前积压的状态数据。
        """

        data_queue = self.get_data_queue()

        if data_queue is None:
            return

        while True:
            try:
                data_queue.get_nowait()

            except queue.Empty:
                break

            except Exception as exc:
                print(f"[UI] Queue clear failed: {exc}")
                break

    # =====================================================
    # Scope Start / Stop
    # =====================================================

    def start_scope(self):
        """
        清空旧数据并启动新的 Scope 采集。
        X 轴固定显示 10 秒对应的采样点范围。
        """

        self.clear_data_queue()

        self.time_data.clear()
        self.value_data.clear()
        self.counter_x = 0

        self.curve.setData([], [])

        self._manual_view = False

        # 关闭双轴自动缩放
        self.view_box.enableAutoRange(
            axis=pg.ViewBox.XYAxes,
            enable=False,
        )

        # 初始 X 轴固定为 0～10 秒对应的点数
        self.view_box.setXRange(
            0,
            SCOPE_WINDOW_POINTS,
            padding=0,
        )

        # 只允许 Y 轴自动适应数据
        self.view_box.enableAutoRange(
            axis=pg.ViewBox.YAxis,
            enable=True,
        )

        self.scope_running = True

        self.lbl_scope_status.setText("Scope running")
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

    def stop_scope(self):
        """
        停止向曲线添加数据，保留现有曲线。
        """

        self.scope_running = False

        self.lbl_scope_status.setText("Scope stopped")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

    # =====================================================
    # Click data point
    # =====================================================

    def on_points_clicked(self, plot_item, points):
        """
        点击圆形数据点后显示 X/Y 坐标。
        """

        if not points:
            return

        point = points[0]

        x_value = point.pos().x()
        y_value = point.pos().y()

        QMessageBox.information(
            self,
            "Point Data",
            f"X: {x_value:.3f}\n" f"Y: {y_value:.6f}",
        )

    # =====================================================
    # X-axis zoom
    # =====================================================

    def zoom_x(self, factor):
        """
        factor < 1：横向放大
        factor > 1：横向缩小
        保持纵轴范围不变。
        """

        x_range, _ = self.view_box.viewRange()

        x_min, x_max = x_range
        center = (x_min + x_max) / 2.0
        half_width = (x_max - x_min) * factor / 2.0

        if half_width <= 0:
            return

        self._manual_view = True

        self.view_box.enableAutoRange(
            axis=pg.ViewBox.XYAxes,
            enable=False,
        )

        self.view_box.setXRange(
            center - half_width,
            center + half_width,
            padding=0,
        )

    # =====================================================
    # Auto Range
    # =====================================================

    def auto_range(self):
        """
        恢复实时窗口跟随模式：
        X 轴显示最近 10 秒，Y 轴自动适应。
        """

        self._manual_view = False

        self.view_box.enableAutoRange(
            axis=pg.ViewBox.XYAxes,
            enable=False,
        )

        self.view_box.enableAutoRange(
            axis=pg.ViewBox.YAxis,
            enable=True,
        )

        if self.counter_x <= SCOPE_WINDOW_POINTS:
            self.view_box.setXRange(
                0,
                SCOPE_WINDOW_POINTS,
                padding=0,
            )
        else:
            self.view_box.setXRange(
                self.counter_x - SCOPE_WINDOW_POINTS,
                self.counter_x,
                padding=0,
            )

    # =====================================================
    # Heartbeat
    # =====================================================

    def write_heartbeat(self):

        if self._closing:
            return

        if not robotInterface.m_robotConnected:
            return

        try:
            robotInterface.writeHeartBeat()

        except Exception as exc:
            print(f"[UI] Heartbeat write failed: {exc}")

    # =====================================================
    # UI and plot update
    # =====================================================

    def update_ui_and_plot(self):

        if self._closing:
            return

        is_connected = bool(robotInterface.m_robotConnected)

        self.set_indicator_status(is_connected)

        if not is_connected:
            return

        data_queue = self.get_data_queue()

        if data_queue is None:
            return

        updated = False

        for _ in range(MAX_SAMPLES_PER_UPDATE):

            try:
                self.slaveRobotData = data_queue.get_nowait()

            except queue.Empty:
                break

            except Exception as exc:
                print(f"[UI] Queue read failed: {exc}")
                break

            # Stop 时仍然消费队列，避免旧数据积压
            if not self.scope_running:
                continue

            try:
                value = float(self.slaveRobotData.m_armJntPos[ROBOT_INDEX][JOINT_INDEX])

                self.counter_x += 1

                self.time_data.append(self.counter_x)
                self.value_data.append(value)

                updated = True

            except (
                AttributeError,
                IndexError,
                TypeError,
                ValueError,
            ) as exc:
                print(f"[UI] Invalid robot status data: {exc}")

        if not self.scope_running or not updated:
            return

        # 每次 UI 更新只刷新一次曲线
        self.curve.setData(
            list(self.time_data),
            list(self.value_data),
        )

        # 纵轴自动适应数据，不自动缩放横轴
        self.view_box.enableAutoRange(
            axis=pg.ViewBox.YAxis,
            enable=True,
        )

        # 用户手动缩放或平移时，不覆盖用户视图
        if not self._manual_view:

            if self.counter_x <= SCOPE_WINDOW_POINTS:
                # 前 10 秒：固定窗口，避免横轴持续变化
                self.view_box.setXRange(
                    0,
                    SCOPE_WINDOW_POINTS,
                    padding=0,
                )

            else:
                # 超过 10 秒：窗口随最新数据向右滚动
                self.view_box.setXRange(
                    self.counter_x - SCOPE_WINDOW_POINTS,
                    self.counter_x,
                    padding=0,
                )

    # =====================================================
    # Close window
    # =====================================================

    def closeEvent(self, event):

        self._closing = True

        self.ui_timer.stop()
        self.heartbeat_timer.stop()

        disconnect_func = getattr(
            robotInterface,
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

                    if robotInterface.m_robotConnected:
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

                if robotInterface.m_robotConnected:
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
    window.show()

    sys.exit(app.exec_())
