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


class RobotMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Robot TwinCAT Control & Monitor Interface")
        self.resize(1000, 700)

        self._closing = False
        self._connecting = False
        self.scope_running = False
        self.last_time = time.perf_counter()

        # =================================================
        # 1. Main layout
        # =================================================

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # =================================================
        # 2. TwinCAT connection bar
        # =================================================

        top_layout = QHBoxLayout()

        lbl_ams = QLabel("AMS Net ID:")
        self.input_ams = QLineEdit(DEFAULT_AMS_NET_ID)
        self.input_ams.setFixedWidth(180)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.on_connect_clicked)

        self.lbl_status = QLabel("Disconnected")
        self.indicator_light = QLabel()
        self.indicator_light.setFixedSize(25, 25)
        self.set_indicator_status(False)

        top_layout.addWidget(lbl_ams)
        top_layout.addWidget(self.input_ams)
        top_layout.addWidget(self.btn_connect)
        top_layout.addStretch()
        top_layout.addWidget(QLabel("ADS Status:"))
        top_layout.addWidget(self.lbl_status)
        top_layout.addWidget(self.indicator_light)

        main_layout.addLayout(top_layout)

        # =================================================
        # 3. Oscilloscope title and Start / Stop controls
        # =================================================

        lbl_oscilloscope = QLabel(
            f"<b>Real-time Oscilloscope "
            f"(Robot {ROBOT_INDEX}, Joint {JOINT_INDEX})</b>"
        )
        main_layout.addWidget(lbl_oscilloscope)

        scope_control_layout = QHBoxLayout()

        self.btn_start = QPushButton("Start")
        self.btn_stop = QPushButton("Stop")
        self.btn_start.clicked.connect(self.start_scope)
        self.btn_stop.clicked.connect(self.stop_scope)
        self.btn_stop.setEnabled(False)

        self.lbl_scope_status = QLabel("Scope stopped")

        scope_control_layout.addWidget(self.btn_start)
        scope_control_layout.addWidget(self.btn_stop)
        scope_control_layout.addStretch()
        scope_control_layout.addWidget(self.lbl_scope_status)

        main_layout.addLayout(scope_control_layout)

        # =================================================
        # 4. Plot widget
        # =================================================

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("w")
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel("bottom", "Sample Index")
        self.plot_widget.setLabel("left", "Joint Position")
        self.plot_widget.setMouseEnabled(x=True, y=True)
        self.plot_widget.getPlotItem().setMenuEnabled(True)

        self.plot_widget.enableAutoRange(axis="y", enable=True)

        # Circle symbols make individual samples clickable.
        self.curve = self.plot_widget.plot(
            pen=pg.mkPen(color="b", width=2),
            name="Joint Position",
            symbol="o",
            symbolSize=5,
            symbolBrush="b",
            symbolPen=pg.mkPen(color="b", width=1),
        )
        self.curve.setClipToView(True)
        self.curve.setDownsampling(auto=True, method="peak")
        self.curve.sigPointsClicked.connect(self.on_points_clicked)

        self.time_data = deque(maxlen=MAX_PLOT_POINTS)
        self.value_data = deque(maxlen=MAX_PLOT_POINTS)
        self.counter_x = 0

        main_layout.addWidget(self.plot_widget)

        # =================================================
        # 5. Heartbeat timer
        # =================================================

        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.setInterval(HEARTBEAT_INTERVAL_MS)
        self.heartbeat_timer.timeout.connect(self.write_heartbeat)

        # =================================================
        # 6. UI / data queue timer
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
        """Update the ADS connection label and indicator light."""
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
        """Establish the ADS connection when Connect is clicked."""
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
                "Failed to connect to TwinCAT. "
                "Check AMS Net ID and PLC status.",
            )

    # =====================================================
    # Oscilloscope Start / Stop
    # =====================================================

    def start_scope(self):
        """Start plotting new data without clearing the existing curve."""
        self.scope_running = True
        self.lbl_scope_status.setText("Scope running")
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

    def stop_scope(self):
        """Stop updating the plot and keep the current curve visible."""
        self.scope_running = False
        self.lbl_scope_status.setText("Scope stopped")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

    def on_points_clicked(self, plot_item, points):
        """Show the sample index and value when a curve point is clicked."""
        if not points:
            return

        point = points[0]
        x_value = point.pos().x()
        y_value = point.pos().y()

        QMessageBox.information(
            self,
            "Sample Information",
            f"Sample Index: {x_value:.0f}\n"
            f"Joint Position: {y_value:.6f}",
        )

    # =====================================================
    # Heartbeat
    # =====================================================

    def write_heartbeat(self):
        """Write heartbeat while connected, regardless of scope state."""
        if self._closing or not robotInterface.m_robotConnected:
            return

        try:
            robotInterface.writeHeartBeat()
        except Exception as exc:
            print(f"[UI] Heartbeat write failed: {exc}")

    # =====================================================
    # UI and plot update
    # =====================================================

    def update_ui_and_plot(self):
        """Consume queued status data; update the curve only when Start is active."""
        if self._closing:
            return

        is_connected = bool(robotInterface.m_robotConnected)
        self.set_indicator_status(is_connected)

        if not is_connected:
            return

        data_queue = getattr(
            robotInterface,
            "m_robotStatusDataQueue",
            None,
        )
        if data_queue is None:
            # Compatibility with an older variable name. It must also be queue.Queue.
            data_queue = getattr(
                robotInterface,
                "m_robotStatusDataList",
                None,
            )

        if data_queue is None:
            return

        updated = False

        for _ in range(MAX_SAMPLES_PER_UPDATE):
            try:
                data_item = data_queue.get_nowait()
            except queue.Empty:
                break
            except Exception as exc:
                print(f"[UI] Queue read failed: {exc}")
                break

            # While stopped, consume the queue but leave the displayed curve unchanged.
            if not self.scope_running:
                continue

            try:
                value = float(
                    data_item.m_armJntPos[ROBOT_INDEX][JOINT_INDEX]
                )
                self.counter_x += 1
                self.time_data.append(self.counter_x)
                self.value_data.append(value)
                updated = True
            except (AttributeError, IndexError, TypeError, ValueError) as exc:
                print(f"[UI] Invalid robot status data: {exc}")

        if self.scope_running and updated:
            self.curve.setData(
                list(self.time_data),
                list(self.value_data),
            )

        # Optional timing diagnostic; uncomment if needed.
        # current_time = time.perf_counter()
        # interval_ms = (current_time - self.last_time) * 1000.0
        # print(f"[UI Interval] {interval_ms:.2f} ms")
        # self.last_time = current_time

    # =====================================================
    # Close window and release ADS resources
    # =====================================================

    def closeEvent(self, event):
        """Stop timers and attempt to release ADS resources."""
        self._closing = True
        self.ui_timer.stop()
        self.heartbeat_timer.stop()

        disconnect_func = getattr(robotInterface, "disconnect", None)
        if callable(disconnect_func):
            try:
                result = disconnect_func()
                if result is False:
                    self._closing = False
                    QMessageBox.warning(
                        self,
                        "Warning",
                        "ADS resources have not been released successfully. "
                        "Please try closing again.",
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
