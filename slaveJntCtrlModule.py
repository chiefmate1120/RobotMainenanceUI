import queue
from collections import deque

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QMessageBox,
    QTabWidget,
    QTableWidget,
    QHeaderView,
    QTableWidgetItem,
    QComboBox,
    QStyledItemDelegate,
    QStyle,
    QStyleOptionViewItem,
    QStyleOptionButton,
)
from PyQt5.QtCore import QTimer, Qt, QRect, QRectF, QEvent, pyqtSignal
from PyQt5.QtGui import QColor, QIntValidator, QPainter

import pyqtgraph as pg

import slaveRobotInterface

# =========================================================
# Configuration
# =========================================================

DATA_INTERVAL_MS = 20
MAX_PLOT_POINTS = 10000
MAX_SAMPLES_PER_UPDATE = 500
SCOPE_WINDOW_POINTS = MAX_PLOT_POINTS

ARM_OP_MODE_POSITION = 0
ARM_OP_MODE_CURRENT = 1

CART_COLUMN_FIELDS = [
    "m_cartJntPos",
    "m_cartCmdWord",
    "m_cartStatusWord",
    "m_cartCmdMotorEncoder",
    "m_cartCurMotorEncoder",
    "m_cartcurLinkEncoder",
]

ARM_COLUMN_FIELDS = [
    "m_armJntPos",
    "m_armCmdWord",
    "m_armStatusWord",
    "m_armOPMode",
    "m_armCmdMotorEncoder",
    "m_armCurMotorEncoder",
    "m_armCurLinkEncoder",
    "m_armCmdMotorCurrent",
    "m_armCurMotorCurrent",
]

CURVE_COLORS = [
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#17becf",
    "#bcbd22",
    "#7f7f7f",
]


def apply_tab_bar_style(tab_widget):
    """分页标签加高，并区分上凸/下凹状态。"""
    tab_bar = tab_widget.tabBar()
    base_tab_height = tab_bar.sizeHint().height()
    target_tab_height = max(base_tab_height * 2, 48)

    tab_bar.setStyleSheet(f"""
        QTabBar::tab {{
            height: {target_tab_height}px;
            padding: 0 16px;
            margin-right: 3px;
            background-color: #dfe6e9;
            color: #2c3e50;
            font-weight: bold;
            border-top: 2px solid #ffffff;
            border-left: 2px solid #ffffff;
            border-right: 2px solid #95a5a6;
            border-bottom: 2px solid #95a5a6;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
        }}
        QTabBar::tab:hover {{
            background-color: #ecf0f1;
        }}
        QTabBar::tab:selected {{
            background-color: #ffffff;
            color: #1a5276;
            border-top: 2px solid #566573;
            border-left: 2px solid #566573;
            border-right: 2px solid #bdc3c7;
            border-bottom: 1px solid #ffffff;
            margin-top: 4px;
        }}
        """)


class ToggleSwitch(QWidget):
    """左右滑动开关，可附带文字标签。"""

    toggled = pyqtSignal(bool)

    def __init__(self, text="", parent=None, track_width=44, track_height=22):
        super().__init__(parent)

        self._text = text
        self._checked = False
        self._track_width = track_width
        self._track_height = track_height
        self._spacing = 8 if text else 0

        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.StrongFocus)

        text_width = self.fontMetrics().horizontalAdvance(text) if text else 0
        self.setFixedSize(
            track_width + self._spacing + text_width + 2,
            max(track_height, self.fontMetrics().height()),
        )

    def isChecked(self):
        return self._checked

    def setChecked(self, checked):
        checked = bool(checked)
        if checked == self._checked:
            return
        self._checked = checked
        self.update()
        self.toggled.emit(checked)

    def toggle(self):
        if self.isEnabled():
            self.setChecked(not self._checked)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.toggle()
            event.accept()
            return
        super().mousePressEvent(event)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Space, Qt.Key_Return, Qt.Key_Enter):
            self.toggle()
            event.accept()
            return
        super().keyPressEvent(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        track_top = (self.height() - self._track_height) / 2.0
        track_rect = QRectF(0.0, track_top, self._track_width, self._track_height)

        if not self.isEnabled():
            track_color = QColor("#d5dbdb")
        elif self._checked:
            track_color = QColor("#2ecc71")
        else:
            track_color = QColor("#bdc3c7")

        painter.setPen(Qt.NoPen)
        painter.setBrush(track_color)
        painter.drawRoundedRect(
            track_rect,
            self._track_height / 2.0,
            self._track_height / 2.0,
        )

        margin = 2.0
        knob_diameter = self._track_height - 2.0 * margin
        knob_x = self._track_width - margin - knob_diameter if self._checked else margin
        knob_rect = QRectF(
            knob_x,
            (self.height() - knob_diameter) / 2.0,
            knob_diameter,
            knob_diameter,
        )
        painter.setBrush(QColor("#ffffff"))
        painter.drawEllipse(knob_rect)

        if self._text:
            text_color = QColor("#2c3e50" if self.isEnabled() else "#95a5a6")
            painter.setPen(text_color)
            text_rect = QRectF(
                self._track_width + self._spacing,
                0.0,
                self.width() - self._track_width - self._spacing,
                self.height(),
            )
            painter.drawText(
                text_rect,
                Qt.AlignVCenter | Qt.AlignLeft,
                self._text,
            )


class RightCheckDelegate(QStyledItemDelegate):
    """数据格文字正常绘制，勾选框绘制在单元格最右侧。"""

    RIGHT_MARGIN = 6
    MIN_SIZE = 12
    MAX_SIZE = 18

    def _check_rect(self, rect):
        size = max(self.MIN_SIZE, min(self.MAX_SIZE, rect.height() - 8))
        left = rect.right() - self.RIGHT_MARGIN - size + 1
        top = rect.top() + (rect.height() - size) // 2
        return QRect(left, top, size, size)

    def paint(self, painter, option, index):
        check_state = index.data(Qt.CheckStateRole)

        if check_state is None:
            super().paint(painter, option, index)
            return

        opt = QStyleOptionViewItem(option)
        self.initStyleOption(opt, index)

        opt.features &= ~QStyleOptionViewItem.HasCheckIndicator

        widget = opt.widget
        style = widget.style() if widget is not None else QApplication.style()

        style.drawControl(QStyle.CE_ItemViewItem, opt, painter, widget)

        button = QStyleOptionButton()
        flags = index.flags()

        if (flags & Qt.ItemIsEnabled) and (flags & Qt.ItemIsUserCheckable):
            button.state |= QStyle.State_Enabled
        else:
            button.state &= ~QStyle.State_Enabled

        if check_state == Qt.Checked:
            button.state |= QStyle.State_On
        else:
            button.state |= QStyle.State_Off

        button.rect = self._check_rect(option.rect)
        style.drawPrimitive(
            QStyle.PE_IndicatorCheckBox,
            button,
            painter,
            widget,
        )

    def editorEvent(self, event, model, option, index):
        if event.type() in (
            QEvent.MouseButtonPress,
            QEvent.MouseButtonRelease,
            QEvent.MouseButtonDblClick,
        ):
            if (
                event.type() == QEvent.MouseButtonRelease
                and event.button() == Qt.LeftButton
                and (index.flags() & Qt.ItemIsEnabled)
                and (index.flags() & Qt.ItemIsUserCheckable)
                and self._check_rect(option.rect).contains(event.pos())
            ):
                current = index.data(Qt.CheckStateRole)
                model.setData(
                    index,
                    Qt.Unchecked if current == Qt.Checked else Qt.Checked,
                    Qt.CheckStateRole,
                )
                return True

            return False

        return super().editorEvent(event, model, option, index)


class NoScaleLabelAxisItem(pg.AxisItem):
    """保留自动 SI 前缀用于刻度缩放，但轴标题不追加 (x...) 比例。"""

    def labelString(self):
        return self.labelText


class SlaveJntCtrlPage(QWidget):
    """关节状态和控制分页：患者台车 / 1~4号臂。"""

    # 子分页切换时请求主窗口关闭“上位机控制”。
    control_disable_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.slaveRobotData = slaveRobotInterface.ST_RMRobotStatusData()
        self.command_data = slaveRobotInterface.ST_RMJntCtrlData()

        self._control_enabled = False
        self.scope_running = False

        self.time_data = deque(maxlen=MAX_PLOT_POINTS)
        self.value_data = []
        self.channel_descriptors = []
        self.channel_names = []
        self.curves = []

        self._status_tables = []
        self._table_meta = {}
        self._right_check_delegate = RightCheckDelegate(self)

        self.cart_enable_switches = []
        self.cart_cmd_pos_edits = []

        self.arm_enable_switches = [[] for _ in range(4)]
        self.arm_mode_combos = [[] for _ in range(4)]
        self.arm_cmd_pos_edits = [[] for _ in range(4)]
        self.arm_cmd_curr_edits = [[] for _ in range(4)]

        self.counter_x = 0
        self._manual_view = False

        layout = QVBoxLayout(self)

        # -------------------------------------------------
        # 子分页
        # -------------------------------------------------
        self.status_tabs = QTabWidget(self)
        self.status_tabs.setDocumentMode(True)
        self.status_tabs.setMovable(False)

        # “下发指令”按键放在子分页右上角。
        self.btn_send_command = QPushButton("下发指令")
        self.btn_send_command.setCheckable(False)
        self.btn_send_command.setCursor(Qt.PointingHandCursor)
        self.btn_send_command.setMinimumWidth(100)
        self.btn_send_command.setStyleSheet(
            "QPushButton {"
            " padding: 5px 16px;"
            " font-weight: bold;"
            " border: 1px solid #2980b9;"
            " border-radius: 4px;"
            " background-color: #3498db;"
            " color: white;"
            "}"
            "QPushButton:hover { background-color: #2980b9; }"
            "QPushButton:pressed { background-color: #1f618d; }"
        )
        self.btn_send_command.pressed.connect(self.on_send_command_pressed)

        corner_widget = QWidget()
        corner_layout = QHBoxLayout(corner_widget)
        corner_layout.setContentsMargins(0, 0, 8, 0)
        corner_layout.addWidget(self.btn_send_command)
        self.status_tabs.setCornerWidget(corner_widget, Qt.TopRightCorner)

        cart_columns = [
            "关节位置",
            "控制字",
            "状态字",
            "指令电机编码器值",
            "实际电机编码器值",
            "实际关节编码器值",
        ]
        self.cart_table = self._create_status_table(
            ["支撑脚1", "支撑脚2", "立柱", "横梁旋转", "横梁伸缩", "悬吊旋转"],
            cart_columns,
            kind="cart",
            tab_name="患者台车",
        )
        self.status_tabs.addTab(self.cart_table, "患者台车")

        arm_columns = [
            "关节位置",
            "控制字",
            "状态字",
            "控制模式",
            "指令电机编码器值",
            "实际电机编码器值",
            "实际关节编码器值",
            "指令电流",
            "实际电流",
        ]
        self.arm_tables = []
        for arm_index, tab_name in enumerate(["1号臂", "2号臂", "3号臂", "4号臂"]):
            table = self._create_status_table(
                [f"关节{i}" for i in range(1, 14)],
                arm_columns,
                kind="arm",
                tab_name=tab_name,
                arm_index=arm_index,
            )
            self.arm_tables.append(table)
            self.status_tabs.addTab(table, tab_name)

        self.status_tabs.currentChanged.connect(self.on_status_tab_changed)

        apply_tab_bar_style(self.status_tabs)

        base_button_height = self.btn_send_command.sizeHint().height()
        self.btn_send_command.setMinimumHeight(max(base_button_height * 2, 40))

        layout.addWidget(self.status_tabs)

        # -------------------------------------------------
        # 示波器
        # -------------------------------------------------
        layout.addWidget(QLabel("<b>实时数据显示</b>"))

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

        scope_layout.addWidget(self.btn_start)
        scope_layout.addWidget(self.btn_stop)
        scope_layout.addSpacing(15)
        scope_layout.addWidget(self.btn_x_zoom_in)
        scope_layout.addWidget(self.btn_x_zoom_out)
        scope_layout.addWidget(self.btn_auto_range)
        scope_layout.addStretch()

        layout.addLayout(scope_layout)

        self.plot_widget = pg.PlotWidget(
            axisItems={"left": NoScaleLabelAxisItem(orientation="left")}
        )
        self.plot_widget.setBackground("w")

        plot_item = self.plot_widget.getPlotItem()
        self.view_box = plot_item.getViewBox()

        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("bottom", "Sample Index")
        self.plot_widget.setLabel("left", "Value")

        self.y_scale_label = QLabel(self.plot_widget)
        self.y_scale_label.setStyleSheet(
            "background-color: rgba(255, 255, 255, 180);"
            " color: #2c3e50;"
            " padding: 1px 4px;"
        )
        self.y_scale_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.y_scale_label.move(8, 6)
        self.y_scale_label.hide()

        self.view_box.sigRangeChanged.connect(self.update_y_scale_label)

        self.plot_widget.setMouseEnabled(x=True, y=True)
        self.view_box.setMouseMode(pg.ViewBox.RectMode)
        plot_item.setMenuEnabled(True)
        self.plot_widget.getAxis("bottom").setStyle(tickTextOffset=8)

        self.legend = self.plot_widget.addLegend(offset=(10, 10))

        layout.addWidget(self.plot_widget)

        # 初始状态：上位机控制关闭。
        self.set_control_enabled(False)

        # 独立数据刷新定时器。
        self.data_timer = QTimer(self)
        self.data_timer.setTimerType(Qt.PreciseTimer)
        self.data_timer.setInterval(DATA_INTERVAL_MS)
        self.data_timer.timeout.connect(self.process_data)
        self.data_timer.start()

    # =====================================================
    # 表格创建
    # =====================================================

    def _create_status_table(
        self,
        row_names,
        column_names,
        kind,
        tab_name,
        arm_index=None,
    ):
        """创建状态表格；首列为名称，末尾为上位机指令控制列。"""
        if kind == "cart":
            command_columns = ["关节使能", "指令位置"]
        else:
            command_columns = ["关节使能", "控制模式", "指令位置", "指令电流"]

        total_columns = 1 + len(column_names) + len(command_columns)
        table = QTableWidget(len(row_names), total_columns, self)
        table.setHorizontalHeaderLabels(["名称"] + list(column_names) + command_columns)
        table.setVerticalHeaderLabels([])
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setAlternatingRowColors(True)
        table.verticalHeader().setVisible(False)

        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        for c in range(1, total_columns):
            header.setSectionResizeMode(c, QHeaderView.Stretch)
        header.setStretchLastSection(False)
        table.setMinimumHeight(180)

        command_start = 1 + len(column_names)

        for c in range(1, len(column_names) + 1):
            table.setItemDelegateForColumn(c, self._right_check_delegate)

        for r, name in enumerate(row_names):
            item = QTableWidgetItem(name)
            table.setItem(r, 0, item)

            for c in range(len(column_names)):
                cell = QTableWidgetItem("--")
                cell.setFlags(cell.flags() | Qt.ItemIsUserCheckable)
                cell.setCheckState(Qt.Unchecked)
                table.setItem(r, c + 1, cell)

            enable_switch = ToggleSwitch()
            enable_switch.setEnabled(False)
            table.setCellWidget(
                r,
                command_start,
                self._wrap_cell_widget(enable_switch, center=True),
            )

            if kind == "cart":
                enable_switch.toggled.connect(
                    lambda checked, row=r: self._on_cart_enable_toggled(row, checked)
                )
                self.cart_enable_switches.append(enable_switch)

                pos_edit = QLineEdit("0")
                pos_edit.setEnabled(False)
                pos_edit.textChanged.connect(
                    lambda text, row=r: self._on_cart_cmd_pos_changed(row, text)
                )
                table.setCellWidget(
                    r,
                    command_start + 1,
                    self._wrap_cell_widget(pos_edit),
                )
                self.cart_cmd_pos_edits.append(pos_edit)
            else:
                enable_switch.toggled.connect(
                    lambda checked, arm=arm_index, joint=r: (
                        self._on_arm_enable_toggled(arm, joint, checked)
                    )
                )
                self.arm_enable_switches[arm_index].append(enable_switch)

                mode_combo = QComboBox()
                mode_combo.addItem("位置模式", ARM_OP_MODE_POSITION)
                mode_combo.addItem("电流模式", ARM_OP_MODE_CURRENT)
                mode_combo.setEnabled(False)
                mode_combo.currentIndexChanged.connect(
                    lambda index, arm=arm_index, joint=r: (
                        self._on_arm_mode_changed(arm, joint)
                    )
                )
                table.setCellWidget(
                    r,
                    command_start + 1,
                    self._wrap_cell_widget(mode_combo),
                )
                self.arm_mode_combos[arm_index].append(mode_combo)

                pos_edit = QLineEdit("0")
                pos_edit.setEnabled(False)
                pos_edit.textChanged.connect(
                    lambda text, arm=arm_index, joint=r: (
                        self._on_arm_cmd_pos_changed(arm, joint, text)
                    )
                )
                table.setCellWidget(
                    r,
                    command_start + 2,
                    self._wrap_cell_widget(pos_edit),
                )
                self.arm_cmd_pos_edits[arm_index].append(pos_edit)

                curr_edit = QLineEdit("0")
                curr_edit.setEnabled(False)
                curr_edit.setValidator(
                    QIntValidator(-2147483648, 2147483647, curr_edit)
                )
                curr_edit.textChanged.connect(
                    lambda text, arm=arm_index, joint=r: (
                        self._on_arm_cmd_current_changed(arm, joint, text)
                    )
                )
                table.setCellWidget(
                    r,
                    command_start + 3,
                    self._wrap_cell_widget(curr_edit),
                )
                self.arm_cmd_curr_edits[arm_index].append(curr_edit)

        self._status_tables.append(table)
        self._table_meta[table] = {
            "kind": kind,
            "arm_index": arm_index,
            "tab_name": tab_name,
            "row_names": list(row_names),
            "column_names": list(column_names),
        }
        return table

    @staticmethod
    def _wrap_cell_widget(child, center=False):
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(4, 2, 4, 2)
        if center:
            layout.addStretch()
            layout.addWidget(child)
            layout.addStretch()
        else:
            layout.addWidget(child)
        return container

    # =====================================================
    # 数据更新
    # =====================================================

    def process_data(self):
        """从共享队列读取状态数据，更新表格和示波器。"""
        if not slaveRobotInterface.m_robotConnected:
            return

        data_queue = getattr(
            slaveRobotInterface,
            "m_robotStatusDataQueue",
            None,
        )
        if data_queue is None:
            return

        updated = False

        for _ in range(MAX_SAMPLES_PER_UPDATE):
            try:
                data = data_queue.get_nowait()
            except queue.Empty:
                break
            except Exception as exc:
                print(f"[SlaveJntCtrl] Queue read failed: {exc}")
                break

            self.slaveRobotData = data

            if not self.scope_running:
                continue

            try:
                sample = [
                    self._read_channel_value(channel, data)
                    for channel in self.channel_descriptors
                ]

                if len(sample) != len(self.value_data):
                    continue

                self.counter_x += 1
                self.time_data.append(self.counter_x)

                for column, value in zip(self.value_data, sample):
                    column.append(value)

                updated = True
            except (
                AttributeError,
                IndexError,
                TypeError,
                ValueError,
            ) as exc:
                print(f"[SlaveJntCtrl] Invalid status data: {exc}")

        if self.slaveRobotData is not None:
            self.update_status_tables(self.slaveRobotData)

        if not self.scope_running or not updated:
            return

        x_values = list(self.time_data)

        for curve, column in zip(self.curves, self.value_data):
            curve.setData(x_values, list(column))

        self.view_box.enableAutoRange(
            axis=pg.ViewBox.YAxis,
            enable=True,
        )

        if not self._manual_view:
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

    def update_status_tables(self, data):
        """更新患者台车和 1~4 号臂表格。"""
        for row in range(6):
            values = [
                data.m_cartJntPos[row],
                data.m_cartCmdWord[row],
                data.m_cartStatusWord[row],
                data.m_cartCmdMotorEncoder[row],
                data.m_cartCurMotorEncoder[row],
                data.m_cartcurLinkEncoder[row],
            ]
            self._set_table_row(self.cart_table, row, values)

        for arm_index, table in enumerate(self.arm_tables):
            for joint_index in range(13):
                values = [
                    data.m_armJntPos[arm_index][joint_index],
                    data.m_armCmdWord[arm_index][joint_index],
                    data.m_armStatusWord[arm_index][joint_index],
                    data.m_armOPMode[arm_index][joint_index],
                    data.m_armCmdMotorEncoder[arm_index][joint_index],
                    data.m_armCurMotorEncoder[arm_index][joint_index],
                    data.m_armCurLinkEncoder[arm_index][joint_index],
                    data.m_armCmdMotorCurrent[arm_index][joint_index],
                    data.m_armCurMotorCurrent[arm_index][joint_index],
                ]
                self._set_table_row(table, joint_index, values)

        # 上位机控制关闭时，指令位置跟随状态表格，控制模式固定为位置模式。
        if not self._control_enabled:
            self._refresh_command_positions(data)
            self._force_position_command_modes()

    @staticmethod
    def _format_cell(value):
        if value is None:
            return "--"
        try:
            number = float(value)
            if number.is_integer():
                return str(int(number))
            return f"{number:.5f}".rstrip("0").rstrip(".")
        except (TypeError, ValueError):
            return str(value)

    def _set_table_row(self, table, row, values):
        for col, value in enumerate(values, start=1):
            item = table.item(row, col)
            if item is None:
                item = QTableWidgetItem()
                item.setCheckState(Qt.Unchecked)
                if self.scope_running:
                    item.setFlags(item.flags() & ~Qt.ItemIsUserCheckable)
                else:
                    item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                table.setItem(row, col, item)
            item.setText(self._format_cell(value))

    # =====================================================
    # 示波器数据选择
    # =====================================================

    def _set_checkboxes_enabled(self, enabled):
        for table in self._status_tables:
            for row in range(table.rowCount()):
                for col in range(1, table.columnCount()):
                    item = table.item(row, col)
                    if item is None:
                        continue
                    flags = item.flags()
                    if enabled:
                        flags |= Qt.ItemIsUserCheckable
                    else:
                        flags &= ~Qt.ItemIsUserCheckable
                    item.setFlags(flags)

    def _clear_all_checkboxes(self):
        for table in self._status_tables:
            for row in range(table.rowCount()):
                for col in range(1, table.columnCount()):
                    item = table.item(row, col)
                    if item is not None:
                        item.setCheckState(Qt.Unchecked)

    def on_status_tab_changed(self, index):
        """切换子分页：清空勾选、写入模块号，并请求关闭上位机控制。"""
        self._clear_all_checkboxes()
        slaveRobotInterface.writeModuleIdx(int(index))
        self.control_disable_requested.emit()

    def current_module_index(self):
        return int(self.status_tabs.currentIndex())

    def _collect_selected_channels(self):
        table = self.status_tabs.currentWidget()
        meta = self._table_meta.get(table)
        if meta is None:
            return []

        channels = []
        for row in range(table.rowCount()):
            name_item = table.item(row, 0)
            row_name = name_item.text() if name_item else f"行{row + 1}"
            for col in range(1, table.columnCount()):
                item = table.item(row, col)
                if item is None or item.checkState() != Qt.Checked:
                    continue
                column_name = meta["column_names"][col - 1]
                channels.append(
                    {
                        "label": f"{meta['tab_name']}-{row_name}-{column_name}",
                        "kind": meta["kind"],
                        "arm_index": meta["arm_index"],
                        "row": row,
                        "data_col": col - 1,
                    }
                )
        return channels

    @staticmethod
    def _read_channel_value(channel, data):
        if channel["kind"] == "cart":
            field_name = CART_COLUMN_FIELDS[channel["data_col"]]
            value = getattr(data, field_name)[channel["row"]]
        else:
            field_name = ARM_COLUMN_FIELDS[channel["data_col"]]
            value = getattr(data, field_name)[channel["arm_index"]][channel["row"]]
        return float(getattr(value, "value", value))

    # =====================================================
    # 上位机指令控制
    # =====================================================

    def set_control_enabled(self, enabled):
        """主窗口“上位机控制”按钮调用此方法。"""
        enabled = bool(enabled)
        self._control_enabled = enabled

        for switch, edit in zip(
            self.cart_enable_switches,
            self.cart_cmd_pos_edits,
        ):
            switch.setEnabled(enabled)
            if not enabled:
                switch.setChecked(False)
            edit.setEnabled(enabled)

        for arm_index in range(4):
            for switch in self.arm_enable_switches[arm_index]:
                switch.setEnabled(enabled)
                if not enabled:
                    switch.setChecked(False)

            for combo in self.arm_mode_combos[arm_index]:
                combo.setEnabled(enabled)

            for joint_index in range(len(self.arm_cmd_pos_edits[arm_index])):
                self._update_arm_input_enabled(
                    arm_index,
                    joint_index,
                    enabled,
                )

        # 关闭上位机控制后，控制模式始终回到位置模式。
        if not enabled:
            self._force_position_command_modes()

    def _update_arm_input_enabled(
        self,
        arm_index,
        joint_index,
        control_enabled=None,
    ):
        if control_enabled is None:
            control_enabled = self._control_enabled

        pos_edit = self.arm_cmd_pos_edits[arm_index][joint_index]
        curr_edit = self.arm_cmd_curr_edits[arm_index][joint_index]

        if not control_enabled:
            pos_edit.setEnabled(False)
            curr_edit.setEnabled(False)
            return

        mode = self.arm_mode_combos[arm_index][joint_index].currentData()

        if mode == ARM_OP_MODE_CURRENT:
            pos_edit.setEnabled(False)
            curr_edit.setEnabled(True)
        else:
            pos_edit.setEnabled(True)
            curr_edit.setEnabled(False)

    def on_send_command_pressed(self):
        success = slaveRobotInterface.writeUICmds(self.command_data)
        if not success:
            QMessageBox.warning(
                self,
                "Warning",
                "指令下发失败，请检查 TwinCAT 连接状态。",
            )

    def _on_cart_enable_toggled(self, row, checked):
        self.command_data.m_cartJntEnable[row] = bool(checked)

    def _on_arm_enable_toggled(self, arm_index, joint_index, checked):
        self.command_data.m_armJntEnable[arm_index][joint_index] = bool(checked)

    def _on_arm_mode_changed(self, arm_index, joint_index):
        combo = self.arm_mode_combos[arm_index][joint_index]
        value = combo.currentData()
        if value is not None:
            self.command_data.m_armCmdOPMode[arm_index][joint_index] = int(value)
        self._update_arm_input_enabled(arm_index, joint_index)

    def _on_cart_cmd_pos_changed(self, row, text):
        value = self._parse_float(text)
        if value is not None:
            self.command_data.m_cartCmdJntPos[row] = value

    def _on_arm_cmd_pos_changed(self, arm_index, joint_index, text):
        value = self._parse_float(text)
        if value is not None:
            self.command_data.m_armCmdJntPos[arm_index][joint_index] = value

    def _on_arm_cmd_current_changed(self, arm_index, joint_index, text):
        value = self._parse_float(text)
        if value is not None:
            self.command_data.m_armCmdMotorCurrent[arm_index][joint_index] = int(
                round(value)
            )

    @staticmethod
    def _parse_float(text):
        try:
            return float(text)
        except (TypeError, ValueError):
            return None

    def _refresh_command_positions(self, data):
        """控制关闭时，指令位置跟随状态表第一列。"""
        try:
            for row, edit in enumerate(self.cart_cmd_pos_edits):
                value = data.m_cartJntPos[row]
                text = self._format_cell(value)
                if edit.text() != text:
                    edit.setText(text)
                self.command_data.m_cartCmdJntPos[row] = float(value)
        except (AttributeError, IndexError, TypeError, ValueError) as exc:
            print(f"[SlaveJntCtrl] Cart position refresh failed: {exc}")

        for arm_index, edits in enumerate(self.arm_cmd_pos_edits):
            try:
                for joint_index, edit in enumerate(edits):
                    value = data.m_armJntPos[arm_index][joint_index]
                    text = self._format_cell(value)
                    if edit.text() != text:
                        edit.setText(text)
                    self.command_data.m_armCmdJntPos[arm_index][joint_index] = float(
                        value
                    )
            except (
                AttributeError,
                IndexError,
                TypeError,
                ValueError,
            ) as exc:
                print(f"[SlaveJntCtrl] Arm {arm_index} position refresh failed: {exc}")

    def _force_position_command_modes(self):
        """上位机控制关闭时，指令控制模式固定为位置模式。"""
        for arm_index, combos in enumerate(self.arm_mode_combos):
            for joint_index, combo in enumerate(combos):
                position_index = combo.findData(ARM_OP_MODE_POSITION)
                if position_index < 0:
                    position_index = 0

                if combo.currentIndex() != position_index:
                    combo.setCurrentIndex(position_index)

                self.command_data.m_armCmdOPMode[arm_index][joint_index] = int(
                    combo.currentData()
                )

    # =====================================================
    # 示波器控制
    # =====================================================

    def _clear_curves(self):
        for curve in self.curves:
            self.plot_widget.removeItem(curve)
        self.curves = []
        if self.legend is not None:
            self.legend.clear()

    def _create_curves(self, channels):
        self._clear_curves()
        for index, channel in enumerate(channels):
            color = CURVE_COLORS[index % len(CURVE_COLORS)]
            curve = self.plot_widget.plot(
                [],
                [],
                pen=pg.mkPen(color=color, width=1.5),
                name=channel["label"],
                symbol="o",
                symbolSize=5,
                symbolBrush=color,
                symbolPen=pg.mkPen(color=color, width=1),
            )
            curve.setClipToView(True)
            curve.setDownsampling(auto=True, method="peak")
            curve.sigPointsClicked.connect(self.on_points_clicked)
            self.curves.append(curve)

    def start_scope(self):
        channels = self._collect_selected_channels()

        if not channels:
            QMessageBox.warning(
                self,
                "Warning",
                "请先勾选至少一个数据格，再启动示波器。",
            )
            return

        self.channel_descriptors = channels
        self.channel_names = [channel["label"] for channel in channels]

        while True:
            data_queue = getattr(
                slaveRobotInterface,
                "m_robotStatusDataQueue",
                None,
            )
            if data_queue is None:
                break
            try:
                data_queue.get_nowait()
            except queue.Empty:
                break
            except Exception:
                break

        self.time_data.clear()
        self.value_data = [
            deque(maxlen=MAX_PLOT_POINTS) for _ in self.channel_descriptors
        ]
        self.counter_x = 0

        self._create_curves(self.channel_descriptors)

        self._manual_view = False
        self.view_box.enableAutoRange(
            axis=pg.ViewBox.XYAxes,
            enable=False,
        )
        self.view_box.setXRange(
            0,
            SCOPE_WINDOW_POINTS,
            padding=0,
        )
        self.view_box.enableAutoRange(
            axis=pg.ViewBox.YAxis,
            enable=True,
        )

        self.scope_running = True
        self._set_checkboxes_enabled(False)
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

    def stop_scope(self):
        self.scope_running = False
        self._set_checkboxes_enabled(True)
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

    def on_points_clicked(self, plot_item, points):
        if not points:
            return
        point = points[0]
        x_value = point.pos().x()
        y_value = point.pos().y()
        curve_name = plot_item.opts.get("name") or "Curve"
        QMessageBox.information(
            self,
            "Point Data",
            f"{curve_name}\nX: {x_value:.3f}\nY: {y_value:.6f}",
        )

    def update_y_scale_label(self, *args):
        (_, _), (y_min, y_max) = self.view_box.viewRange()
        scaling_value = max(abs(y_min), abs(y_max))

        if scaling_value <= 0:
            self.y_scale_label.hide()
            return

        scale, _ = pg.functions.siScale(scaling_value)
        factor = 1.0 / scale

        if abs(factor - 1.0) < 1e-12:
            self.y_scale_label.hide()
            return

        self.y_scale_label.setText(f"x{factor:g}")
        self.y_scale_label.adjustSize()
        self.y_scale_label.show()
        self.y_scale_label.raise_()

    def zoom_x(self, factor):
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

    def auto_range(self):
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

    def stop_timer(self):
        self.data_timer.stop()
