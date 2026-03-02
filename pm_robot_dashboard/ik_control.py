
import PyQt6.QtWidgets as Q
from PyQt6.QtCore import Qt, QTimer

from pm_robot_dashboard.node import PmJogToolNode
from pm_robot_dashboard.ik_model import IkControlModel
class IkControlWidget(Q.QWidget):
    def __init__(self, parent: Q.QWidget, node: PmJogToolNode):
        super().__init__(parent)
        self.node = node
        self.model = IkControlModel(node)

        self._build_ui()
        self._connect_signals()

        # GUI timer to poll model state and update widgets
        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self._update_gui_from_model)
        self.gui_timer.start(100)  


    def _build_ui(self):
        main_layout = Q.QVBoxLayout(self)

        # Top row: active tool + target frame
        top_row = Q.QHBoxLayout()

        tool_label = Q.QLabel("Active tool")
        self.tool_combo = Q.QComboBox()
        self.tool_combo.addItems(self.model.tools)
        self.tool_combo.setCurrentText(self.model.get_active_tool())

        frame_label = Q.QLabel("Target frame")
        self.frame_combo = Q.QComboBox()

        top_row.addWidget(tool_label)
        top_row.addWidget(self.tool_combo)
        top_row.addSpacing(20)
        top_row.addWidget(frame_label)
        top_row.addWidget(self.frame_combo)

        main_layout.addLayout(top_row)

        # Current pose display
        current_group = Q.QGroupBox("Current pose [mm]")
        cg_layout = Q.QHBoxLayout()
        self.cur_x = Q.QLineEdit(); self.cur_x.setReadOnly(True); self.cur_x.setFixedWidth(100)
        self.cur_y = Q.QLineEdit(); self.cur_y.setReadOnly(True); self.cur_y.setFixedWidth(100)
        self.cur_z = Q.QLineEdit(); self.cur_z.setReadOnly(True); self.cur_z.setFixedWidth(100)
        cg_layout.addWidget(Q.QLabel("X:")); cg_layout.addWidget(self.cur_x)
        cg_layout.addWidget(Q.QLabel("Y:")); cg_layout.addWidget(self.cur_y)
        cg_layout.addWidget(Q.QLabel("Z:")); cg_layout.addWidget(self.cur_z)
        current_group.setLayout(cg_layout)
        main_layout.addWidget(current_group)

        # Target pose display
        target_group = Q.QGroupBox("Target pose [mm] + orientation [quat]")
        tg_layout = Q.QHBoxLayout()
        self.tgt_x = Q.QLineEdit(); self.tgt_x.setReadOnly(True); self.tgt_x.setFixedWidth(100)
        self.tgt_y = Q.QLineEdit(); self.tgt_y.setReadOnly(True); self.tgt_y.setFixedWidth(100)
        self.tgt_z = Q.QLineEdit(); self.tgt_z.setReadOnly(True); self.tgt_z.setFixedWidth(100)
        self.tgt_ow = Q.QLineEdit(); self.tgt_ow.setReadOnly(True); self.tgt_ow.setFixedWidth(80)
        self.tgt_ox = Q.QLineEdit(); self.tgt_ox.setReadOnly(True); self.tgt_ox.setFixedWidth(80)
        self.tgt_oy = Q.QLineEdit(); self.tgt_oy.setReadOnly(True); self.tgt_oy.setFixedWidth(80)
        self.tgt_oz = Q.QLineEdit(); self.tgt_oz.setReadOnly(True); self.tgt_oz.setFixedWidth(80)

        tg_layout.addWidget(Q.QLabel("X:")); tg_layout.addWidget(self.tgt_x)
        tg_layout.addWidget(Q.QLabel("Y:")); tg_layout.addWidget(self.tgt_y)
        tg_layout.addWidget(Q.QLabel("Z:")); tg_layout.addWidget(self.tgt_z)
        tg_layout.addWidget(Q.QLabel("qw:")); tg_layout.addWidget(self.tgt_ow)
        tg_layout.addWidget(Q.QLabel("qx:")); tg_layout.addWidget(self.tgt_ox)
        tg_layout.addWidget(Q.QLabel("qy:")); tg_layout.addWidget(self.tgt_oy)
        tg_layout.addWidget(Q.QLabel("qz:")); tg_layout.addWidget(self.tgt_oz)

        target_group.setLayout(tg_layout)
        main_layout.addWidget(target_group)

        # Jog controls (X/Y/Z)
        jog_group = Q.QGroupBox("Add to target pose [mm]")
        jg_layout = Q.QVBoxLayout()
        translation_list = [-10.0,-1.0,-0.1,-0.01,-0.001,-0.0001, 0.0001,0.001,0.01,0.1,1.0,10.0]

        self.jog_buttons = []  # keep refs if you want

        for axis in ['x', 'y', 'z']:
            row = Q.QHBoxLayout()
            row.addWidget(Q.QLabel(f"{axis.upper()}"))
            for step in translation_list:
                btn = Q.QPushButton(str(step))
                btn.setFixedWidth(60)
               
                btn._axis = axis
                btn._step = step
                self.jog_buttons.append(btn)
                row.addWidget(btn)
            jg_layout.addLayout(row)

        jog_group.setLayout(jg_layout)
        main_layout.addWidget(jog_group)

        # Buttons
        btn_row = Q.QHBoxLayout()
        self.btn_set_tgt_from_cur = Q.QPushButton("Set target = current")
        self.btn_move = Q.QPushButton("Move to target")
        self.chk_automove = Q.QCheckBox("Auto move on jog")
        btn_row.addWidget(self.btn_set_tgt_from_cur)
        btn_row.addWidget(self.btn_move)
        btn_row.addWidget(self.chk_automove)
        main_layout.addLayout(btn_row)

        # Joint state table
        # self.joint_table = Q.QTableWidget()
        # self.joint_table.setMinimumWidth(350)
        # main_layout.addWidget(self.joint_table)

        # Simple log output
        self.log_widget = Q.QPlainTextEdit()
        self.log_widget.setReadOnly(True)
        self.log_widget.setMaximumHeight(120)
        main_layout.addWidget(self.log_widget)

        self.setLayout(main_layout)


    def _connect_signals(self):
        self.tool_combo.currentTextChanged.connect(self._on_tool_changed)
        self.frame_combo.currentTextChanged.connect(self._on_frame_changed)
        self.btn_set_tgt_from_cur.clicked.connect(self._on_set_target_from_current)
        self.btn_move.clicked.connect(self._on_move_clicked)
        for btn in self.jog_buttons:
            btn.clicked.connect(lambda checked, b=btn: self._on_jog_clicked(b))


    def _on_tool_changed(self, tool: str):
        self.model.set_active_tool(tool)
        self.model.update_current_pose_from_active_tool()
        self._update_gui_from_model()

    def _on_frame_changed(self, frame: str):
        if not frame:
            return
        self.model.set_target_from_frame(frame)
        self._update_gui_from_model()

    def _on_set_target_from_current(self):
        self.model.copy_current_to_target()
        self._update_gui_from_model()

    def _on_move_clicked(self):
        success = self.model.move_to_target()
        if success:
            self._log("Move successful")
        else:
            self._log("Move failed")

    def _on_jog_clicked(self, btn: Q.QPushButton):
        axis = btn._axis  # 'x', 'y', 'z'
        step = btn._step
        vec = self.model.rel_movement
        if axis == 'x':
            vec.x += step
        elif axis == 'y':
            vec.y += step
        elif axis == 'z':
            vec.z += step

        # If automove: send immediately
        if self.chk_automove.isChecked():
            self._log(f"Add {step} mm to {axis} (auto-move)")
            self.model.move_to_target()
        else:
            self._log(f"Add {step} mm to {axis}")


    def _update_gui_from_model(self):
        m = self.model

        # update frames if changed
        if m.frame_added:
            self.frame_combo.clear()
            self.frame_combo.addItems(m.available_frames)
            m.frame_added = False

        # current pose
        self.cur_x.setText(f"{m.current_pose.position.x:.6f}")
        self.cur_y.setText(f"{m.current_pose.position.y:.6f}")
        self.cur_z.setText(f"{m.current_pose.position.z:.6f}")

        # target pose
        self.tgt_x.setText(f"{m.target_pose.position.x:.6f}")
        self.tgt_y.setText(f"{m.target_pose.position.y:.6f}")
        self.tgt_z.setText(f"{m.target_pose.position.z:.6f}")
        self.tgt_ow.setText(f"{m.target_pose.orientation.w:.6f}")
        self.tgt_ox.setText(f"{m.target_pose.orientation.x:.6f}")
        self.tgt_oy.setText(f"{m.target_pose.orientation.y:.6f}")
        self.tgt_oz.setText(f"{m.target_pose.orientation.z:.6f}")

        # joint table
        #self._update_joint_table()

    def _update_joint_table(self):
        m = self.model
        if not m.joint_state_list:
            return

        # show only name + position
        self.joint_table.setColumnCount(2)
        self.joint_table.setHorizontalHeaderLabels(["Name", "Position [rad]"])
        self.joint_table.setRowCount(len(m.joint_state_list))

        for row, (name, pos, vel, eff) in enumerate(m.joint_state_list):
            self.joint_table.setItem(row, 0, Q.QTableWidgetItem(str(name)))
            self.joint_table.setItem(row, 1, Q.QTableWidgetItem(f"{pos:.6f}"))

        self.joint_table.resizeColumnsToContents()

    def _log(self, text: str):
        self.log_widget.appendPlainText(text)
