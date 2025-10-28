# pm_robot_dashboard/ik_control.py

from functools import partial
import yaml

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QLabel, QPushButton,
    QComboBox, QLineEdit, QCheckBox, QHBoxLayout, QTableWidget, QTableWidgetItem
)
from PyQt6.QtCore import Qt

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Pose, Vector3
from tf2_ros import Buffer, TransformListener
from rosidl_runtime_py.convert import message_to_ordereddict

# Optional service imports
try:
    from pm_moveit_interfaces.srv import MoveToPose
    HAVE_MOVEIT_IF = True
except Exception:
    print("pm_moveit_interfaces not found, IK move functionality will be disabled.")
    HAVE_MOVEIT_IF = False

try:
    from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
    HAVE_SERVICE_ACTION = True
except Exception:
    print("ServiceAction helper not found, will use raw service client fallback.")
    HAVE_SERVICE_ACTION = False


class IkControlWidget(QWidget):
    """
    IK control tab for the PM robot dashboard.
    - Shows current tool pose (mm)
    - Lets you pick a target frame from TF
    - Jog target position in mm (X/Y/Z)
    - Move to target via pm_moveit_server services (meters on wire)
    """

    TOOLS = ["PM_Robot_Tool_TCP", "Cam1_Toolhead_TCP", "Laser_Toolhead_TCP"]
    TOOL_SERVICE = {
        "PM_Robot_Tool_TCP": "/pm_moveit_server/move_tool_to_pose",
        "Cam1_Toolhead_TCP": "/pm_moveit_server/move_cam1_to_pose",
        "Laser_Toolhead_TCP": "/pm_moveit_server/move_laser_to_pose",
    }

    BLACKLIST = set([
        "world", "map", "odom", "base_link", "pm_robot_base_link",
        "Camera_Top_View_Link", "Camera_Top_View_Link_Optical",
        "Camera_Bottom_View_Link", "Camera_Bottom_View_Link_Optical",
        "axis_base", "base_link_empthy", "housing", "housing_hr", "housing_hl",
        "housing_vl", "housing_vr",
    ])

    JOG_STEPS = [-10.0, -1.0, -0.1, -0.01, -0.001, -0.0001,
                  0.0001, 0.001, 0.01, 0.1, 1.0, 10.0]  # mm

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.logger = node.get_logger()

        # State (all UI-facing positions are in mm)
        self.active_tool = self.TOOLS[0]
        self.current_pose = Pose()  # mm
        self.target_pose = Pose()   # mm

        # TF2
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=True)

        # ROS timer group
        self.cb_group = ReentrantCallbackGroup()

        # --- Build UI FIRST ---
        self._build_ui()
        # Initial TF/UI sync
        self._refresh_frames()
        self._set_target_to_current_tool_pose()

        # --- Start timer LAST (prevents early callback before widgets exist) ---
        self.timer = self.node.create_timer(0.1, self._on_timer, callback_group=self.cb_group)

        # Capability checks
        if not HAVE_MOVEIT_IF:
            self._warn("pm_moveit_interfaces not found: Move will be disabled.")
            self.move_btn.setEnabled(False)
        if not HAVE_SERVICE_ACTION:
            self._warn("ServiceAction helper not found: will use raw service client fallback.")

    # ---------- UI ----------

    def _build_ui(self):
        layout = QVBoxLayout()
        grid = QGridLayout()

        # Active tool
        grid.addWidget(QLabel("Active Tool"), 0, 0)
        self.tool_combo = QComboBox()
        self.tool_combo.addItems(self.TOOLS)
        self.tool_combo.setCurrentText(self.active_tool)
        self.tool_combo.currentTextChanged.connect(self._on_tool_changed)
        grid.addWidget(self.tool_combo, 1, 0)

        # Target frame
        grid.addWidget(QLabel("Target Frame"), 0, 1)
        self.frame_combo = QComboBox()
        self.frame_combo.currentTextChanged.connect(self._on_target_frame_changed)
        grid.addWidget(self.frame_combo, 1, 1)

        # Current pose (read-only, mm)
        grid.addWidget(
            QLabel("Current Pose [mm] (X/Y/Z)"),
            2, 0, 1, 2,
            alignment=Qt.AlignmentFlag.AlignLeft
        )

        self.cur_x = QLineEdit()
        self.cur_y = QLineEdit()
        self.cur_z = QLineEdit()
        for w in (self.cur_x, self.cur_y, self.cur_z):
            w.setReadOnly(True)
            w.setFixedWidth(120)

        cur_row = QHBoxLayout()
        cur_row.addWidget(self.cur_x)
        cur_row.addWidget(self.cur_y)
        cur_row.addWidget(self.cur_z)

        layout.addLayout(grid)
        layout.addLayout(cur_row)

        # Target pose (read-only, mm)
        layout.addWidget(QLabel("Target Pose [mm] (X/Y/Z)"))
        self.tgt_x = QLineEdit()
        self.tgt_y = QLineEdit()
        self.tgt_z = QLineEdit()
        for w in (self.tgt_x, self.tgt_y, self.tgt_z):
            w.setReadOnly(True)
            w.setFixedWidth(120)

        tgt_row = QHBoxLayout()
        tgt_row.addWidget(self.tgt_x)
        tgt_row.addWidget(self.tgt_y)
        tgt_row.addWidget(self.tgt_z)
        layout.addLayout(tgt_row)

        # Jog buttons for X/Y/Z in mm
        layout.addWidget(QLabel("Jog Target (mm):"))
        for axis, coord in (("X", "x"), ("Y", "y"), ("Z", "z")):
            row = QHBoxLayout()
            row.addWidget(QLabel(f"{axis}:"))
            for step in self.JOG_STEPS:
                btn = QPushButton(f"{step:g}")
                btn.setFixedWidth(70)
                btn.clicked.connect(partial(self._jog_target_mm, coord, step))
                row.addWidget(btn)
            layout.addLayout(row)

        # Controls
        ctrl_row = QHBoxLayout()
        self.auto_chk = QCheckBox("Auto Move")
        set_tgt_btn = QPushButton("Set Target = Current")
        set_tgt_btn.clicked.connect(self._set_target_to_current_tool_pose)
        self.move_btn = QPushButton("Move to Target")
        self.move_btn.clicked.connect(self._move_to_target)
        ctrl_row.addWidget(self.auto_chk)
        ctrl_row.addWidget(set_tgt_btn)
        ctrl_row.addWidget(self.move_btn)
        layout.addLayout(ctrl_row)

        # Joint table placeholder (optional)
        self.table = QTableWidget()
        self.table.setMinimumWidth(420)
        self.table.setMinimumHeight(320)
        layout.addWidget(self.table)

        self.setLayout(layout)

    # ---------- Helpers ----------

    def _warn(self, msg: str):
        self.logger.warn(msg)

    def _info(self, msg: str):
        self.logger.info(msg)

    def _update_pose_fields(self):
        # Guard against early timer (shouldn’t happen with correct order, but safe)
        if not hasattr(self, "cur_x"):
            return
        self.cur_x.setText(f"{self.current_pose.position.x:.6f}")
        self.cur_y.setText(f"{self.current_pose.position.y:.6f}")
        self.cur_z.setText(f"{self.current_pose.position.z:.6f}")

        self.tgt_x.setText(f"{self.target_pose.position.x:.6f}")
        self.tgt_y.setText(f"{self.target_pose.position.y:.6f}")
        self.tgt_z.setText(f"{self.target_pose.position.z:.6f}")

    def _refresh_frames(self):
        try:
            yaml_text = self.tf_buffer.all_frames_as_yaml()
            frames_dict = yaml.safe_load(yaml_text) or {}
            frames = [f for f in frames_dict.keys() if f not in self.BLACKLIST]
            frames.sort()
            self.frame_combo.blockSignals(True)
            self.frame_combo.clear()
            self.frame_combo.addItems(frames)
            self.frame_combo.blockSignals(False)
        except Exception as e:
            # It’s normal to see exceptions early if TF isn’t ready yet
            self.logger.debug(f"TF frames not ready: {e}")

    def _mm_to_m(self, x_mm: float) -> float:
        return x_mm / 1000.0

    def _lookup_tool_pose_mm(self) -> Pose:
        pose = Pose()
        try:
            tf = self.tf_buffer.lookup_transform(
                "world", self.active_tool, Time(), timeout=Duration(seconds=0.5)
            )
            pose.position.x = tf.transform.translation.x * 1000.0
            pose.position.y = tf.transform.translation.y * 1000.0
            pose.position.z = tf.transform.translation.z * 1000.0
            pose.orientation = tf.transform.rotation
        except Exception as e:
            self.logger.debug(f"lookup tool pose failed: {e}")
        return pose

    def _set_target_to_current_tool_pose(self):
        self.current_pose = self._lookup_tool_pose_mm()
        self.target_pose.position.x = self.current_pose.position.x
        self.target_pose.position.y = self.current_pose.position.y
        self.target_pose.position.z = self.current_pose.position.z
        self.target_pose.orientation = self.current_pose.orientation
        self._update_pose_fields()

    def _set_target_from_frame(self, frame: str):
        if not frame:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                "world", frame, Time(), timeout=Duration(seconds=0.5)
            )
            self.target_pose.position.x = tf.transform.translation.x * 1000.0
            self.target_pose.position.y = tf.transform.translation.y * 1000.0
            self.target_pose.position.z = tf.transform.translation.z * 1000.0
            self.target_pose.orientation = tf.transform.rotation
            self._update_pose_fields()
        except Exception as e:
            self.logger.warn(f"Failed to set target from frame '{frame}': {e}")

    # ---------- Slots ----------

    def _on_tool_changed(self, text: str):
        self.active_tool = text
        self._info(f"Active tool = {text}")
        self._set_target_to_current_tool_pose()

    def _on_target_frame_changed(self, text: str):
        self._set_target_from_frame(text)

    def _jog_target_mm(self, coord: str, delta_mm: float):
        if coord == "x":
            self.target_pose.position.x += delta_mm
        elif coord == "y":
            self.target_pose.position.y += delta_mm
        else:
            self.target_pose.position.z += delta_mm

        self._update_pose_fields()

        if self.auto_chk.isChecked():
            self._move_to_target()

    def _move_to_target(self):
        if not HAVE_MOVEIT_IF:
            self._warn("Move disabled: pm_moveit_interfaces not available.")
            return

        service_name = self.TOOL_SERVICE.get(self.active_tool)
        if not service_name:
            self._warn(f"No service configured for tool '{self.active_tool}'.")
            return

        req = MoveToPose.Request()
        # convert mm -> m
        req.move_to_pose.position.x = self._mm_to_m(self.target_pose.position.x)
        req.move_to_pose.position.y = self._mm_to_m(self.target_pose.position.y)
        req.move_to_pose.position.z = self._mm_to_m(self.target_pose.position.z)
        req.move_to_pose.orientation = self.target_pose.orientation
        req.execute_movement = True

        success = False
        try:
            if HAVE_SERVICE_ACTION:
                sa = ServiceAction(self.node, service_name, "pm_moveit_interfaces/srv/MoveToPose")
                sa.set_service_bool_identifier("success")
                sa.set_req_message_from_dict(message_to_ordereddict(req))
                success = sa.execute()
            else:
                client = self.node.create_client(MoveToPose, service_name)
                if not client.wait_for_service(timeout_sec=2.0):
                    self._warn(f"Service {service_name} not available.")
                    return
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                if future.result() is not None:
                    success = bool(getattr(future.result(), "success", False))
        except Exception as e:
            self._warn(f"Move call failed: {e}")
            success = False

        if success:
            self._info(f"Move to target via {service_name}: success")
        else:
            self._warn(f"Move to target via {service_name}: FAILED")

    # ---------- Timer ----------

    def _on_timer(self):
        if not hasattr(self, "cur_x"):
            return
        self.current_pose = self._lookup_tool_pose_mm()
        self._update_pose_fields()
        self._refresh_frames()

    # Example: call with a list of (name, position) to fill the joint table
    def populate_joint_table(self, joint_state_list):
        if not joint_state_list:
            return
        column_names = ["Name", "Position [m]"]
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(column_names)
        self.table.setRowCount(len(joint_state_list))
        for r, (name, pos) in enumerate(joint_state_list):
            self.table.setItem(r, 0, QTableWidgetItem(str(name)))
            self.table.setItem(r, 1, QTableWidgetItem(f"{float(pos):.9f}"))
