import math
import threading
from typing import List, Dict
from dataclasses import dataclass
from functools import partial

import PyQt6.QtWidgets as Q
from PyQt6.QtCore import Qt
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QColor
import time
from pm_robot_dashboard.node import PmJogToolNode
from pm_robot_dashboard.controller import JointJogControl, JointLimits

from urdf_parser_py.urdf import URDF
from urdf_parser_py.urdf import Joint as URDFJoint
from rcl_interfaces.srv import GetParameters

def deg2rad(degrees: float) -> float:
    return degrees * (math.pi / 180.0)


def rad2deg(radians: float) -> float:
    return radians * (180.0 / math.pi)

@dataclass
class Component:
    name: str
    joints: List[str]
    units: List[str]  # One unit per joint
    steps: List[List[float]] = None  # Per-joint steps, auto-populate if None

class JointsControlWidget(Q.QScrollArea):
    node: PmJogToolNode
    controls: Dict[str, JointJogControl]

    components: List[Component]
    limits: Dict[str, JointLimits]

    got_description = pyqtSignal()

    ROTATIONAL_STEPS = [-1.0, -0.1, -0.01, -0.001, -0.0001, 0.0001, 0.001, 0.01, 0.1, 1.0]
    LATERAL_STEPS = [-10.0, -1.0, -0.1, -0.01, -0.001, -0.0001, 0.0001, 0.001, 0.01, 0.1, 1.0, 10.0]

    def __init__(self, parent: Q.QWidget, node: PmJogToolNode):
        super().__init__(parent=parent)
        self.node = node
        self.logger = node.get_logger()
        self.controls = {}
        self.components = [
            Component(
                "pm_robot_xyz_axis_controller",
                ["X_Axis_Joint", "Y_Axis_Joint", "Z_Axis_Joint"],
                ["mm", "mm", "mm"],
                [self.LATERAL_STEPS, self.LATERAL_STEPS, self.LATERAL_STEPS],
            ),
            Component(
                "pm_robot_t_axis_controller",
                ["T_Axis_Joint"],
                ["deg"],
                [self.ROTATIONAL_STEPS],
            ),
            Component(
                "pm_robot_gonio_right_controller",
                ["Gonio_Right_Stage_1_Joint", "Gonio_Right_Stage_2_Joint"],
                ["deg", "deg"],
                [self.ROTATIONAL_STEPS, self.ROTATIONAL_STEPS],
            ),
            Component(
                "pm_robot_gonio_left_controller",
                ["Gonio_Left_Stage_1_Joint", "Gonio_Left_Stage_2_Joint"],
                ["deg", "deg"],
                [self.ROTATIONAL_STEPS, self.ROTATIONAL_STEPS],
            ),
            Component(
                "smaract_hexapod_controller",
                ["SP_X_Joint", "SP_Y_Joint", "SP_Z_Joint", "SP_A_Joint", "SP_B_Joint", "SP_C_Joint"],
                ["mm", "mm", "mm", "deg", "deg", "deg"],
                [self.LATERAL_STEPS, 
                 self.LATERAL_STEPS, 
                 self.LATERAL_STEPS, 
                 self.ROTATIONAL_STEPS, 
                 self.ROTATIONAL_STEPS, 
                 self.ROTATIONAL_STEPS],
            ),
        ]
        self.limits = {}
        for comp in self.components:
            for joint in comp.joints:
                self.limits[joint] = JointLimits(-10.0, 10.0)

        # Show a waiting label while fetching robot description
        waiting_widget = Q.QWidget()
        waiting_layout = Q.QVBoxLayout(waiting_widget)
        waiting_layout.addStretch()
        
        waiting_label = Q.QLabel("⏳ Waiting for robot description to be published...")
        waiting_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        waiting_label.setStyleSheet("font-size: 16px; color: #666;")
        waiting_layout.addWidget(waiting_label)
        
        waiting_layout.addStretch()
        self.setWidget(waiting_widget)
        
        self.should_wait = False

        # Connect signal before starting thread
        self.got_description.connect(self.build_ui)

        # Fetch robot description in a separate thread to avoid blocking
        thread = threading.Thread(target=self.fetch_robot_description, daemon=True)
        thread.start()

    def fetch_robot_description(self):
        """Fetch robot description from service (blocking operation)."""

        robot_description_client = self.node.create_client(GetParameters, f'robot_state_publisher/get_parameters')

        while not robot_description_client.wait_for_service(timeout_sec=1.0):
            self.should_wait = True
            self.node.get_logger().info("Robot Description service not available, waiting...")

        desc_future = robot_description_client.call_async(GetParameters.Request(names = ['robot_description']))

        desc_future.add_done_callback(self.on_robot_description)

    def on_robot_description(self, future):
        future.result()

        self.logger.warn("Test")


        robot_description = future.result().values[0].string_value
        urdf = URDF.from_xml_string(robot_description)
        for joint in urdf.joints:
            joint: URDFJoint
            if joint.name in self.limits:
                self.limits[joint.name] = JointLimits(joint.limit.lower, joint.limit.upper)

        self.got_description.emit()

        self.logger.info("Successfully fetched robot description and updated joint limits, UI is now responsive.")

    def build_ui(self):

        self.logger.info("Building Joints Control UI...")

        wait_time = 8.0
        if self.should_wait:
            self.logger.warn(f"Waiting for controllers to initialize for {wait_time} seconds...")
            time.sleep(wait_time) # wait for all the controlers to initialize and fetch their current joint values before building the UI, to
        
        for comp in self.components:
            
            #self.logger.info(f"Setting up control for component '{comp.name}' with joints {comp.joints} and limits {[self.limits[joint] for joint in comp.joints]}")
            
            name = f"/{comp.name}/follow_joint_trajectory"
            # This checks if the action server is available and sets up the control, but does not block if it's not available
            control = JointJogControl(
                self.node,
                comp.joints,
                name,
                "/joint_states",
                self.limits
            )
            self.controls[comp.name] = control

        for control in self.controls.values():
            if control.is_available:
                control.set_target_from_current()
            else:
                pass
                #self.node.get_logger().warn(f"Skipping initialization for unavailable control")

        # Create a container widget to hold the layout
        container = Q.QWidget()
        main_layout = Q.QVBoxLayout(container)
        main_layout.setSizeConstraint(Q.QLayout.SizeConstraint.SetMinimumSize)

        for comp in self.components:

            control = self.controls[comp.name]
            
            # Skip UI creation for unavailable controllers
            if not control.is_available:
                self.logger.warn(f"Skipping UI for unavailable controller: {comp.name}")
                continue

            group_box = Q.QGroupBox(comp.name)
            group_layout = Q.QVBoxLayout()
            for joint_idx, joint in enumerate(comp.joints):
                control = self.controls[comp.name]

                joint_label = Q.QLabel(joint)
                joint_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                if joint == "Z_Axis_Joint":
                    joint_label.setToolTip("!!! Use negative values to move the robot up !!!")
                group_layout.addWidget(joint_label)

                current_readout = Q.QLineEdit("0.0")
                current_readout.setReadOnly(True)
                current_readout.setFixedWidth(100)
                current_readout.setToolTip(f"Current value of '{joint}'")
                current_readout.setAlignment(Qt.AlignmentFlag.AlignCenter)

                target_readout = Q.QLineEdit("0.0")
                target_readout.setReadOnly(True)
                target_readout.setFixedWidth(100)
                target_readout.setToolTip(f"Target value of '{joint}'")
                target_readout.setAlignment(Qt.AlignmentFlag.AlignCenter)

                control.gui_update_values.gui_update_current_values_signal.connect(
                    partial(self.update_readout, current_readout, joint_idx, comp.units[joint_idx])
                )
                control.gui_update_values.gui_update_target_values_signal.connect(
                    partial(self.update_readout, target_readout, joint_idx, comp.units[joint_idx])
                )

                steps_layout = Q.QHBoxLayout()
                for i, step in enumerate(comp.steps[joint_idx]):
                    if i == int(len(comp.steps[joint_idx]) / 2):
                        read_out_layout = Q.QVBoxLayout()
                        read_out_layout.addWidget(current_readout)
                        read_out_layout.addWidget(target_readout)
                        steps_layout.addLayout(read_out_layout)

                    button = Q.QPushButton(f"{step}")
                    button.setFixedWidth(60)
                    button.clicked.connect(
                        partial(self.change_joint_target_value, control, joint, step, comp.units[joint_idx])
                    )
                    steps_layout.addWidget(button)
                group_layout.addLayout(steps_layout)

            buttons_layout = Q.QHBoxLayout()
            
            set_to_current_button = Q.QPushButton("Set to Current")
            set_to_current_button.clicked.connect(control.set_target_from_current)
            buttons_layout.addWidget(set_to_current_button)
            
            send_button = Q.QPushButton("Send")
            send_button.clicked.connect(partial(self.on_send_button_clicked, control, comp))
            buttons_layout.addWidget(send_button)
            
            group_layout.addLayout(buttons_layout)

            group_box.setLayout(group_layout)
            main_layout.addWidget(group_box)

        main_layout.addStretch()
        
        # Set the container widget on the scroll area
        self.setWidget(container)
        self.setWidgetResizable(True)

    def on_send_button_clicked(self, control: JointJogControl, component: Component):
        """Handle send button click with confirmation dialog if movement is large."""
        # Check if any joint has a significant movement
        large_movements = []
        
        for joint_idx, joint_name in enumerate(component.joints):
            unit = component.units[joint_idx]
            threshold = 5.0 if unit == "deg" else 10.0  # 5 deg for rotational, 10 mm for lateral
            
            current = control.get_current_joint_value_for_joint(joint_name)
            target = control.get_target_joint_value_for_joint(joint_name)
            
            # Convert to display units for comparison
            if unit == "deg":
                diff = abs(rad2deg(target - current))
            elif unit == "mm":
                diff = abs((target - current) * 1000)
            else:
                diff = abs(target - current)
            
            if diff > threshold:
                large_movements.append((joint_name, current, target, diff, unit))
        
        # If significant movements detected, show confirmation dialog
        if large_movements:
            self.show_movement_confirmation_dialog(control, large_movements)
        else:
            control.send_target_joint_values()

    def show_movement_confirmation_dialog(self, control: JointJogControl, movements: list):
        """Show a confirmation dialog for large robot movements."""
        dialog = Q.QDialog(self)
        dialog.setWindowTitle("Confirm Robot Movement")
        dialog.setMinimumWidth(400)
        
        layout = Q.QVBoxLayout()
        
        # Warning message in red
        warning_label = Q.QLabel("⚠️ LARGE MOVEMENT DETECTED")
        warning_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
        warning_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(warning_label)
        
        # Separator
        layout.addWidget(Q.QFrame())
        
        # Movement details in red
        details_label = Q.QLabel("The following joints will move significantly:\n")
        details_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(details_label)
        
        # Movement information
        for joint_name, current, target, diff, unit in movements:
            if unit == "deg":
                current_display = rad2deg(current)
                target_display = rad2deg(target)
            elif unit == "mm":
                current_display = current * 1000
                target_display = target * 1000
            else:
                current_display = current
                target_display = target
            
            info_text = f"• {joint_name}:\n  Current: {current_display:.4f} {unit}\n  Target: {target_display:.4f} {unit}\n  Difference: {diff:.4f} {unit}"
            info_label = Q.QLabel(info_text)
            info_label.setStyleSheet("color: red;")
            layout.addWidget(info_label)
        
        # Collision warning
        layout.addWidget(Q.QFrame())
        collision_warning = Q.QLabel("⚠️ COLLISION WARNING")
        collision_warning.setStyleSheet("color: red; font-weight: bold; font-size: 12px;")
        collision_warning.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(collision_warning)
        
        collision_info = Q.QLabel(
            "This movement is NOT collision-safe checked!\n\n"
            "Please verify that:\n"
            "• The robot path is clear of obstacles\n"
            "• No collisions will occur during movement\n"
        )
        collision_info.setStyleSheet("color: red;")
        collision_info.setWordWrap(True)
        layout.addWidget(collision_info)
        
        # Question
        question_label = Q.QLabel("\nDo you really want to move the robot?")
        question_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(question_label)
        
        # Buttons
        button_layout = Q.QHBoxLayout()
        cancel_button = Q.QPushButton("Cancel")
        confirm_button = Q.QPushButton("Confirm Move")
        
        cancel_button.clicked.connect(dialog.reject)
        confirm_button.clicked.connect(dialog.accept)
        
        button_layout.addWidget(cancel_button)
        button_layout.addWidget(confirm_button)
        layout.addLayout(button_layout)
        
        dialog.setLayout(layout)
        
        # Show dialog and execute movement if confirmed
        if dialog.exec() == Q.QDialog.DialogCode.Accepted:
            control.send_target_joint_values()

    def change_joint_target_value(self, control: JointJogControl, joint_name: str, step: float, unit: str):
        current_value = control.get_target_joint_value_for_joint(joint_name)
        if unit == "deg":
            step = deg2rad(step)
        elif unit == "mm":
            step = step / 1000.0
        new_value = current_value + step
        control.set_target_joint_value(joint_name, new_value)

    def update_readout(self, readout: Q.QLineEdit, joint_idx: int, unit: str, values: List[float]):
        if unit == "deg":
            readout.setText(str(round(rad2deg(values[joint_idx]), 6)))
        elif unit == "mm":
            readout.setText(str(round(values[joint_idx] * 1000, 6)))
        else:
            readout.setText(str(round(values[joint_idx], 6)))
