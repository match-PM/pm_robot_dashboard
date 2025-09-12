import math
from typing import List, Dict
from dataclasses import dataclass
from functools import partial

import PyQt6.QtWidgets as Q
from PyQt6.QtCore import Qt
from PyQt6.QtCore import pyqtSignal

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
    steps: List[float]
    unit: str

class JointsControlWidget(Q.QScrollArea):
    node: PmJogToolNode
    controls: Dict[str, JointJogControl]

    components: List[Component]
    limits: Dict[str, JointLimits]

    got_description = pyqtSignal()

    def __init__(self, parent: Q.QWidget, node: PmJogToolNode):
        super().__init__(parent=parent)
        self.node = node
        self.controls = {}
        self.components = [
            Component(
                "pm_robot_gonio_right_controller",
                ["Gonio_Right_Stage_1_Joint", "Gonio_Right_Stage_2_Joint"],
                [-1.0, -0.1, -0.01, -0.001, -0.0001, 0.0001, 0.001, 0.01, 0.1, 1.0],
                "deg",
            ),
            Component(
                "pm_robot_gonio_left_controller",
                ["Gonio_Left_Stage_1_Joint", "Gonio_Left_Stage_2_Joint"],
                [-1.0, -0.1, -0.01, -0.001, -0.0001, 0.0001, 0.001, 0.01, 0.1, 1.0],
                "deg",
            ),
            Component(
                "pm_robot_t_axis_controller",
                ["T_Axis_Joint"],
                [-10.0, -1.0, -0.1, -0.01, -0.001, -0.0001, 0.0001, 0.001, 0.01, 0.1, 1.0, 10.0],
                "deg",
            ),
            Component(
                "pm_robot_xyz_axis_controller",
                ["X_Axis_Joint", "Y_Axis_Joint", "Z_Axis_Joint"],
                [-10.0, -1.0, -0.1, -0.01, -0.001, -0.0001, 0.0001, 0.001, 0.01, 0.1, 1.0, 10.0],
                "mm",
            ),
        ]
        self.limits = {}
        for comp in self.components:
            for joint in comp.joints:
                self.limits[joint] = JointLimits(-10.0, 10.0)

        robot_description_client = self.node.create_client(GetParameters, f'robot_state_publisher/get_parameters')
        while not robot_description_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Robot Description service not available, waiting...")

        desc_future = robot_description_client.call_async(GetParameters.Request(names = ['robot_description']))
        desc_future.add_done_callback(self.on_robot_description)

        self.got_description.connect(self.build_ui)

    def on_robot_description(self, future):
        future.result()
        robot_description = future.result().values[0].string_value
        urdf = URDF.from_xml_string(robot_description)
        for joint in urdf.joints:
            joint: URDFJoint
            if joint.name in self.limits:
                self.limits[joint.name] = JointLimits(joint.limit.lower, joint.limit.upper)
        self.got_description.emit()

    def build_ui(self):
        for comp in self.components:
            name = f"/{comp.name}/follow_joint_trajectory"
            control = JointJogControl(
                self.node,
                comp.joints,
                name,
                "/joint_states",
                self.limits
            )
            self.controls[comp.name] = control

        for control in self.controls.values():
            control.set_target_from_current()

        main_layout = Q.QVBoxLayout(self)
        main_layout.setSizeConstraint(Q.QLayout.SizeConstraint.SetMinimumSize)
        
        for comp in self.components:
            group_box = Q.QGroupBox(comp.name)

            group_layout = Q.QVBoxLayout()
            for joint_idx, joint in enumerate(comp.joints):
                control = self.controls[comp.name]

                joint_label = Q.QLabel(joint)
                joint_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
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
                    partial(self.update_readout, current_readout, joint_idx, comp.unit)
                )
                control.gui_update_values.gui_update_target_values_signal.connect(
                    partial(self.update_readout, target_readout, joint_idx, comp.unit)
                )

                steps_layout = Q.QHBoxLayout()
                for i, step in enumerate(comp.steps):
                    if i == int(len(comp.steps) / 2):
                        read_out_layout = Q.QVBoxLayout()
                        read_out_layout.addWidget(current_readout)
                        read_out_layout.addWidget(target_readout)
                        steps_layout.addLayout(read_out_layout)

                    button = Q.QPushButton(f"{step}")
                    button.setFixedWidth(60)
                    button.clicked.connect(
                        partial(self.change_joint_target_value, control, joint, step, comp.unit)
                    )
                    steps_layout.addWidget(button)
                group_layout.addLayout(steps_layout)

            send_button = Q.QPushButton("Send")
            send_button.clicked.connect(control.send_target_joint_values)
            group_layout.addWidget(send_button)

            group_box.setLayout(group_layout)
            main_layout.addWidget(group_box)

        self.setLayout(main_layout)

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
