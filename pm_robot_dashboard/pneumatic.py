from functools import partial
from typing import Awaitable

from PyQt6.QtGui import QFont
import PyQt6.QtWidgets as Q
from PyQt6.QtCore import Qt

from pm_msgs.srv import EmptyWithSuccess, PneumaticGetPosition

from pm_robot_dashboard.node import PmJogToolNode
from pm_robot_dashboard.button import OneOfManyButton
from pm_robot_dashboard.lazy import LazyClient
from pm_robot_dashboard.util import clean_topic_name

class PneumaticControlWidget(Q.QScrollArea):
    node: PmJogToolNode

    buttons: dict[str, OneOfManyButton]
    clients: dict[str, dict[str, LazyClient]]

    def __init__(self, parent: Q.QWidget, node: PmJogToolNode):
        super().__init__(parent=parent)
        self.node = node
        self.clients = {}
        self.buttons = {}

        main_layout = Q.QVBoxLayout(self)
        main_layout.setSizeConstraint(Q.QLayout.SizeConstraint.SetMinimumSize)

        get_states_button = Q.QPushButton("Get States")
        get_states_button.clicked.connect(self.get_positions)
        main_layout.addWidget(get_states_button)

        title_font = QFont("Arial", 16, QFont.Weight.Bold)
        for pneumatic in self.node.config.pneumatics:
            pneumatic = clean_topic_name(pneumatic)

            group = Q.QGroupBox()
            group_layout = Q.QVBoxLayout()

            group_title = Q.QLabel(pneumatic)
            group_title.setFont(title_font)
            group_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            group_layout.addWidget(group_title)

            buttons = OneOfManyButton(["Backward", "Forward"])
            buttons.changed.connect(partial(self.handle_click, pneumatic))
            group_layout.addWidget(buttons)

            group.setLayout(group_layout)
            main_layout.addWidget(group)

            self.buttons[pneumatic] = buttons
            self.clients[pneumatic] = {
                "status": LazyClient(self.node, PneumaticGetPosition, f"/pm_pneumatic_controller/{pneumatic}/GetPosition"),
                0: LazyClient(self.node, EmptyWithSuccess, f"/pm_pneumatic_controller/{pneumatic}/MoveBackward"),
                1: LazyClient(self.node, EmptyWithSuccess, f"/pm_pneumatic_controller/{pneumatic}/MoveForward"),
            }

        main_layout.addStretch()
        self.setLayout(main_layout)

        self.get_positions()

    def get_positions(self):
        for pneumatic in self.node.config.pneumatics:
            pneumatic = clean_topic_name(pneumatic)

            future = self.clients[pneumatic]["status"].call_async()
            if future is None:
                self.node.get_logger().error(f"Failed to get status for {pneumatic}")
                continue

            future.add_done_callback(partial(self.get_positions_success, pneumatic))

    def get_positions_success(self, pneumatic: str, future: Awaitable):
        position = future.result().position
        self.node.get_logger().info(f"Got position for {pneumatic}: {position}")
        if position == -1:
            position = 0
        else:
            position = 1
        self.buttons[pneumatic].set_active(position)

    def handle_click(self, pneumatic: str, idx: int):
        future = self.clients[pneumatic][idx].call_async()
        if future is None:
            self.node.get_logger().error(f"Failed to handle click for {pneumatic}")
            return

        future.add_done_callback(partial(self.handle_click_success, pneumatic, idx))

    def handle_click_success(self, pneumatic: str, idx: int, _future: Awaitable):
        self.buttons[pneumatic].set_active(idx)
        self.node.get_logger().info(f"Successfully handled click for {pneumatic}")
