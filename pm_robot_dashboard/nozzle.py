import threading
from functools import partial
from typing import Awaitable

from PyQt6.QtGui import QFont
import PyQt6.QtWidgets as Q
from PyQt6.QtCore import Qt

from pm_msgs.srv import EmptyWithSuccess, NozzleGetPosition

from pm_robot_dashboard.node import PmJogToolNode
from pm_robot_dashboard.button import OneOfManyButton
from pm_robot_dashboard.lazy import LazyClient
from pm_robot_dashboard.util import clean_topic_name

class NozzleControlWidget(Q.QScrollArea):
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

        for nozzle in self.node.config.nozzles:
            nozzle = clean_topic_name(nozzle)

            group = Q.QGroupBox()
            group_layout = Q.QVBoxLayout()

            group_title = Q.QLabel(nozzle)
            group_title.setFont(title_font)
            group_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            group_layout.addWidget(group_title)

            buttons = OneOfManyButton(["Vacuum", "Off", "Pressure"], 1)
            buttons.changed.connect(partial(self.handle_click, nozzle))
            group_layout.addWidget(buttons)

            group.setLayout(group_layout)
            main_layout.addWidget(group)

            self.buttons[nozzle] = buttons
            self.clients[nozzle] = {
                "status": LazyClient(self.node, NozzleGetPosition, f"/pm_nozzle_controller/{nozzle}/GetPosition"),
                0: LazyClient(self.node, EmptyWithSuccess, f"/pm_nozzle_controller/{nozzle}/Vacuum"),
                1: LazyClient(self.node, EmptyWithSuccess, f"/pm_nozzle_controller/{nozzle}/TurnOff"),
                2: LazyClient(self.node, EmptyWithSuccess, f"/pm_nozzle_controller/{nozzle}/Pressure"),
            }

        main_layout.addStretch()
        self.setLayout(main_layout)

        # Run get_positions in a separate thread to avoid blocking
        thread = threading.Thread(target=self.get_positions, daemon=True)
        thread.start()

    def get_positions(self):
        for nozzle in self.node.config.nozzles:
            nozzle = clean_topic_name(nozzle)

            future = self.clients[nozzle]["status"].call_async()
            if future is None:
                self.node.get_logger().error(f"Failed to get status for {nozzle}")
                continue

            future.add_done_callback(partial(self.get_positions_success, nozzle))

    def get_positions_success(self, nozzle: str, future: Awaitable):
        position = future.result().position
        self.buttons[nozzle].set_active(position + 1)

    def handle_click(self, nozzle: str, idx: int):
        future = self.clients[nozzle][idx].call_async()
        if future is None:
            self.node.get_logger().error(f"Failed to handle click for {nozzle}")
            return

        future.add_done_callback(partial(self.handle_click_success, nozzle, idx))

    def handle_click_success(self, nozzle: str, idx: int, _future: Awaitable):
        self.buttons[nozzle].set_active(idx)
        self.node.get_logger().info(f"Successfully handled click for {nozzle}")
