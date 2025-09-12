from typing import List
from functools import partial

from PyQt6.QtCore import pyqtSignal
import PyQt6.QtWidgets as Q


class OneOfManyButton(Q.QWidget):
    changed = pyqtSignal(int, name="changed")

    buttons: List[Q.QPushButton]

    def __init__(self, options: List[str], default_active: int = 0):
        super().__init__(parent=None)

        layout = Q.QHBoxLayout()

        self.buttons = []
        for i, option in enumerate(options):
            button = Q.QPushButton(option)
            button.clicked.connect(partial(self.handle_click, i))
            layout.addWidget(button)
            self.buttons.append(button)

        self.setLayout(layout)

        self.set_active(default_active)

    def set_active(self, idx: int):
        if idx >= len(self.buttons):
            raise IndexError("idx out of range")

        for i, button in enumerate(self.buttons):
            if i == idx:
                button.setStyleSheet("background-color: green")
            else:
                button.setStyleSheet("background-color: grey")

    def handle_click(self, idx: int):
        self.changed.emit(idx)
