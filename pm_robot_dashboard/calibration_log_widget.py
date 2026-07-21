import json
import os
from typing import Optional

import PyQt6.QtWidgets as Q
from PyQt6.QtCore import Qt

from pm_robot_dashboard.calibration_logs import (
    CalibrationLogEvent,
    compact_json_summary,
    format_timestamp,
    list_calibration_events,
    load_json_file,
    resolve_calibration_log_sources,
)


class CalibrationLogWidget(Q.QWidget):
    def __init__(self, node=None):
        super().__init__()
        self.node = node
        self.sources = resolve_calibration_log_sources(self.node)
        self.current_mode = "Simulation"
        self.current_log_dir = self.source_path_for_mode(self.current_mode)
        self.events = []
        self._init_ui()
        self.auto_detect_mode()
        self.refresh()

    def _init_ui(self):
        main_layout = Q.QVBoxLayout(self)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        mode_button_layout = Q.QHBoxLayout()
        mode_button_layout.setSpacing(6)
        self.sim_hw_button = Q.QPushButton("Sim HW")
        self.real_hw_button = Q.QPushButton("Real HW")
        self.sim_hw_button.clicked.connect(self.set_sim_mode)
        self.real_hw_button.clicked.connect(self.set_real_mode)
        mode_button_layout.addWidget(self.sim_hw_button)
        mode_button_layout.addWidget(self.real_hw_button)

        self.refresh_button = Q.QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh)
        mode_button_layout.addWidget(self.refresh_button)

        mode_button_layout.addSpacing(12)
        mode_button_layout.addWidget(Q.QLabel("Folder:"))

        self.path_label = Q.QLineEdit()
        self.path_label.setReadOnly(True)
        self.path_label.setMinimumWidth(260)
        self.path_label.setSizePolicy(
            Q.QSizePolicy.Policy.Expanding,
            Q.QSizePolicy.Policy.Fixed,
        )
        mode_button_layout.addWidget(self.path_label, stretch=1)
        main_layout.addLayout(mode_button_layout)

        splitter = Q.QSplitter(Qt.Orientation.Horizontal)

        left_panel = Q.QWidget()
        left_layout = Q.QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)

        self.status_label = Q.QLabel()
        self.status_label.setWordWrap(True)
        left_layout.addWidget(self.status_label)

        self.table = Q.QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["Time", "Calibration Event", "JSON", "Path"])
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(Q.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(Q.QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(Q.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.itemSelectionChanged.connect(self.show_selected_event)
        left_layout.addWidget(self.table)

        splitter.addWidget(left_panel)

        right_panel = Q.QWidget()
        right_layout = Q.QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self.detail_title = Q.QLabel("Select a calibration event")
        self.detail_title.setStyleSheet("font-weight: bold; color: #0055AA;")
        self.detail_title.setWordWrap(True)
        right_layout.addWidget(self.detail_title)

        self.detail_summary = Q.QLabel()
        self.detail_summary.setWordWrap(True)
        self.detail_summary.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        right_layout.addWidget(self.detail_summary)

        self.detail_text = Q.QPlainTextEdit()
        self.detail_text.setReadOnly(True)
        right_layout.addWidget(self.detail_text)

        splitter.addWidget(right_panel)
        splitter.setSizes([520, 680])
        main_layout.addWidget(splitter)

    def source_path_for_mode(self, mode: str) -> Optional[str]:
        for source_mode, path in self.sources:
            if source_mode == mode:
                return path
        return None

    def set_sim_mode(self):
        self.current_mode = "Simulation"
        self.current_log_dir = self.source_path_for_mode(self.current_mode)
        self.update_mode_buttons(set_to_real_hw=False)
        self.refresh()

    def set_real_mode(self):
        self.current_mode = "Real Hardware"
        self.current_log_dir = self.source_path_for_mode(self.current_mode)
        self.update_mode_buttons(set_to_real_hw=True)
        self.refresh()

    def auto_detect_mode(self):
        try:
            pm_robot_utils = self.node.pm_robot_utils if self.node is not None else None
            current_mode = pm_robot_utils.get_mode() if pm_robot_utils is not None else None
            if current_mode == pm_robot_utils.REAL_MODE:
                self.current_mode = "Real Hardware"
                self.current_log_dir = self.source_path_for_mode(self.current_mode)
                self.update_mode_buttons(set_to_real_hw=True)
                return
        except Exception:
            pass

        self.current_mode = "Simulation"
        self.current_log_dir = self.source_path_for_mode(self.current_mode)
        self.update_mode_buttons(set_to_real_hw=False)

    def update_mode_buttons(self, set_to_real_hw: bool = False):
        active_style = "background-color: #4CAF50; color: white; font-weight: bold; padding: 8px 12px; border-radius: 3px; border: 2px solid #2E7D32;"
        inactive_style = "background-color: #e0e0e0; color: black; padding: 8px 12px; border-radius: 3px; border: 1px solid #999;"

        self.sim_hw_button.setStyleSheet(inactive_style)
        self.real_hw_button.setStyleSheet(inactive_style)

        if set_to_real_hw:
            self.real_hw_button.setStyleSheet(active_style)
        else:
            self.sim_hw_button.setStyleSheet(active_style)

    def refresh(self):
        self.sources = resolve_calibration_log_sources(self.node)
        self.current_log_dir = self.source_path_for_mode(self.current_mode)
        self.real_hw_button.setEnabled(self.source_path_for_mode("Real Hardware") is not None)
        self.real_hw_button.setToolTip(
            "" if self.real_hw_button.isEnabled() else "Real hardware calibration log path could not be resolved."
        )

        if not self.current_log_dir:
            self.events = []
            self.path_label.setText("unavailable")
            self.status_label.setText(f"{self.current_mode} calibration log path could not be resolved.")
            self.render_table()
            self.clear_details()
            return

        self.path_label.setText(self.current_log_dir)
        self.path_label.setToolTip(self.current_log_dir)

        try:
            self.events = list_calibration_events(self.current_log_dir, mode=self.current_mode)
        except Exception as e:
            self.events = []
            self.status_label.setText(f"Could not load calibration log index: {e}")
            self.clear_details()
            self.render_table()
            return

        if not os.path.isdir(self.current_log_dir):
            self.status_label.setText(f"{self.current_mode} calibration log folder does not exist yet.")
        elif not self.events:
            self.status_label.setText(
                f"No {self.current_mode} calibration events found in last_calibrations.yaml."
            )
        else:
            self.status_label.setText(
                f"Showing {len(self.events)} {self.current_mode} calibration event(s), newest first."
            )

        self.render_table()
        if self.events:
            self.table.selectRow(0)
        else:
            self.clear_details()

    def render_table(self):
        self.table.setRowCount(len(self.events))

        for row, event in enumerate(self.events):
            values = [
                format_timestamp(event.timestamp, event.timestamp_text),
                event.name,
                "Available" if event.json_available else "Missing",
                event.json_path or "",
            ]
            for column, value in enumerate(values):
                item = Q.QTableWidgetItem(value)
                if column == 3:
                    item.setToolTip(value)
                item.setData(Qt.ItemDataRole.UserRole, event)
                self.table.setItem(row, column, item)

        self.table.resizeColumnsToContents()
        self.table.horizontalHeader().setStretchLastSection(True)

    def selected_event(self) -> Optional[CalibrationLogEvent]:
        selected_ranges = self.table.selectedRanges()
        if not selected_ranges:
            return None

        row = selected_ranges[0].topRow()
        item = self.table.item(row, 0)
        if item is None:
            return None

        event = item.data(Qt.ItemDataRole.UserRole)
        return event if isinstance(event, CalibrationLogEvent) else None

    def show_selected_event(self):
        event = self.selected_event()
        if event is None:
            self.clear_details()
            return

        self.detail_title.setText(event.name)
        lines = [
            f"Timestamp: {format_timestamp(event.timestamp, event.timestamp_text)}",
            f"Mode: {event.mode}",
            f"YAML index: {os.path.join(event.log_dir, 'last_calibrations.yaml')}",
        ]

        if not event.json_path:
            lines.append("JSON log: missing")
            self.detail_summary.setText("\n".join(lines))
            self.detail_text.setPlainText(
                "No matching JSON log file was found for this event. "
                "The event is still listed because it exists in last_calibrations.yaml."
            )
            return

        lines.append(f"JSON log: {event.json_path}")
        try:
            data = load_json_file(event.json_path)
        except Exception as e:
            lines.append(f"JSON status: could not be read ({e})")
            self.detail_summary.setText("\n".join(lines))
            self.detail_text.setPlainText("")
            return

        summary_lines = compact_json_summary(data)
        if summary_lines:
            lines.extend(summary_lines)

        self.detail_summary.setText("\n".join(lines))
        self.detail_text.setPlainText(json.dumps(data, indent=2, sort_keys=False))

    def clear_details(self):
        self.detail_title.setText("Select a calibration event")
        self.detail_summary.setText("")
        self.detail_text.setPlainText("")
