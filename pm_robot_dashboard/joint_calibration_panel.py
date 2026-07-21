import copy
import math
import os
import shutil
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, TYPE_CHECKING, Tuple

import yaml
import PyQt6.QtWidgets as Q
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor

from pm_robot_dashboard.calibration_logs import append_manual_calibration_log

if TYPE_CHECKING:
    from pm_robot_dashboard.node import PmJogToolNode
    from pm_robot_modules.submodules.pm_robot_config import PmRobotConfig


TRANSLATION_KEYS = ("x_offset", "y_offset", "z_offset")
ROTATION_KEYS = ("rx_offset", "ry_offset", "rz_offset")
CALIBRATION_KEYS = TRANSLATION_KEYS + ROTATION_KEYS

TRANSLATION_DISPLAY_FACTORS = {
    "um": 1.0,
    "mm": 1e-3,
    "m": 1e-6,
}

TRANSLATION_DECIMAL_PLACES = {
    "um": 3,
    "mm": 6,
    "m": 9,
}

ROTATION_DECIMAL_PLACES = {
    "deg": 6,
    "rad": 6,
}


@dataclass
class CalibrationChange:
    joint_name: str
    key: str
    old_value: Optional[float]
    new_value: float


class JointCalibrationConfig:
    """Loads, holds and saves one joint calibration YAML file."""

    def __init__(self, pm_robot_config: "PmRobotConfig", use_real_hw: bool):
        self.pm_robot_config = pm_robot_config
        self.use_real_hw = use_real_hw
        self.file_path = self.pm_robot_config.get_joint_config_path(use_real_HW=use_real_hw)
        self.data: Dict[str, Dict[str, float]] = {}

    @property
    def mode_label(self) -> str:
        return "Real Hardware" if self.use_real_hw else "Simulation"

    def load(self):
        self.data = self.load_data_from_path(self.file_path)

    def load_data_from_path(self, file_path: str) -> Dict[str, Dict[str, float]]:
        if not os.path.isfile(file_path):
            raise FileNotFoundError(f"Joint calibration file does not exist: {file_path}")

        with open(file_path, "r") as file:
            loaded = yaml.safe_load(file)

        if not isinstance(loaded, dict):
            raise ValueError(f"Joint calibration file is empty or invalid: {file_path}")

        for joint_name, joint_config in loaded.items():
            if not isinstance(joint_config, dict):
                raise ValueError(
                    f"Joint '{joint_name}' calibration entry is invalid in {file_path}"
                )

        return loaded

    @property
    def archive_dir(self) -> str:
        return os.path.join(os.path.dirname(os.path.abspath(self.file_path)), "archive")

    def archive_files(self) -> List[str]:
        if not os.path.isdir(self.archive_dir):
            return []

        archive_paths = []
        for file_name in os.listdir(self.archive_dir):
            file_path = os.path.join(self.archive_dir, file_name)
            if os.path.isfile(file_path) and file_name.lower().endswith((".yaml", ".yml")):
                archive_paths.append(file_path)

        return sorted(archive_paths, key=os.path.getmtime, reverse=True)

    def joints(self) -> List[str]:
        return list(self.data.keys())

    def get_value(self, joint_name: str, key: str) -> Optional[float]:
        value = self.data.get(joint_name, {}).get(key, None)
        if value is None:
            return None
        return float(value)

    def apply_changes(self, changes: List[CalibrationChange]):
        for change in changes:
            self.data.setdefault(change.joint_name, {})
            self.data[change.joint_name][change.key] = self.normalize_base_value(
                change.key,
                change.new_value,
            )

    def save(self) -> Optional[str]:
        os.makedirs(os.path.dirname(os.path.abspath(self.file_path)), exist_ok=True)
        self.normalize_data()
        archive_path = self.archive_current_file()

        with open(self.file_path, "w") as file:
            yaml.dump(self.data, file, default_flow_style=False, sort_keys=False)

        return archive_path

    def archive_current_file(self) -> Optional[str]:
        if not os.path.isfile(self.file_path):
            return None

        os.makedirs(self.archive_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        file_stem, file_ext = os.path.splitext(os.path.basename(self.file_path))
        archive_path = os.path.join(self.archive_dir, f"{file_stem}_{timestamp}{file_ext}")

        index = 1
        while os.path.exists(archive_path):
            archive_path = os.path.join(
                self.archive_dir,
                f"{file_stem}_{timestamp}_{index}{file_ext}",
            )
            index += 1

        shutil.copy2(self.file_path, archive_path)
        return archive_path

    def normalize_data(self):
        for joint_config in self.data.values():
            for key, value in list(joint_config.items()):
                if key in CALIBRATION_KEYS and value is not None:
                    joint_config[key] = self.normalize_base_value(key, float(value))

    @staticmethod
    def normalize_base_value(key: str, value: float) -> float:
        if key in TRANSLATION_KEYS:
            return round(float(value), TRANSLATION_DECIMAL_PLACES["um"])
        if key in ROTATION_KEYS:
            return round(float(value), ROTATION_DECIMAL_PLACES["deg"])
        return float(value)


class JointCalibrationPanel(Q.QWidget):
    MODE_SIM = "Simulation"
    MODE_REAL = "Real Hardware"

    def __init__(self, node: "PmJogToolNode"):
        super().__init__()
        self.node = node

        if hasattr(node, "pm_robot_utils"):
            self._pm_robot_utils = node.pm_robot_utils
        else:
            from pm_skills.py_modules.PmRobotUtils import PmRobotUtils

            self._pm_robot_utils = PmRobotUtils(node)

        self._pm_robot_config = self._pm_robot_utils.pm_robot_config
        self._config: Optional[JointCalibrationConfig] = None
        self._loaded_data: Dict[str, Dict[str, float]] = {}
        self._updating_table = False
        self._dirty = False
        self._use_real_hw = self.detect_initial_use_real_hw()
        self._last_use_real_hw = self._use_real_hw
        self._last_translation_unit = "um"
        self._last_rotation_unit = "deg"
        self._last_archive_path: Optional[str] = None
        self._last_manual_log_path: Optional[str] = None
        self._last_manual_log_error: Optional[str] = None
        self._save_loaded_configuration = False

        self._init_ui()
        self.update_mode_buttons(set_to_real_hw=self._use_real_hw)
        self.load_selected_config()

    def _init_ui(self):
        main_layout = Q.QVBoxLayout(self)

        controls_layout = Q.QHBoxLayout()

        self.sim_hw_button = Q.QPushButton("Sim HW")
        self.real_hw_button = Q.QPushButton("Real HW")
        self.sim_hw_button.clicked.connect(self.set_sim_mode)
        self.real_hw_button.clicked.connect(self.set_real_mode)
        controls_layout.addWidget(self.sim_hw_button)
        controls_layout.addWidget(self.real_hw_button)

        self.translation_unit_combo = Q.QComboBox()
        self.translation_unit_combo.addItems(["um", "mm", "m"])
        self.translation_unit_combo.currentTextChanged.connect(lambda _text: self.on_unit_changed())
        controls_layout.addWidget(Q.QLabel("Linear Unit:"))
        controls_layout.addWidget(self.translation_unit_combo)

        self.rotation_unit_combo = Q.QComboBox()
        self.rotation_unit_combo.addItems(["deg", "rad"])
        self.rotation_unit_combo.currentTextChanged.connect(lambda _text: self.on_unit_changed())
        controls_layout.addWidget(Q.QLabel("Angular Unit:"))
        controls_layout.addWidget(self.rotation_unit_combo)

        controls_layout.addStretch()

        self.reload_button = Q.QPushButton("Reload")
        self.reload_button.clicked.connect(self.reload_config)
        controls_layout.addWidget(self.reload_button)

        self.save_button = Q.QPushButton("Save Changes")
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.save_changes)
        controls_layout.addWidget(self.save_button)

        main_layout.addLayout(controls_layout)

        archive_layout = Q.QHBoxLayout()
        archive_layout.addWidget(Q.QLabel("Archive:"))

        self.archive_combo = Q.QComboBox()
        self.archive_combo.setMinimumWidth(420)
        self.archive_combo.activated.connect(lambda _index: self.load_archive_config())
        archive_layout.addWidget(self.archive_combo)

        self.refresh_archive_button = Q.QPushButton("Refresh Archive")
        self.refresh_archive_button.clicked.connect(self.update_archive_controls)
        archive_layout.addWidget(self.refresh_archive_button)

        archive_layout.addStretch()
        main_layout.addLayout(archive_layout)

        self.mode_label = Q.QLabel()
        self.mode_label.setStyleSheet("font-weight: bold; color: #0055AA;")
        main_layout.addWidget(self.mode_label)

        self.path_label = Q.QLabel()
        self.path_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.path_label.setWordWrap(True)
        main_layout.addWidget(self.path_label)

        self.status_label = Q.QLabel()
        self.status_label.setWordWrap(True)
        main_layout.addWidget(self.status_label)

        self.table = Q.QTableWidget()
        self.table.setColumnCount(1 + len(CALIBRATION_KEYS))
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setStretchLastSection(False)
        self.table.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self.show_table_context_menu)
        self.table.itemChanged.connect(self.on_item_changed)
        main_layout.addWidget(self.table)

    def detect_initial_use_real_hw(self) -> bool:
        try:
            current_mode = self._pm_robot_utils.get_mode()
            if current_mode == self._pm_robot_utils.REAL_MODE:
                return True
            if current_mode in (
                self._pm_robot_utils.UNITY_MODE,
                self._pm_robot_utils.GAZEBO_MODE,
            ):
                return False

            self.node.get_logger().warning(
                f"Unknown PM robot mode '{current_mode}', using simulation calibration."
            )
        except Exception as e:
            self.node.get_logger().warning(
                f"Could not auto-detect PM robot mode, using simulation calibration: {e}"
            )

        return False

    def resolve_use_real_hw(self) -> Tuple[bool, str]:
        if self._use_real_hw:
            return True, self.MODE_REAL
        return False, self.MODE_SIM

    def set_sim_mode(self):
        self.switch_mode(use_real_hw=False)

    def set_real_mode(self):
        self.switch_mode(use_real_hw=True)

    def switch_mode(self, use_real_hw: bool):
        if use_real_hw == self._use_real_hw:
            self.update_mode_buttons(set_to_real_hw=use_real_hw)
            return

        previous_mode = self._use_real_hw
        if not self.prepare_for_reload_or_switch():
            self.update_mode_buttons(set_to_real_hw=previous_mode)
            return

        self._use_real_hw = use_real_hw
        self.update_mode_buttons(set_to_real_hw=use_real_hw)
        self.load_selected_config_unchecked()

    def update_mode_buttons(self, set_to_real_hw: bool = False):
        active_style = "background-color: #4CAF50; color: white; font-weight: bold; padding: 8px 12px; border-radius: 3px; border: 2px solid #2E7D32;"
        inactive_style = "background-color: #e0e0e0; color: black; padding: 8px 12px; border-radius: 3px; border: 1px solid #999;"

        self.sim_hw_button.setStyleSheet(inactive_style)
        self.real_hw_button.setStyleSheet(inactive_style)

        if set_to_real_hw:
            self.real_hw_button.setStyleSheet(active_style)
        else:
            self.sim_hw_button.setStyleSheet(active_style)

    def load_selected_config(self):
        if not self.prepare_for_reload_or_switch():
            self._use_real_hw = self._last_use_real_hw
            self.update_mode_buttons(set_to_real_hw=self._last_use_real_hw)
            return

        self.load_selected_config_unchecked()

    def load_selected_config_unchecked(self):
        use_real_hw, mode_text = self.resolve_use_real_hw()
        config = JointCalibrationConfig(self._pm_robot_config, use_real_hw=use_real_hw)

        try:
            config.load()
            config.normalize_data()
            self._config = config
            self._loaded_data = copy.deepcopy(config.data)
            self._save_loaded_configuration = False
            self.mode_label.setText(f"Loaded Configuration: {mode_text}")
            self.path_label.setText(f"File: {config.file_path}")
            self.status_label.setText("")
            self.render_table()
            self.set_dirty(False)
            self.update_archive_controls()
            self.set_archive_combo_current()
            self._last_use_real_hw = self._use_real_hw
            self._last_translation_unit = self.translation_unit_combo.currentText()
            self._last_rotation_unit = self.rotation_unit_combo.currentText()
        except Exception as e:
            self._config = config
            self._loaded_data = {}
            self._save_loaded_configuration = False
            self.clear_table()
            self.mode_label.setText(f"Loaded Configuration: {mode_text}")
            self.path_label.setText(f"File: {config.file_path}")
            self.status_label.setText(f"Error loading joint calibration: {e}")
            self.set_dirty(False)
            self.update_archive_controls()
            self.set_archive_combo_current()
            self._last_use_real_hw = self._use_real_hw
            self._last_translation_unit = self.translation_unit_combo.currentText()
            self._last_rotation_unit = self.rotation_unit_combo.currentText()

    def reload_config(self):
        self.load_selected_config()

    def update_archive_controls(self):
        self.archive_combo.blockSignals(True)
        self.archive_combo.clear()

        archive_files = self._config.archive_files() if self._config is not None else []
        enabled = self._config is not None

        if self._config is not None:
            self.archive_combo.addItem("CURRENT", "__current__")
            self.archive_combo.setItemData(
                self.archive_combo.count() - 1,
                self._config.file_path,
                Qt.ItemDataRole.ToolTipRole,
            )
            for archive_path in archive_files:
                self.archive_combo.addItem(self.format_archive_name(archive_path), archive_path)
                self.archive_combo.setItemData(
                    self.archive_combo.count() - 1,
                    archive_path,
                    Qt.ItemDataRole.ToolTipRole,
                )
        else:
            self.archive_combo.addItem("CURRENT unavailable", None)

        self.archive_combo.setEnabled(enabled)
        self.refresh_archive_button.setEnabled(self._config is not None)
        self.archive_combo.blockSignals(False)

    def set_archive_combo_current(self):
        self.archive_combo.blockSignals(True)
        for index in range(self.archive_combo.count()):
            if self.archive_combo.itemData(index) == "__current__":
                self.archive_combo.setCurrentIndex(index)
                break
        self.archive_combo.blockSignals(False)

    def set_archive_combo_current_path(self, archive_path: str):
        self.archive_combo.blockSignals(True)
        for index in range(self.archive_combo.count()):
            if self.archive_combo.itemData(index) == archive_path:
                self.archive_combo.setCurrentIndex(index)
                break
        self.archive_combo.blockSignals(False)

    def format_archive_name(self, archive_path: str) -> str:
        archive_name, _extension = os.path.splitext(os.path.basename(archive_path))

        if self._config is not None:
            active_name, _active_extension = os.path.splitext(os.path.basename(self._config.file_path))
            prefix = f"{active_name}_"
            if archive_name.startswith(prefix):
                archive_name = archive_name[len(prefix):]

        parts = archive_name.split("_")
        if len(parts) >= 2:
            timestamp = "_".join(parts[:2])
            try:
                parsed_timestamp = time.strptime(timestamp, "%Y%m%d_%H%M%S")
                display_name = time.strftime("%Y-%m-%d %H:%M:%S", parsed_timestamp)
                if len(parts) > 2:
                    display_name += f" ({'_'.join(parts[2:])})"
                return display_name
            except ValueError:
                pass

        return archive_name.replace("_", " ")

    def load_archive_config(self):
        if self._config is None:
            return

        archive_path = self.archive_combo.currentData()
        if archive_path == "__current__":
            self.reload_config()
            return

        if not archive_path:
            self.status_label.setText("No archived joint calibration file selected.")
            return

        if not self.prepare_for_reload_or_switch():
            return

        self.load_selected_config_unchecked()

        try:
            archive_data = self._config.load_data_from_path(archive_path)
            self._config.data = archive_data
            self._config.normalize_data()
            self._save_loaded_configuration = True
            self.render_table()
            self.refresh_dirty_state()
            self._last_archive_path = archive_path
            self.set_archive_combo_current_path(archive_path)
            self.status_label.setText(
                f"Loaded archive: {archive_path}. Save Changes writes it to the active file."
            )
        except Exception as e:
            self.status_label.setText(f"Error loading archived joint calibration: {e}")

    def on_unit_changed(self):
        if self._config is None:
            return

        try:
            changes = self.collect_changes(
                translation_unit=self._last_translation_unit,
                rotation_unit=self._last_rotation_unit,
            )
        except ValueError as e:
            self.status_label.setText(str(e))
            self.translation_unit_combo.blockSignals(True)
            self.rotation_unit_combo.blockSignals(True)
            self.translation_unit_combo.setCurrentText(self._last_translation_unit)
            self.rotation_unit_combo.setCurrentText(self._last_rotation_unit)
            self.translation_unit_combo.blockSignals(False)
            self.rotation_unit_combo.blockSignals(False)
            return

        working_data = copy.deepcopy(self._config.data)
        for change in changes:
            working_data.setdefault(change.joint_name, {})
            working_data[change.joint_name][change.key] = change.new_value

        self._config.data = working_data
        self.render_table()
        self.refresh_dirty_state()
        self._last_translation_unit = self.translation_unit_combo.currentText()
        self._last_rotation_unit = self.rotation_unit_combo.currentText()

    def on_item_changed(self, item: Q.QTableWidgetItem):
        if self._updating_table or self._save_loaded_configuration or item.column() == 0:
            return
        self.refresh_dirty_state()

    def show_table_context_menu(self, position):
        item = self.table.itemAt(position)
        if (
            item is None
            or item.column() == 0
            or self._config is None
            or self._save_loaded_configuration
        ):
            return

        key = CALIBRATION_KEYS[item.column() - 1]
        unit = self.unit_for_key(key)

        menu = Q.QMenu(self)
        add_action = menu.addAction(f"Add Value ({unit})")
        selected_action = menu.exec(self.table.viewport().mapToGlobal(position))

        if selected_action == add_action:
            self.prompt_add_value(item, key, unit)

    def prompt_add_value(self, item: Q.QTableWidgetItem, key: str, unit: str):
        delta, ok = Q.QInputDialog.getDouble(
            self,
            "Add Calibration Value",
            f"Add to {key} ({unit}):",
            0.0,
            -2147483647.0,
            2147483647.0,
            self.decimal_places_for_key(key),
        )
        if not ok:
            return

        try:
            self.add_display_value_to_item(item, key, delta)
        except ValueError as e:
            self.status_label.setText(str(e))

    def add_display_value_to_item(self, item: Q.QTableWidgetItem, key: str, delta: float):
        current_text = item.text().strip()
        if current_text:
            try:
                current_value = float(current_text)
            except ValueError as e:
                raise ValueError(f"Invalid current value for {key}: {current_text}") from e
        else:
            current_value = 0.0

        item.setText(self.format_display_delta_result(current_value + delta, key))

    def format_display_delta_result(self, display_value: float, key: str) -> str:
        decimals = self.decimal_places_for_key(key)
        return f"{display_value:.{decimals}f}"

    def set_dirty(self, dirty: bool):
        self._dirty = dirty
        self.save_button.setEnabled(dirty and self._config is not None)
        if self._save_loaded_configuration:
            self.save_button.setText("Restore Archive")
        else:
            self.save_button.setText("Save Changes")

    def refresh_dirty_state(self):
        try:
            self.set_dirty(bool(self.collect_changes()) or self.has_loaded_configuration_change())
        except ValueError:
            self.set_dirty(True)

    def has_loaded_configuration_change(self) -> bool:
        return self._save_loaded_configuration and self._config is not None and self._config.data != self._loaded_data

    def clear_table(self):
        self._updating_table = True
        self.table.clear()
        self.table.setRowCount(0)
        self._set_headers()
        self._updating_table = False

    def render_table(self):
        if self._config is None:
            self.clear_table()
            return

        self._updating_table = True
        self.table.clear()
        self._set_headers()
        self.table.setRowCount(len(self._config.joints()))

        for row, joint_name in enumerate(self._config.joints()):
            joint_item = Q.QTableWidgetItem(joint_name)
            joint_item.setFlags(joint_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            joint_item.setData(Qt.ItemDataRole.UserRole, joint_name)
            self.mark_joint_item_if_archived_difference(joint_item, joint_name)
            self.table.setItem(row, 0, joint_item)

            for column, key in enumerate(CALIBRATION_KEYS, start=1):
                base_value = self._config.get_value(joint_name, key)
                item = Q.QTableWidgetItem(self.format_display_value(base_value, key))
                item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                if self._save_loaded_configuration:
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                if base_value is None:
                    item.setToolTip("Missing in YAML. Enter a value to add it on save.")
                self.mark_value_item_if_archived_difference(item, joint_name, key, base_value)
                self.table.setItem(row, column, item)

        self.table.resizeColumnsToContents()
        self.table.horizontalHeader().setStretchLastSection(False)
        self._updating_table = False

    def mark_joint_item_if_archived_difference(self, item: Q.QTableWidgetItem, joint_name: str):
        if not self._save_loaded_configuration:
            return

        current_joint = self._loaded_data.get(joint_name)
        archive_joint = self._config.data.get(joint_name) if self._config is not None else None
        if current_joint != archive_joint:
            item.setBackground(QColor("#FFF3CD"))
            item.setToolTip("This joint differs from CURRENT.")

    def mark_value_item_if_archived_difference(
        self,
        item: Q.QTableWidgetItem,
        joint_name: str,
        key: str,
        archive_value: Optional[float],
    ):
        if not self._save_loaded_configuration:
            return

        current_value = self._loaded_data.get(joint_name, {}).get(key, None)
        archive_missing = archive_value is None
        current_missing = current_value is None

        if archive_missing and current_missing:
            return

        if (
            not archive_missing
            and not current_missing
            and math.isclose(float(current_value), float(archive_value), rel_tol=1e-12, abs_tol=1e-9)
        ):
            return

        current_display = "missing"
        if current_value is not None:
            current_display = self.format_display_value(float(current_value), key)

        archive_display = "missing"
        if archive_value is not None:
            archive_display = self.format_display_value(float(archive_value), key)

        item.setBackground(QColor("#FFF3CD"))
        item.setToolTip(f"CURRENT: {current_display}\nArchive: {archive_display}")

    def _set_headers(self):
        linear_unit = self.translation_unit_combo.currentText()
        angular_unit = self.rotation_unit_combo.currentText()
        headers = ["Joint"]
        headers.extend([f"{key} ({linear_unit})" for key in TRANSLATION_KEYS])
        headers.extend([f"{key} ({angular_unit})" for key in ROTATION_KEYS])
        self.table.setHorizontalHeaderLabels(headers)

    def collect_changes(
        self,
        translation_unit: Optional[str] = None,
        rotation_unit: Optional[str] = None,
    ) -> List[CalibrationChange]:
        if self._config is None:
            return []

        changes = []
        for row in range(self.table.rowCount()):
            joint_item = self.table.item(row, 0)
            joint_name = joint_item.data(Qt.ItemDataRole.UserRole) or joint_item.text()

            for column, key in enumerate(CALIBRATION_KEYS, start=1):
                item = self.table.item(row, column)
                text = item.text().strip() if item else ""
                old_value = self._loaded_data.get(joint_name, {}).get(key, None)

                if text == "":
                    if old_value is None:
                        continue
                    raise ValueError(f"Value for {joint_name}.{key} cannot be empty.")

                try:
                    display_value = float(text)
                except ValueError as e:
                    raise ValueError(f"Invalid number for {joint_name}.{key}: {text}") from e

                new_value = self.display_to_base_value(
                    display_value,
                    key,
                    translation_unit=translation_unit,
                    rotation_unit=rotation_unit,
                )
                new_value = JointCalibrationConfig.normalize_base_value(key, new_value)
                if old_value is None or not math.isclose(
                    float(old_value), new_value, rel_tol=1e-12, abs_tol=1e-9
                ):
                    changes.append(
                        CalibrationChange(
                            joint_name=joint_name,
                            key=key,
                            old_value=None if old_value is None else float(old_value),
                            new_value=new_value,
                        )
                    )

        return changes

    def save_changes(self):
        if self._config is None:
            return

        try:
            changes = self.collect_changes()
        except ValueError as e:
            self.status_label.setText(str(e))
            return

        has_configuration_change = self.has_loaded_configuration_change()
        if not changes and not has_configuration_change:
            self.status_label.setText("No calibration changes to save.")
            self.set_dirty(False)
            return

        restoring_archive = self.has_loaded_configuration_change()

        if not self.confirm_save(changes):
            return

        if self.apply_changes_to_file(changes):
            self.load_selected_config_unchecked()
            archive_text = ""
            if self._last_archive_path:
                archive_text = f" Archived previous version to: {self._last_archive_path}"
            log_text = ""
            if self._last_manual_log_path:
                log_text = f" Manual change log: {self._last_manual_log_path}"
            elif self._last_manual_log_error:
                log_text = f" Manual change was saved, but logging failed: {self._last_manual_log_error}"
            action_text = f"Saved {len(changes)} calibration value(s)."
            if restoring_archive:
                action_text = f"Restored archived configuration with {len(changes)} value-level difference(s)."
            self.status_label.setText(
                f"{action_text}{archive_text}{log_text}"
            )

    def prepare_for_reload_or_switch(
        self,
        translation_unit: Optional[str] = None,
        rotation_unit: Optional[str] = None,
    ) -> bool:
        if self._save_loaded_configuration:
            return True

        try:
            changes = self.collect_changes(
                translation_unit=translation_unit,
                rotation_unit=rotation_unit,
            )
        except ValueError as e:
            self.status_label.setText(str(e))
            return False

        if not changes:
            self.set_dirty(False)
            return True

        decision = self.confirm_apply_or_discard_changes(changes)
        if decision == "apply":
            return self.apply_changes_to_file(changes)
        if decision == "discard":
            self.set_dirty(False)
            return True
        return False

    def apply_changes_to_file(self, changes: List[CalibrationChange]) -> bool:
        try:
            self._last_manual_log_path = None
            self._last_manual_log_error = None
            if self._save_loaded_configuration:
                self._config.data = copy.deepcopy(self._config.data)
            else:
                self._config.data = copy.deepcopy(self._loaded_data)
            self._config.apply_changes(changes)
            self._last_archive_path = self._config.save()
            try:
                self._last_manual_log_path = append_manual_calibration_log(
                    changes=self.changes_to_log_entries(changes),
                    mode=self._config.mode_label,
                    active_file=self._config.file_path,
                    archive_file=self._last_archive_path,
                )
            except Exception as e:
                self._last_manual_log_error = str(e)
                self.node.get_logger().warning(
                    f"Could not write manual calibration log: {e}"
                )
            self.update_archive_controls()
            self.set_archive_combo_current()
            self.set_dirty(False)
            self._save_loaded_configuration = False
            return True
        except Exception as e:
            self.status_label.setText(f"Error saving joint calibration: {e}")
            return False

    def changes_to_log_entries(self, changes: List[CalibrationChange]) -> List[Dict[str, object]]:
        entries = []
        for change in changes:
            entries.append(
                {
                    "joint_name": change.joint_name,
                    "key": change.key,
                    "old_value": change.old_value,
                    "new_value": change.new_value,
                    "display_unit": self.unit_for_key(change.key),
                    "old_display_value": (
                        None
                        if change.old_value is None
                        else self.format_display_value(change.old_value, change.key)
                    ),
                    "new_display_value": self.format_display_value(change.new_value, change.key),
                }
            )
        return entries

    def confirm_save(self, changes: List[CalibrationChange]) -> bool:
        dialog = Q.QMessageBox(self)
        dialog.setIcon(Q.QMessageBox.Icon.Warning)
        dialog.setWindowTitle("Confirm Joint Calibration Change")
        if self.has_loaded_configuration_change():
            dialog.setText("Restore the selected archived configuration to CURRENT?")
            change_summary = (
                f"The archived configuration will replace the active file.\n"
                f"{len(changes)} value-level difference(s) detected."
            )
        else:
            dialog.setText("Do you really want to modify the current joint calibration file?")
            change_summary = f"{len(changes)} value(s) will be changed."
        dialog.setInformativeText(
            f"Mode: {self._config.mode_label}\n"
            f"File: {self._config.file_path}\n\n"
            f"{change_summary}"
        )
        if changes:
            dialog.setDetailedText("\n".join(self.format_change(change) for change in changes))
        dialog.setStandardButtons(
            Q.QMessageBox.StandardButton.Cancel | Q.QMessageBox.StandardButton.Yes
        )
        dialog.setDefaultButton(Q.QMessageBox.StandardButton.Cancel)
        return dialog.exec() == Q.QMessageBox.StandardButton.Yes

    def confirm_apply_or_discard_changes(self, changes: List[CalibrationChange]) -> str:
        dialog = Q.QMessageBox(self)
        dialog.setIcon(Q.QMessageBox.Icon.Warning)
        dialog.setWindowTitle("Unsaved Joint Calibration Changes")
        dialog.setText("Apply or discard the changed joint calibration values?")
        change_summary = f"{len(changes)} value(s) changed."
        if self.has_loaded_configuration_change():
            change_summary = (
                f"The loaded configuration differs from the active file.\n"
                f"{len(changes)} value-level difference(s) detected."
            )
        dialog.setInformativeText(
            f"File: {self._config.file_path}\n\n"
            f"{change_summary}"
        )
        if changes:
            dialog.setDetailedText("\n".join(self.format_change(change) for change in changes))

        apply_button = dialog.addButton("Apply", Q.QMessageBox.ButtonRole.AcceptRole)
        discard_button = dialog.addButton("Discard", Q.QMessageBox.ButtonRole.DestructiveRole)
        cancel_button = dialog.addButton(Q.QMessageBox.StandardButton.Cancel)
        dialog.setDefaultButton(cancel_button)
        dialog.exec()

        clicked_button = dialog.clickedButton()
        if clicked_button == apply_button:
            return "apply"
        if clicked_button == discard_button:
            return "discard"
        return "cancel"

    def format_change(self, change: CalibrationChange) -> str:
        old_display = "missing"
        if change.old_value is not None:
            old_display = self.format_display_value(change.old_value, change.key)

        new_display = self.format_display_value(change.new_value, change.key)
        unit = self.unit_for_key(change.key)
        return f"{change.joint_name}.{change.key}: {old_display} -> {new_display} {unit}"

    def format_display_value(self, base_value: Optional[float], key: str) -> str:
        if base_value is None:
            return ""
        display_value = self.base_to_display_value(float(base_value), key)
        decimals = self.decimal_places_for_key(key)
        return f"{display_value:.{decimals}f}"

    def base_to_display_value(self, base_value: float, key: str) -> float:
        if key in TRANSLATION_KEYS:
            return base_value * TRANSLATION_DISPLAY_FACTORS[self.translation_unit_combo.currentText()]
        if self.rotation_unit_combo.currentText() == "rad":
            return math.radians(base_value)
        return base_value

    def display_to_base_value(
        self,
        display_value: float,
        key: str,
        translation_unit: Optional[str] = None,
        rotation_unit: Optional[str] = None,
    ) -> float:
        if key in TRANSLATION_KEYS:
            unit = translation_unit or self.translation_unit_combo.currentText()
            return display_value / TRANSLATION_DISPLAY_FACTORS[unit]

        unit = rotation_unit or self.rotation_unit_combo.currentText()
        if unit == "rad":
            return math.degrees(display_value)
        return display_value

    def unit_for_key(self, key: str) -> str:
        if key in TRANSLATION_KEYS:
            return self.translation_unit_combo.currentText()
        return self.rotation_unit_combo.currentText()

    def decimal_places_for_key(self, key: str) -> int:
        if key in TRANSLATION_KEYS:
            return TRANSLATION_DECIMAL_PLACES[self.translation_unit_combo.currentText()]
        return ROTATION_DECIMAL_PLACES[self.rotation_unit_combo.currentText()]
