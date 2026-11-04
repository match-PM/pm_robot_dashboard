"""PM robot bringup configuration editor and requirement-file creator."""

import sys
from pathlib import Path

import yaml
from PyQt6.QtCore import QSignalBlocker, Qt
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTreeWidget,
    QTreeWidgetItem,
    QTreeWidgetItemIterator,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node

from pm_robot_modules.submodules.configuration_requirements import (
    DEFAULT_CONFIGURATION_FILE_NAME,
)
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils


class ConfigurationRequirementDialog(QDialog):
    """Select bringup properties and save them as a partial YAML mapping."""

    PATH_ROLE = int(Qt.ItemDataRole.UserRole)
    EXCLUDED_KEYS = {'unity_parent'}
    EXCLUDED_PREFIXES = ('available_', 'availabe_')

    def __init__(self, configuration: dict, parent=None):
        super().__init__(parent)
        self.configuration = configuration
        self.setWindowTitle('Create PM Robot Configuration Requirement')
        self.resize(760, 680)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        title = QLabel('Select the properties this assembly process requires')
        title.setObjectName('dialogTitle')
        layout.addWidget(title)

        description = QLabel(
            'Only checked properties are written. Unchecked properties are '
            'ignored when check_pm_robot_configuration is called.'
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        selection_buttons = QHBoxLayout()
        select_all_button = QPushButton('Select all')
        clear_all_button = QPushButton('Clear all')
        select_all_button.clicked.connect(lambda: self._set_all_checked(True))
        clear_all_button.clicked.connect(lambda: self._set_all_checked(False))
        selection_buttons.addWidget(select_all_button)
        selection_buttons.addWidget(clear_all_button)
        selection_buttons.addStretch()
        layout.addLayout(selection_buttons)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(['Configuration property', 'Current value'])
        self.tree.setAlternatingRowColors(True)
        self.tree.setColumnWidth(0, 480)
        layout.addWidget(self.tree, 1)

        self._populate_tree(
            self.tree.invisibleRootItem(),
            self.configuration,
            (),
        )
        self.tree.expandToDepth(1)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save
            | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.button(QDialogButtonBox.StandardButton.Save).setText(
            'Create config file'
        )
        buttons.accepted.connect(self._save_selected_configuration)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.setStyleSheet(
            '#dialogTitle { font-size: 16px; font-weight: bold; }'
            'QTreeWidget { border: 1px solid #b8b8b8; }'
        )

    @classmethod
    def _is_excluded_key(cls, key: str) -> bool:
        return key in cls.EXCLUDED_KEYS or key.startswith(
            cls.EXCLUDED_PREFIXES
        )

    @staticmethod
    def _display_name(key: str) -> str:
        return key.replace('_', ' ').strip().title()

    @staticmethod
    def _display_value(value) -> str:
        if isinstance(value, bool):
            return 'true' if value else 'false'
        if value is None:
            return 'null'
        return str(value)

    def _populate_tree(self, parent, value, path):
        if not isinstance(value, dict):
            return

        for key, child_value in value.items():
            if self._is_excluded_key(str(key)) or isinstance(child_value, list):
                continue

            child_path = path + (key,)
            item = QTreeWidgetItem(parent)
            item.setText(0, self._display_name(str(key)))
            item.setToolTip(0, '.'.join(str(part) for part in child_path))

            if isinstance(child_value, dict):
                item.setFlags(
                    item.flags()
                    | Qt.ItemFlag.ItemIsUserCheckable
                    | Qt.ItemFlag.ItemIsAutoTristate
                )
                item.setCheckState(0, Qt.CheckState.Checked)
                self._populate_tree(item, child_value, child_path)
                if item.childCount() == 0:
                    parent.removeChild(item)
                continue

            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(0, Qt.CheckState.Checked)
            item.setText(1, self._display_value(child_value))
            item.setData(0, self.PATH_ROLE, list(child_path))

    def _set_all_checked(self, checked: bool):
        state = Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked
        iterator = QTreeWidgetItemIterator(self.tree)
        while iterator.value():
            item = iterator.value()
            if item.data(0, self.PATH_ROLE) is not None:
                item.setCheckState(0, state)
            iterator += 1

    def _value_at_path(self, path):
        value = self.configuration
        for key in path:
            value = value[key]
        return value

    @staticmethod
    def _set_nested_value(target: dict, path, value):
        destination = target
        for key in path[:-1]:
            destination = destination.setdefault(key, {})
        destination[path[-1]] = value

    def selected_configuration(self) -> dict:
        selected = {}
        iterator = QTreeWidgetItemIterator(self.tree)
        while iterator.value():
            item = iterator.value()
            path = item.data(0, self.PATH_ROLE)
            if path is not None and item.checkState(0) == Qt.CheckState.Checked:
                self._set_nested_value(
                    selected,
                    path,
                    self._value_at_path(path),
                )
            iterator += 1
        return selected

    def _save_selected_configuration(self):
        selected = self.selected_configuration()
        if not selected:
            QMessageBox.warning(
                self,
                'No properties selected',
                'Select at least one configuration property.',
            )
            return

        suggested_path = str(Path.cwd() / DEFAULT_CONFIGURATION_FILE_NAME)
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            'Create PM robot configuration requirement',
            suggested_path,
            'YAML files (*.yaml *.yml)',
        )
        if not file_path:
            return

        output_path = Path(file_path)
        if output_path.suffix.lower() not in ('.yaml', '.yml'):
            output_path = output_path.with_suffix('.yaml')

        try:
            with output_path.open('w', encoding='utf-8') as output_file:
                yaml.safe_dump(
                    selected,
                    output_file,
                    default_flow_style=False,
                    sort_keys=False,
                )
        except OSError as error:
            QMessageBox.critical(
                self,
                'Could not create configuration file',
                str(error),
            )
            return

        QMessageBox.information(
            self,
            'Configuration file created',
            f'Created partial configuration requirement:\n{output_path}',
        )
        self.accept()


class PmRobotConfigWidget(QWidget):
    """Grouped editor for the active PM robot bringup configuration."""

    MIN_COMBO_WIDTH = 260

    def __init__(self, ros_node: Node):
        super().__init__()
        self.node = ros_node
        if hasattr(ros_node, 'pm_robot_utils'):
            self._pm_robot_utils = ros_node.pm_robot_utils
        else:
            self._pm_robot_utils = PmRobotUtils(ros_node)
        self.pm_robot_config = self._pm_robot_utils.pm_robot_config

        self._build_ui()
        self._connect_signals()
        self.auto_detect_mode()

    def _build_ui(self):
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(14, 14, 14, 14)
        root_layout.setSpacing(12)

        mode_frame = QFrame()
        mode_frame.setObjectName('modeFrame')
        mode_layout = QHBoxLayout(mode_frame)
        mode_layout.setContentsMargins(10, 8, 10, 8)

        mode_title = QLabel('Configuration source')
        mode_title.setObjectName('sectionTitle')
        mode_layout.addWidget(mode_title)
        mode_layout.addStretch()

        self.sim_hw_button = QPushButton('Simulation')
        self.real_hw_button = QPushButton('Real hardware')
        self.auto_detect_button = QPushButton('Auto detect')
        mode_layout.addWidget(self.sim_hw_button)
        mode_layout.addWidget(self.real_hw_button)
        mode_layout.addWidget(self.auto_detect_button)
        root_layout.addWidget(mode_frame)

        self.config_mode_label = QLabel()
        self.config_mode_label.setObjectName('modeLabel')
        self.config_path_label = QLabel()
        self.config_path_label.setObjectName('pathLabel')
        self.config_path_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        root_layout.addWidget(self.config_mode_label)
        root_layout.addWidget(self.config_path_label)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget()
        content_layout = QGridLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setHorizontalSpacing(12)
        content_layout.setVerticalSpacing(12)

        content_layout.addWidget(self._build_vacuum_group(), 0, 0)
        content_layout.addWidget(self._build_parallel_gripper_group(1), 0, 1)
        content_layout.addWidget(self._build_parallel_gripper_group(2), 1, 0)
        content_layout.addWidget(self._build_dispenser_group(), 1, 1)
        content_layout.addWidget(self._build_measurement_group(), 2, 0)
        content_layout.addWidget(self._build_station_group(), 2, 1)
        content_layout.setColumnStretch(0, 1)
        content_layout.setColumnStretch(1, 1)
        content_layout.setRowStretch(3, 1)

        scroll_area.setWidget(content)
        root_layout.addWidget(scroll_area, 1)

        action_layout = QHBoxLayout()
        self.load_config_button = QPushButton('Load config file')
        self.load_config_button.setObjectName('secondaryAction')
        self.create_config_button = QPushButton('Create config file')
        self.create_config_button.setObjectName('secondaryAction')
        self.reload_button = QPushButton('Reload')
        self.save_button = QPushButton('Save bringup config')
        self.save_button.setObjectName('primaryAction')
        action_layout.addWidget(self.load_config_button)
        action_layout.addWidget(self.create_config_button)
        action_layout.addStretch()
        action_layout.addWidget(self.reload_button)
        action_layout.addWidget(self.save_button)
        root_layout.addLayout(action_layout)

        self.setStyleSheet(
            '#modeFrame { background: #f0f3f6; border: 1px solid #ccd3da; '
            'border-radius: 5px; }'
            '#sectionTitle { font-size: 14px; font-weight: bold; }'
            '#modeLabel { font-size: 13px; font-weight: bold; color: #0055aa; }'
            '#pathLabel { color: #555; }'
            'QGroupBox { font-weight: bold; border: 1px solid #c8c8c8; '
            'border-radius: 5px; margin-top: 10px; padding-top: 10px; }'
            'QGroupBox::title { subcontrol-origin: margin; left: 10px; '
            'padding: 0 4px; }'
            'QPushButton { padding: 7px 12px; }'
            '#primaryAction { background: #1976d2; color: white; '
            'font-weight: bold; }'
            '#secondaryAction { font-weight: bold; }'
        )

    def _new_combo(self):
        combo = QComboBox()
        combo.setMinimumWidth(self.MIN_COMBO_WIDTH)
        combo.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed,
        )
        return combo

    def _build_vacuum_group(self):
        group = QGroupBox('Vacuum gripper')
        layout = QFormLayout(group)
        self.box_activate_vacuum = QCheckBox('Active')
        self.vacuum_tool_combobox = self._new_combo()
        self.vacuum_tip_combobox = self._new_combo()
        layout.addRow('State', self.box_activate_vacuum)
        layout.addRow('Tool', self.vacuum_tool_combobox)
        layout.addRow('Attachment', self.vacuum_tip_combobox)
        return group

    def _build_parallel_gripper_group(self, number):
        group = QGroupBox(f'Parallel gripper {number} jaw')
        layout = QFormLayout(group)
        active = QCheckBox('Active')
        tool = self._new_combo()
        attachment = self._new_combo()

        if number == 1:
            self.box_activate_gripper_1_jaw = active
            self.gripper_1_jaw_combobox = tool
            self.gripper_1_jaw_tip_combobox = attachment
        else:
            self.box_activate_gripper_2_jaw = active
            self.gripper_2_jaw_combobox = tool
            self.gripper_2_jaw_tip_combobox = attachment

        layout.addRow('State', active)
        layout.addRow('Tool', tool)
        layout.addRow('Jaw', attachment)
        return group

    def _build_dispenser_group(self):
        group = QGroupBox('Dispenser tips')
        layout = QFormLayout(group)
        self.dispenser_tip_combobox = self._new_combo()
        self.dispenser_2k_tip_combobox = self._new_combo()
        layout.addRow('1K tip', self.dispenser_tip_combobox)
        layout.addRow('2K tip', self.dispenser_2k_tip_combobox)
        return group

    def _build_measurement_group(self):
        group = QGroupBox('Measurement systems')
        layout = QFormLayout(group)
        self.use_confocal_head_checkbox = QCheckBox('Enabled')
        self.use_confocal_bottom_checkbox = QCheckBox('Enabled')
        self.use_keyence_bottom_checkbox = QCheckBox('Enabled')
        self.confocal_sensor_head_combobox = self._new_combo()
        layout.addRow('Confocal head', self.use_confocal_head_checkbox)
        layout.addRow('Head configuration', self.confocal_sensor_head_combobox)
        layout.addRow('Confocal bottom', self.use_confocal_bottom_checkbox)
        layout.addRow('Keyence bottom', self.use_keyence_bottom_checkbox)
        return group

    def _build_station_group(self):
        group = QGroupBox('Stations and chucks')
        layout = QGridLayout(group)
        layout.addWidget(QLabel('Station'), 0, 0)
        layout.addWidget(QLabel('Active'), 0, 1)
        layout.addWidget(QLabel('Chuck'), 0, 2)

        self.box_activate_gonio_left = QCheckBox()
        self.box_activate_gonio_right = QCheckBox()
        self.box_activate_smarpod = QCheckBox()
        self.gonio_left_combobox = self._new_combo()
        self.gonio_right_combobox = self._new_combo()
        self.smarpod_combobox = self._new_combo()
        self.smarpod_combobox_chuck_center = self._new_combo()

        rows = (
            ('Gonio left', self.box_activate_gonio_left,
             self.gonio_left_combobox),
            ('Gonio right', self.box_activate_gonio_right,
             self.gonio_right_combobox),
            ('SmarPod', self.box_activate_smarpod,
             self.smarpod_combobox),
        )
        for row, (label, checkbox, combo) in enumerate(rows, start=1):
            layout.addWidget(QLabel(label), row, 0)
            layout.addWidget(checkbox, row, 1)
            layout.addWidget(combo, row, 2)

        layout.addWidget(QLabel('SmarPod center'), 4, 0)
        layout.addWidget(self.smarpod_combobox_chuck_center, 4, 2)
        layout.setColumnStretch(2, 1)
        return group

    def _connect_signals(self):
        self.sim_hw_button.clicked.connect(self.set_sim_mode)
        self.real_hw_button.clicked.connect(self.set_real_mode)
        self.auto_detect_button.clicked.connect(self.auto_detect_mode)
        self.save_button.clicked.connect(self.save_config)
        self.reload_button.clicked.connect(self.reload_config)
        self.load_config_button.clicked.connect(self.load_config_file)
        self.create_config_button.clicked.connect(self.create_config_file)

        self.box_activate_vacuum.clicked.connect(
            lambda checked: self._set_active_tool('vacuum', checked)
        )
        self.box_activate_gripper_1_jaw.clicked.connect(
            lambda checked: self._set_active_tool('gripper_1', checked)
        )
        self.box_activate_gripper_2_jaw.clicked.connect(
            lambda checked: self._set_active_tool('gripper_2', checked)
        )

        self.vacuum_tool_combobox.currentTextChanged.connect(
            lambda text: self._tool_changed('vacuum', text)
        )
        self.gripper_1_jaw_combobox.currentTextChanged.connect(
            lambda text: self._tool_changed('gripper_1', text)
        )
        self.gripper_2_jaw_combobox.currentTextChanged.connect(
            lambda text: self._tool_changed('gripper_2', text)
        )
        self.vacuum_tip_combobox.currentTextChanged.connect(
            lambda text: self._attachment_changed('vacuum', text)
        )
        self.gripper_1_jaw_tip_combobox.currentTextChanged.connect(
            lambda text: self._attachment_changed('gripper_1', text)
        )
        self.gripper_2_jaw_tip_combobox.currentTextChanged.connect(
            lambda text: self._attachment_changed('gripper_2', text)
        )

        self.dispenser_tip_combobox.currentTextChanged.connect(
            self._set_dispenser_1k_tip
        )
        self.dispenser_2k_tip_combobox.currentTextChanged.connect(
            self._set_dispenser_2k_tip
        )
        self.confocal_sensor_head_combobox.currentTextChanged.connect(
            self._set_confocal_sensor_head
        )

        self.box_activate_gonio_left.clicked.connect(
            lambda checked: self._set_station_active('gonio_left', checked)
        )
        self.box_activate_gonio_right.clicked.connect(
            lambda checked: self._set_station_active('gonio_right', checked)
        )
        self.box_activate_smarpod.clicked.connect(
            lambda checked: self._set_station_active('smarpod', checked)
        )
        self.gonio_left_combobox.currentTextChanged.connect(
            lambda text: self._set_station_chuck('gonio_left', text)
        )
        self.gonio_right_combobox.currentTextChanged.connect(
            lambda text: self._set_station_chuck('gonio_right', text)
        )
        self.smarpod_combobox.currentTextChanged.connect(
            lambda text: self._set_station_chuck('smarpod', text)
        )
        self.smarpod_combobox_chuck_center.currentTextChanged.connect(
            self._set_smarpod_chuck_center
        )

        self.use_confocal_head_checkbox.clicked.connect(
            lambda checked: self._set_measurement_enabled(
                'use_confocal_head', checked
            )
        )
        self.use_confocal_bottom_checkbox.clicked.connect(
            lambda checked: self._set_measurement_enabled(
                'use_confocal_bottom', checked
            )
        )
        self.use_keyence_bottom_checkbox.clicked.connect(
            lambda checked: self._set_measurement_enabled(
                'use_keyence_bottom', checked
            )
        )

    def _tool(self, tool_key):
        tools = self.pm_robot_config.tool
        return {
            'vacuum': tools._gripper_vacuum,
            'gripper_1': tools._gripper_1_jaw,
            'gripper_2': tools._gripper_2_jaw,
        }[tool_key]

    def _station(self, station_key):
        return {
            'gonio_left': self.pm_robot_config.gonio_left,
            'gonio_right': self.pm_robot_config.gonio_right,
            'smarpod': self.pm_robot_config.smarpod_station,
        }[station_key]

    def _set_dispenser_1k_tip(self, tip):
        if tip:
            self.pm_robot_config.dispenser_1k.set_currrent_dispenser_tip(tip)

    def _set_dispenser_2k_tip(self, tip):
        if tip:
            self.pm_robot_config.dispenser_2k.set_currrent_dispenser_tip(tip)

    def _set_confocal_sensor_head(self, config_name):
        if config_name:
            self.pm_robot_config.confocal_sensor_head.set_current_config(
                config_name
            )

    def _set_station_chuck(self, station_key, chuck):
        if chuck:
            self._station(station_key).set_current_chuck(chuck)

    def _set_smarpod_chuck_center(self, chuck):
        if chuck:
            self.pm_robot_config.smarpod_station.set_current_chuck_center(chuck)

    def _set_active_tool(self, tool_key, checked):
        selected_tool = self._tool(tool_key)
        if not checked:
            selected_tool.activate()
            self._refresh_tool_controls()
            return

        for key in ('vacuum', 'gripper_1', 'gripper_2'):
            self._tool(key).deactivate()
        selected_tool.activate()
        self._refresh_tool_controls()

    def _tool_changed(self, tool_key, tool_name):
        if not tool_name:
            return
        tool = self._tool(tool_key)
        tool.set_current_tool(tool=tool_name)
        attachments = tool.get_current_extension_list()
        attachment = attachments[0] if attachments else ''
        tool.set_current_tool(ext=attachment)
        self._refresh_attachment_combo(tool_key)

    def _attachment_changed(self, tool_key, attachment):
        if attachment:
            self._tool(tool_key).set_current_tool(ext=attachment)

    def _set_station_active(self, station_key, checked):
        station = self._station(station_key)
        station.activate() if checked else station.deactivate()

    def _set_measurement_enabled(self, key, checked):
        self.pm_robot_config._config_data['measuring_systems'][key] = checked

    @staticmethod
    def _set_combo_items(combo, items, current):
        with QSignalBlocker(combo):
            combo.clear()
            combo.addItems([str(item) for item in items])
            combo.setCurrentText(str(current) if current is not None else '')

    @staticmethod
    def _set_checked(checkbox, checked):
        with QSignalBlocker(checkbox):
            checkbox.setChecked(bool(checked))

    def _refresh_attachment_combo(self, tool_key):
        combo = {
            'vacuum': self.vacuum_tip_combobox,
            'gripper_1': self.gripper_1_jaw_tip_combobox,
            'gripper_2': self.gripper_2_jaw_tip_combobox,
        }[tool_key]
        tool = self._tool(tool_key)
        self._set_combo_items(
            combo,
            tool.get_current_extension_list(),
            tool.get_current_tool_attachment(),
        )

    def _refresh_tool_controls(self):
        controls = {
            'vacuum': (
                self.box_activate_vacuum,
                self.vacuum_tool_combobox,
            ),
            'gripper_1': (
                self.box_activate_gripper_1_jaw,
                self.gripper_1_jaw_combobox,
            ),
            'gripper_2': (
                self.box_activate_gripper_2_jaw,
                self.gripper_2_jaw_combobox,
            ),
        }
        for key, (checkbox, combo) in controls.items():
            tool = self._tool(key)
            self._set_checked(checkbox, tool.get_activate_status())
            self._set_combo_items(
                combo,
                [item[0] for item in tool.get_available_tools()],
                tool.get_current_tool(),
            )
            self._refresh_attachment_combo(key)

    def refresh_ui(self):
        self._refresh_tool_controls()

        self._set_combo_items(
            self.dispenser_tip_combobox,
            self.pm_robot_config.dispenser_1k.get_available_dispenser_tips(),
            self.pm_robot_config.dispenser_1k.get_current_dispenser_tip(),
        )
        self._set_combo_items(
            self.dispenser_2k_tip_combobox,
            self.pm_robot_config.dispenser_2k.get_available_dispenser_tips(),
            self.pm_robot_config.dispenser_2k.get_current_dispenser_tip(),
        )
        self._set_combo_items(
            self.confocal_sensor_head_combobox,
            self.pm_robot_config.confocal_sensor_head.get_available_configs(),
            self.pm_robot_config.confocal_sensor_head.get_current_config(),
        )

        station_controls = (
            (self.pm_robot_config.gonio_left,
             self.box_activate_gonio_left, self.gonio_left_combobox),
            (self.pm_robot_config.gonio_right,
             self.box_activate_gonio_right, self.gonio_right_combobox),
            (self.pm_robot_config.smarpod_station,
             self.box_activate_smarpod, self.smarpod_combobox),
        )
        for station, checkbox, combo in station_controls:
            self._set_checked(checkbox, station.get_activate_status())
            self._set_combo_items(
                combo,
                station.get_available_chucks(),
                station.get_current_chuck(),
            )

        smarpod = self.pm_robot_config.smarpod_station
        self._set_combo_items(
            self.smarpod_combobox_chuck_center,
            smarpod.get_available_chucks_center(),
            smarpod.get_current_chuck_center(),
        )

        measuring = self.pm_robot_config._config_data['measuring_systems']
        self._set_checked(
            self.use_confocal_head_checkbox,
            measuring.get('use_confocal_head', False),
        )
        self._set_checked(
            self.use_confocal_bottom_checkbox,
            measuring.get('use_confocal_bottom', False),
        )
        self._set_checked(
            self.use_keyence_bottom_checkbox,
            measuring.get('use_keyence_bottom', False),
        )

        self.config_path_label.setText(
            'File: ' + self.pm_robot_config.get_active_bringup_config_path()
        )

    def save_config(self):
        try:
            self.pm_robot_config.save_config()
            QMessageBox.information(
                self,
                'Configuration saved',
                'The active bringup configuration was saved successfully.',
            )
        except OSError as error:
            QMessageBox.critical(self, 'Could not save configuration', str(error))

    def reload_config(self):
        try:
            self.pm_robot_config.reload_config()
            self.refresh_ui()
        except Exception as error:
            QMessageBox.critical(
                self,
                'Could not reload configuration',
                str(error),
            )

    def create_config_file(self):
        dialog = ConfigurationRequirementDialog(
            self.pm_robot_config.get_config_data(),
            self,
        )
        dialog.exec()

    def load_config_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            'Load PM robot configuration',
            str(Path.cwd()),
            'YAML files (*.yaml *.yml)',
        )
        if not file_path:
            return

        try:
            with Path(file_path).open('r', encoding='utf-8') as input_file:
                loaded_config = yaml.safe_load(input_file)
            if not isinstance(loaded_config, dict) or not loaded_config:
                raise ValueError(
                    'The loaded configuration must be a non-empty YAML '
                    'mapping.'
                )
            self.pm_robot_config.apply_config_data(loaded_config)
            self.refresh_ui()
        except (OSError, ValueError, yaml.YAMLError) as error:
            QMessageBox.critical(
                self,
                'Could not load configuration',
                str(error),
            )
            return

        QMessageBox.information(
            self,
            'Configuration loaded',
            'The supplied properties were applied to the currently selected '
            'configuration. Review them in the GUI and click "Save bringup '
            'config" to write the changes.',
        )

    def set_sim_mode(self):
        self._switch_mode(False, 'SIMULATION')

    def set_real_mode(self):
        self._switch_mode(True, 'REAL HARDWARE')

    def _switch_mode(self, use_real_config, mode_text):
        try:
            self.pm_robot_config.set_real_HW(use_real_config)
            self.refresh_ui()
            self._update_mode_display(use_real_config, mode_text)
        except Exception as error:
            self.node.get_logger().warning(
                f'Failed to switch configuration mode: {error}'
            )

    def auto_detect_mode(self):
        try:
            mode = self._pm_robot_utils.get_mode()
            use_real_config = mode == self._pm_robot_utils.REAL_MODE
            if mode == self._pm_robot_utils.REAL_MODE:
                mode_text = 'REAL HARDWARE (AUTO)'
            elif mode == self._pm_robot_utils.UNITY_MODE:
                mode_text = 'UNITY / SIMULATION (AUTO)'
            elif mode == self._pm_robot_utils.GAZEBO_MODE:
                mode_text = 'GAZEBO / SIMULATION (AUTO)'
            else:
                mode_text = 'SIMULATION (AUTO FALLBACK)'

            self.pm_robot_config.set_real_HW(use_real_config)
            self.refresh_ui()
            self._update_mode_display(use_real_config, mode_text)
        except Exception as error:
            self.config_mode_label.setText('Configuration mode: ERROR')
            self.node.get_logger().warning(
                f'Failed to auto-detect configuration mode: {error}'
            )

    def _update_mode_display(self, use_real_config, mode_text):
        active_style = (
            'background-color: #2e7d32; color: white; font-weight: bold;'
        )
        inactive_style = ''
        self.real_hw_button.setStyleSheet(
            active_style if use_real_config else inactive_style
        )
        self.sim_hw_button.setStyleSheet(
            inactive_style if use_real_config else active_style
        )
        self.config_mode_label.setText(f'Configuration mode: {mode_text}')


class DummyMain(QMainWindow):
    """Small standalone shell retained for manual widget development."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('PM Robot Configurator')


def main():
    app = QApplication(sys.argv)
    window = DummyMain()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
