import sys
from PyQt6.QtWidgets import QApplication, QSpacerItem,QSizePolicy, QMainWindow, QGridLayout,QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QFormLayout, QHBoxLayout, QCheckBox, QComboBox
import yaml
from PyQt6.QtCore import Qt
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from functools import partial
from typing import Union
from rclpy.node import Node
from pm_robot_modules.submodules.pm_robot_config import (VacuumGripperConfig, 
                                                         ParallelGripperConfig, 
                                                         DispenserTipConfig, 
                                                         GonioConfig, 
                                                         PmRobotConfig)

from pm_skills.py_modules.PmRobotUtils import PmRobotUtils

#class ConfigApp():
class PmRobotConfigWidget(QWidget):
    MIN_COMB_WIDTH = 300
    
    def __init__(self, ros_node:Node ):
        super().__init__()
        self.node = ros_node
        # Reuse the PmRobotUtils already attached to the node (avoids a second instance)
        if hasattr(ros_node, 'pm_robot_utils'):
            self._pm_robot_utils = ros_node.pm_robot_utils
        else:
            self._pm_robot_utils = PmRobotUtils(ros_node)
        self.pm_robot_config = self._pm_robot_utils.pm_robot_config
        self.init_ui()
        self.auto_detect_mode()
        

    def init_ui(self):
        self.central_widget = QWidget(self)
        main_layout = QGridLayout()
        #main_layout = QVBoxLayout()

        # Create mode selection buttons at the top
        mode_button_layout = QHBoxLayout()
        
        self.sim_hw_button = QPushButton("Sim HW")
        self.real_hw_button = QPushButton("Real HW")
        self.auto_detect_button = QPushButton("Auto Detect")
        
        self.sim_hw_button.clicked.connect(self.set_sim_mode)
        self.real_hw_button.clicked.connect(self.set_real_mode)
        self.auto_detect_button.clicked.connect(self.auto_detect_mode)
        
        mode_button_layout.addWidget(self.sim_hw_button)
        mode_button_layout.addWidget(self.real_hw_button)
        mode_button_layout.addWidget(self.auto_detect_button)
        mode_button_layout.addStretch()
        
        main_layout.addLayout(mode_button_layout, 0, 0, 1, 6)
        
        # Create configuration mode label
        self.config_mode_label = QLabel()
        self.config_mode_label.setStyleSheet("font-weight: bold; font-size: 12pt; color: #0055AA;")
        main_layout.addWidget(self.config_mode_label, 1, 0, 1, 6)
        
        # Update row indices for the rest of the widgets
        row_offset = 2

        # vacuum gripper checkbox
        self.box_activate_vacuum = QCheckBox()
        self.box_activate_vacuum.setChecked(self.pm_robot_config.tool._gripper_vacuum.get_activate_status())
        self.box_activate_vacuum.clicked.connect(partial(self.clb_gripper_checkbox_change, self.box_activate_vacuum, self.pm_robot_config.tool._gripper_vacuum))

        # parallel gripper 1 jaw checkbox
        self.box_activate_gripper_1_jaw = QCheckBox()
        self.box_activate_gripper_1_jaw.setChecked(self.pm_robot_config.tool._gripper_1_jaw.get_activate_status())
        self.box_activate_gripper_1_jaw.clicked.connect(partial(self.clb_gripper_checkbox_change, self.box_activate_gripper_1_jaw, self.pm_robot_config.tool._gripper_1_jaw))

        # parallel gripper 2 jaw checkbox
        self.box_activate_gripper_2_jaw = QCheckBox()
        self.box_activate_gripper_2_jaw.setChecked(self.pm_robot_config.tool._gripper_2_jaw.get_activate_status())
        self.box_activate_gripper_2_jaw.clicked.connect(partial(self.clb_gripper_checkbox_change, self.box_activate_gripper_2_jaw, self.pm_robot_config.tool._gripper_2_jaw))

        # gonio left checkbox
        self.box_activate_gonio_left = QCheckBox()
        self.box_activate_gonio_left.setChecked(self.pm_robot_config.gonio_left.get_activate_status())
        self.box_activate_gonio_left.clicked.connect(partial(self.clb_gonio_checkbox_change, self.box_activate_gonio_left, self.pm_robot_config.gonio_left))

        # gonio right checkbox  
        self.box_activate_gonio_right = QCheckBox()
        self.box_activate_gonio_right.setChecked(self.pm_robot_config.gonio_right.get_activate_status())
        self.box_activate_gonio_right.clicked.connect(partial(self.clb_gonio_checkbox_change, self.box_activate_gonio_right, self.pm_robot_config.gonio_right))

        # smarpod checkbox
        self.box_activate_smarpod = QCheckBox()
        self.box_activate_smarpod.setChecked(self.pm_robot_config.smarpod_station.get_activate_status())
        self.box_activate_smarpod.clicked.connect(partial(self.clb_gonio_checkbox_change, self.box_activate_smarpod, self.pm_robot_config.smarpod_station))
        
        # vacuum gripper combobox
        self.vacuum_tool_combobox = QComboBox()
        self.vacuum_tool_combobox.addItems([t[0] for t in self.pm_robot_config.tool._gripper_vacuum.get_available_tools()])
        self.vacuum_tool_combobox.setCurrentText(self.pm_robot_config.tool._gripper_vacuum.get_current_tool())
        self.vacuum_tool_combobox.setFixedWidth(self.MIN_COMB_WIDTH)

        self.vacuum_tip_combobox = QComboBox()
        self.vacuum_tip_combobox.addItems(self.pm_robot_config.tool._gripper_vacuum.get_current_extension_list())
        self.vacuum_tip_combobox.setCurrentText(self.pm_robot_config.tool._gripper_vacuum.get_current_tool_attachment())
        self.vacuum_tip_combobox.setFixedWidth(self.MIN_COMB_WIDTH)
        
        self.vacuum_tool_combobox.currentTextChanged.connect(partial(self.clb_tool_gripper_combobox_change, self.vacuum_tool_combobox, self.vacuum_tip_combobox, self.pm_robot_config.tool._gripper_vacuum))
        self.vacuum_tip_combobox.currentTextChanged.connect(partial(self.clb_tip_gripper_combobox_change, self.vacuum_tip_combobox, self.pm_robot_config.tool._gripper_vacuum))
        
        # parallel gripper 1 jaw combobox
        self.gripper_1_jaw_combobox = QComboBox()
        self.gripper_1_jaw_combobox.addItems([t[0] for t in self.pm_robot_config.tool._gripper_1_jaw.get_available_tools()])
        self.gripper_1_jaw_combobox.setCurrentText(self.pm_robot_config.tool._gripper_1_jaw.get_current_tool())
        self.gripper_1_jaw_combobox.setFixedWidth(self.MIN_COMB_WIDTH)
        
        self.gripper_1_jaw_tip_combobox = QComboBox()
        self.gripper_1_jaw_tip_combobox.addItems(self.pm_robot_config.tool._gripper_1_jaw.get_current_extension_list())
        self.gripper_1_jaw_tip_combobox.setCurrentText(self.pm_robot_config.tool._gripper_1_jaw.get_current_tool_attachment())
        self.gripper_1_jaw_tip_combobox.setFixedWidth(self.MIN_COMB_WIDTH)
        
        self.gripper_1_jaw_combobox.currentTextChanged.connect(partial(self.clb_tool_gripper_combobox_change, 
                                                                       self.gripper_1_jaw_combobox, 
                                                                       self.gripper_1_jaw_tip_combobox, 
                                                                       self.pm_robot_config.tool._gripper_1_jaw))
        
        self.gripper_1_jaw_tip_combobox.currentTextChanged.connect(partial(self.clb_tip_gripper_combobox_change, 
                                                                           self.gripper_1_jaw_tip_combobox, 
                                                                           self.pm_robot_config.tool._gripper_1_jaw))

        # parallel gripper 2 jaw combobox
        self.gripper_2_jaw_combobox = QComboBox()
        self.gripper_2_jaw_combobox.addItems([t[0] for t in self.pm_robot_config.tool._gripper_2_jaw.get_available_tools()])
        self.gripper_2_jaw_combobox.setCurrentText(self.pm_robot_config.tool._gripper_2_jaw.get_current_tool())
        self.gripper_2_jaw_combobox.setFixedWidth(self.MIN_COMB_WIDTH)
        
        self.gripper_2_jaw_tip_combobox = QComboBox()
        self.gripper_2_jaw_tip_combobox.addItems(self.pm_robot_config.tool._gripper_2_jaw.get_current_extension_list())
        self.gripper_2_jaw_tip_combobox.setCurrentText(self.pm_robot_config.tool._gripper_2_jaw.get_current_tool_attachment())
        self.gripper_2_jaw_tip_combobox.setFixedWidth(self.MIN_COMB_WIDTH)
        
        self.gripper_2_jaw_combobox.currentTextChanged.connect(partial(self.clb_tool_gripper_combobox_change, 
                                                                       self.gripper_2_jaw_combobox, 
                                                                       self.gripper_2_jaw_tip_combobox, 
                                                                       self.pm_robot_config.tool._gripper_2_jaw))
        
        self.gripper_2_jaw_tip_combobox.currentTextChanged.connect(partial(self.clb_tip_gripper_combobox_change, 
                                                                           self.gripper_2_jaw_tip_combobox, 
                                                                           self.pm_robot_config.tool._gripper_2_jaw))

        # dispenser tip combobox
        self.dispenser_tip_combobox = QComboBox()
        self.dispenser_tip_combobox.addItems(self.pm_robot_config.dispenser_1k.get_available_dispenser_tips())
        self.dispenser_tip_combobox.setCurrentText(self.pm_robot_config.dispenser_1k.get_current_dispenser_tip())
        self.dispenser_tip_combobox.currentTextChanged.connect(partial(self.clb_set_dispenser_tip, self.dispenser_tip_combobox))
        self.dispenser_tip_combobox.setFixedWidth(self.MIN_COMB_WIDTH)
        
        # gonio left combobox
        self.gonio_left_combobox = QComboBox()
        self.gonio_left_combobox.addItems(self.pm_robot_config.gonio_left.get_available_chucks())
        self.gonio_left_combobox.setCurrentText(self.pm_robot_config.gonio_left.get_current_chuck())
        self.gonio_left_combobox.currentTextChanged.connect(partial(self.clb_gonio_combobox_change, 
                                                                    self.gonio_left_combobox, 
                                                                    self.pm_robot_config.gonio_left))
        self.gonio_left_combobox.setFixedWidth(self.MIN_COMB_WIDTH)

        # gonio right combobox
        self.gonio_right_combobox = QComboBox()
        self.gonio_right_combobox.addItems(self.pm_robot_config.gonio_right.get_available_chucks())
        self.gonio_right_combobox.setCurrentText(self.pm_robot_config.gonio_right.get_current_chuck())
        self.gonio_right_combobox.currentTextChanged.connect(partial(self.clb_gonio_combobox_change, 
                                                                     self.gonio_right_combobox, 
                                                                     self.pm_robot_config.gonio_right))
        self.gonio_right_combobox.setFixedWidth(self.MIN_COMB_WIDTH)

        # smarpod combobox
        self.smarpod_combobox = QComboBox()
        self.smarpod_combobox.addItems(self.pm_robot_config.smarpod_station.get_available_chucks())
        self.smarpod_combobox.setCurrentText(self.pm_robot_config.smarpod_station.get_current_chuck())
        self.smarpod_combobox.currentTextChanged.connect(partial(self.clb_gonio_combobox_change, 
                                                                 self.smarpod_combobox, 
                                                                 self.pm_robot_config.smarpod_station))
        self.smarpod_combobox.setFixedWidth(self.MIN_COMB_WIDTH)

        self.smarpod_combobox_chuck_center = QComboBox()
        self.smarpod_combobox_chuck_center.addItems(self.pm_robot_config.smarpod_station.get_available_chucks_center())
        self.smarpod_combobox_chuck_center.setCurrentText(self.pm_robot_config.smarpod_station.get_current_chuck_center())
        self.smarpod_combobox_chuck_center.currentTextChanged.connect(partial(self.clb_gonio_combobox_change_center,
                                                                              self.smarpod_combobox_chuck_center,
                                                                                self.pm_robot_config.smarpod_station))
        
        CHECKBOX_STYLE = "QCheckBox::indicator { width: 20px; height: 20px; }"

        self.box_activate_vacuum.setStyleSheet(CHECKBOX_STYLE)
        self.box_activate_gripper_1_jaw.setStyleSheet(CHECKBOX_STYLE)
        self.box_activate_gripper_2_jaw.setStyleSheet(CHECKBOX_STYLE)
        self.box_activate_gonio_left.setStyleSheet(CHECKBOX_STYLE)
        self.box_activate_gonio_right.setStyleSheet(CHECKBOX_STYLE)
        self.box_activate_smarpod.setStyleSheet(CHECKBOX_STYLE)

        # Add everything to the main layout
        
        # Header style: bold and larger font size
        header_style = "font-weight: bold; font-size: 14pt;"
        
        # Regular label style (for tools and attachments)
        regular_style = "font-size: 10pt;"

        # --- Section: Vacuum Gripper ---

        vacuum_gripper_label = QLabel("<b>Vacuum Gripper Settings</b>")
        vacuum_gripper_label.setStyleSheet(header_style)  # Apply header style
        main_layout.addWidget(vacuum_gripper_label, 2, 0, 1, 4)  # Header with bold and bigger font

        activate_vacuum_label = QLabel("<b>Activation:</b>")
        activate_vacuum_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(activate_vacuum_label, 3, 0)  # Regular label
        main_layout.addWidget(self.box_activate_vacuum, 3, 1)  # Checkbox for activation

        
        tools_label = QLabel("<b>Tools:</b>")
        tools_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(tools_label, 3, 2)  # Regular label
        main_layout.addWidget(self.vacuum_tool_combobox, 3, 3)
        
        attachments_label = QLabel("<b>Attachments:</b>")
        attachments_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(attachments_label, 3, 4)  # Regular label
        main_layout.addWidget(self.vacuum_tip_combobox, 3, 5)


        # --- Section: Gripper 1 Jaw ---
        gripper1_label = QLabel("<b>Gripper 1 Jaw Settings</b>")
        gripper1_label.setStyleSheet(header_style)  # Apply header style
        main_layout.addWidget(gripper1_label, 4, 0, 1, 4)  # Header with bold and bigger font
        
        activate_gripper1_label = QLabel("<b>Activation:</b>")
        activate_gripper1_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(activate_gripper1_label, 5, 0)  # Regular label
        main_layout.addWidget(self.box_activate_gripper_1_jaw, 5, 1)  # Checkbox for activation
        
        
        tools_label = QLabel("<b>Tools:</b>")
        tools_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(tools_label, 5, 2)  # Regular label
        main_layout.addWidget(self.gripper_1_jaw_combobox, 5, 3)
        
        attachments_label = QLabel("<b>Attachments:</b>")
        attachments_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(attachments_label, 5, 4)  # Regular label
        main_layout.addWidget(self.gripper_1_jaw_tip_combobox, 5, 5)

        # --- Section: Gripper 2 Jaw ---
        gripper2_label = QLabel("<b>Gripper 2 Jaw Settings</b>")
        gripper2_label.setStyleSheet(header_style)  # Apply header style
        main_layout.addWidget(gripper2_label, 6, 0, 1, 4)  # Header with bold and bigger font
        
        activate_gripper2_label = QLabel("<b>Activation:</b>")
        activate_gripper2_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(activate_gripper2_label, 7, 0)  # Regular label
        main_layout.addWidget(self.box_activate_gripper_2_jaw, 7, 1)  # Checkbox for activation
        
        tools_label = QLabel("<b>Tools:</b>")
        tools_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(tools_label, 7, 2)  # Regular label
        main_layout.addWidget(self.gripper_2_jaw_combobox, 7, 3)
        
        attachments_label = QLabel("<b>Attachments:</b>")
        attachments_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(attachments_label, 7, 4)  # Regular label
        main_layout.addWidget(self.gripper_2_jaw_tip_combobox, 7, 5)

        # --- Section: Dispenser Tip ---
        dispenser_label = QLabel("<b>Dispenser Tip Settings</b>")
        dispenser_label.setStyleSheet(header_style)  # Apply header style
        main_layout.addWidget(dispenser_label, 8, 0, 1, 4)  # Header with bold and bigger font
        
        attachments_label = QLabel("<b>Attachments:</b>")
        attachments_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(attachments_label, 9, 4)  # Regular label
        main_layout.addWidget(self.dispenser_tip_combobox, 9, 5)

        # --- Section: Gonio Left ---
        gonio_left_label = QLabel("<b>Gonio Left Settings</b>")
        gonio_left_label.setStyleSheet(header_style)  # Apply header style
        activate_gonio_left_label = QLabel("<b>Activation:</b>")
        activate_gonio_left_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(gonio_left_label, 10, 0, 1, 4)  # Header with bold and bigger font
        main_layout.addWidget(activate_gonio_left_label, 11, 0)  # Regular label
        main_layout.addWidget(self.box_activate_gonio_left, 11, 1)  # Checkbox for activation        
        chucks_label = QLabel("<b>Chucks:</b>")
        chucks_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(chucks_label, 11, 4)  # Regular label
        main_layout.addWidget(self.gonio_left_combobox, 11, 5)

        # --- Section: Gonio Right ---
        gonio_right_label = QLabel("<b>Gonio Right Settings</b>")
        gonio_right_label.setStyleSheet(header_style)  # Apply header style
        activate_gonio_right_label = QLabel("<b>Activation:</b>")
        activate_gonio_right_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(gonio_right_label, 12, 0, 1, 4)  # Header with bold and bigger font
        main_layout.addWidget(activate_gonio_right_label, 13, 0)  # Regular label
        main_layout.addWidget(self.box_activate_gonio_right, 13, 1)  # Checkbox for activation
        chucks_label = QLabel("<b>Chucks:</b>")
        chucks_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(chucks_label, 13, 4)  # Regular label
        main_layout.addWidget(self.gonio_right_combobox, 13, 5)

        # --- Section: Smarpod ---
        smarpod_label = QLabel("<b>Smarpod Settings</b>")
        smarpod_label.setStyleSheet(header_style)  # Apply header style
        activate_smarpod_label = QLabel("<b>Activation:</b>")
        activate_smarpod_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(smarpod_label, 14, 0, 1, 4)  # Header with bold and bigger font
        main_layout.addWidget(activate_smarpod_label, 15, 0)  # Regular label
        main_layout.addWidget(self.box_activate_smarpod, 15, 1)  # Checkbox for activation
        chucks_label = QLabel("<b>Chucks:</b>")
        chucks_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(chucks_label, 15, 4)  # Regular label
        main_layout.addWidget(self.smarpod_combobox, 15, 5)
        
        chuck_center_label = QLabel("<b>Chuck Center:</b>")
        chuck_center_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(chuck_center_label, 16, 4)  # Regular label
        main_layout.addWidget(self.smarpod_combobox_chuck_center, 16, 5)

        # Spacer and Save Button
        main_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        save_button = QPushButton("Save Config")
        save_button.setStyleSheet("padding: 8px 16px; font-weight: bold;")
        save_button.clicked.connect(self.save_config)
        main_layout.addWidget(save_button, 17, 0, 1, 6)
        reload_button = QPushButton("Reload Config")
        reload_button.setStyleSheet("padding: 8px 16px; font-weight: bold;")
        reload_button.clicked.connect(self.reload_config)
        main_layout.addWidget(reload_button, 18, 0, 1, 6)

        # Apply layout
        self.central_widget.setLayout(main_layout)
        self.setWindowTitle('PM Robot Configurator')
        self.setGeometry(100, 100, 1000, 600)

    def clb_gripper_checkbox_change(self, box_widget:QCheckBox, obj:Union[VacuumGripperConfig, ParallelGripperConfig]):
        if not box_widget.isChecked():
            box_widget.setChecked(True)
            return
        
        self.deactivate_grippers()
        obj.activate()
        self.refresh_gripper_checkboxes()

    def deactivate_grippers(self):
        self.pm_robot_config.tool._gripper_vacuum.deactivate()
        self.pm_robot_config.tool._gripper_1_jaw.deactivate()
        self.pm_robot_config.tool._gripper_2_jaw.deactivate()

    def clb_gonio_checkbox_change(self, box_widget:QCheckBox, gonio:GonioConfig):
        if not box_widget.isChecked():
            gonio.deactivate()
        else:
            gonio.activate()
        

    def clb_tool_gripper_combobox_change(self, combobox:QComboBox, combobox_tip: QComboBox ,obj:Union[VacuumGripperConfig, ParallelGripperConfig]):
        obj.set_current_tool(tool=combobox.currentText())
        obj.set_current_tool(ext=obj.get_first_extension_for_current_tool())
        combobox_tip.clear()
        combobox_tip.addItems(obj.get_current_extension_list())
        combobox_tip.setCurrentText(obj.get_first_extension_for_current_tool())

    def clb_tip_gripper_combobox_change(self, combobox_tip: QComboBox ,obj:Union[VacuumGripperConfig, ParallelGripperConfig]):
        obj.set_current_tool(ext=combobox_tip.currentText())

    def clb_gonio_combobox_change(self, combobox:QComboBox, gonio:GonioConfig):
        gonio.set_current_chuck(combobox.currentText())

    def clb_gonio_combobox_change_center(self, combobox:QComboBox, gonio:GonioConfig):
        gonio.set_current_chuck_center(combobox.currentText())

    def clb_set_dispenser_tip(self, combobox: QComboBox):
        self.pm_robot_config.dispenser_1k.set_currrent_dispenser_tip(combobox.currentText())
    
    def refresh_gripper_checkboxes(self):
        self.box_activate_vacuum.setChecked(self.pm_robot_config.tool._gripper_vacuum.get_activate_status())
        self.box_activate_gripper_1_jaw.setChecked(self.pm_robot_config.tool._gripper_1_jaw.get_activate_status())
        self.box_activate_gripper_2_jaw.setChecked(self.pm_robot_config.tool._gripper_2_jaw.get_activate_status())

    def save_config(self):
        self.pm_robot_config.save_config()
    
    def set_sim_mode(self):
        """Switch to simulation hardware mode."""
        try:
            # Set the mode to UNITY_MODE (simulation)
            self.pm_robot_config.set_to_sim_HW()
            self.node.get_logger().info("Switched to Simulation Mode")
            self.reload_config()  # Reload config to update UI with simulation settings
            self.update_mode_buttons(set_to_real_hw=False)
            self.set_config_mode_label("SIMULATION")
        except Exception as e:
            self.node.get_logger().warning(f"Failed to switch to Simulation Mode: {e}")
    
    def set_real_mode(self):
        """Switch to real hardware mode."""
        try:
            # Set the mode to REAL_MODE
            self.pm_robot_config.set_to_real_HW()
            self.node.get_logger().info("Switched to Real Hardware Mode")
            self.reload_config()  # Reload config to update UI with real hardware settings
            self.update_mode_buttons(set_to_real_hw=True)
            self.set_config_mode_label("REAL_HARDWARE")
        except Exception as e:
            self.node.get_logger().warning(f"Failed to switch to Real Hardware Mode: {e}")
    
    def auto_detect_mode(self):
        """Auto-detect current mode and update the configuration."""
        try:
            current_mode = self._pm_robot_utils.get_mode()

            if current_mode == self._pm_robot_utils.UNITY_MODE:
                mode_text = "UNITY_MODE"
                self.set_sim_mode()
                self.node.get_logger().info("Auto-detect mode: Unity detected, switched to Simulation Mode in config.")
            elif current_mode == self._pm_robot_utils.REAL_MODE:
                mode_text = "REAL_HARDWARE"
                self.set_real_mode()
                self.node.get_logger().info("Auto-detect mode: Real Hardware detected, switched to Real Hardware Mode in config.")
            elif current_mode == self._pm_robot_utils.GAZEBO_MODE:
                mode_text = "GAZEBO_MODE"
                self.set_sim_mode()  # Assuming Gazebo should be treated as simulation for config purposes
                self.node.get_logger().info("Auto-detect mode: Gazebo detected, treating as Simulation Mode in config.")
            else:
                mode_text = "UNKNOWN"
                self.set_sim_mode()  # Default to simulation if unknown
                self.node.get_logger().warning(f"Auto-detect mode returned unknown mode '{current_mode}', defaulting to Simulation Mode.")
        
        except Exception as e:
            self.config_mode_label.setText(f"Configuration Mode: ERROR")
            self.node.get_logger().warning(f"Failed to update configuration mode label: {e}")

    def update_mode_buttons(self, set_to_real_hw:bool=False):
        """Update the visual state of mode buttons to show which one is active."""
        
        # Define button styles
        active_style = "background-color: #4CAF50; color: white; font-weight: bold; padding: 8px 12px; border-radius: 3px; border: 2px solid #2E7D32;"
        inactive_style = "background-color: #e0e0e0; color: black; padding: 8px 12px; border-radius: 3px; border: 1px solid #999;"
        
        # Reset all buttons to inactive
        self.sim_hw_button.setStyleSheet(inactive_style)
        self.real_hw_button.setStyleSheet(inactive_style)
        self.auto_detect_button.setStyleSheet(inactive_style)
        
        # Highlight the active button based on current mode
        if set_to_real_hw:
            self.real_hw_button.setStyleSheet(active_style)
        else:
            self.sim_hw_button.setStyleSheet(active_style)

    
    def _disconnect_signals(self):
        """Disconnect all signals from widgets."""
        # Checkbox signals
        self.box_activate_vacuum.clicked.disconnect()
        self.box_activate_gripper_1_jaw.clicked.disconnect()
        self.box_activate_gripper_2_jaw.clicked.disconnect()
        self.box_activate_gonio_left.clicked.disconnect()
        self.box_activate_gonio_right.clicked.disconnect()
        self.box_activate_smarpod.clicked.disconnect()
        
        # Combobox signals
        self.vacuum_tool_combobox.currentTextChanged.disconnect()
        self.vacuum_tip_combobox.currentTextChanged.disconnect()
        self.gripper_1_jaw_combobox.currentTextChanged.disconnect()
        self.gripper_1_jaw_tip_combobox.currentTextChanged.disconnect()
        self.gripper_2_jaw_combobox.currentTextChanged.disconnect()
        self.gripper_2_jaw_tip_combobox.currentTextChanged.disconnect()
        self.dispenser_tip_combobox.currentTextChanged.disconnect()
        self.gonio_left_combobox.currentTextChanged.disconnect()
        self.gonio_right_combobox.currentTextChanged.disconnect()
        self.smarpod_combobox.currentTextChanged.disconnect()
        self.smarpod_combobox_chuck_center.currentTextChanged.disconnect()
    
    def _reconnect_signals(self):
        """Reconnect all signals to the current config objects."""
        # Checkbox signals
        self.box_activate_vacuum.clicked.connect(partial(self.clb_gripper_checkbox_change, self.box_activate_vacuum, self.pm_robot_config.tool._gripper_vacuum))
        self.box_activate_gripper_1_jaw.clicked.connect(partial(self.clb_gripper_checkbox_change, self.box_activate_gripper_1_jaw, self.pm_robot_config.tool._gripper_1_jaw))
        self.box_activate_gripper_2_jaw.clicked.connect(partial(self.clb_gripper_checkbox_change, self.box_activate_gripper_2_jaw, self.pm_robot_config.tool._gripper_2_jaw))
        self.box_activate_gonio_left.clicked.connect(partial(self.clb_gonio_checkbox_change, self.box_activate_gonio_left, self.pm_robot_config.gonio_left))
        self.box_activate_gonio_right.clicked.connect(partial(self.clb_gonio_checkbox_change, self.box_activate_gonio_right, self.pm_robot_config.gonio_right))
        self.box_activate_smarpod.clicked.connect(partial(self.clb_gonio_checkbox_change, self.box_activate_smarpod, self.pm_robot_config.smarpod_station))
        
        # Combobox signals
        self.vacuum_tool_combobox.currentTextChanged.connect(partial(self.clb_tool_gripper_combobox_change, self.vacuum_tool_combobox, self.vacuum_tip_combobox, self.pm_robot_config.tool._gripper_vacuum))
        self.vacuum_tip_combobox.currentTextChanged.connect(partial(self.clb_tip_gripper_combobox_change, self.vacuum_tip_combobox, self.pm_robot_config.tool._gripper_vacuum))
        self.gripper_1_jaw_combobox.currentTextChanged.connect(partial(self.clb_tool_gripper_combobox_change, self.gripper_1_jaw_combobox, self.gripper_1_jaw_tip_combobox, self.pm_robot_config.tool._gripper_1_jaw))
        self.gripper_1_jaw_tip_combobox.currentTextChanged.connect(partial(self.clb_tip_gripper_combobox_change, self.gripper_1_jaw_tip_combobox, self.pm_robot_config.tool._gripper_1_jaw))
        self.gripper_2_jaw_combobox.currentTextChanged.connect(partial(self.clb_tool_gripper_combobox_change, self.gripper_2_jaw_combobox, self.gripper_2_jaw_tip_combobox, self.pm_robot_config.tool._gripper_2_jaw))
        self.gripper_2_jaw_tip_combobox.currentTextChanged.connect(partial(self.clb_tip_gripper_combobox_change, self.gripper_2_jaw_tip_combobox, self.pm_robot_config.tool._gripper_2_jaw))
        self.dispenser_tip_combobox.currentTextChanged.connect(partial(self.clb_set_dispenser_tip, self.dispenser_tip_combobox))
        self.gonio_left_combobox.currentTextChanged.connect(partial(self.clb_gonio_combobox_change, self.gonio_left_combobox, self.pm_robot_config.gonio_left))
        self.gonio_right_combobox.currentTextChanged.connect(partial(self.clb_gonio_combobox_change, self.gonio_right_combobox, self.pm_robot_config.gonio_right))
        self.smarpod_combobox.currentTextChanged.connect(partial(self.clb_gonio_combobox_change, self.smarpod_combobox, self.pm_robot_config.smarpod_station))
        self.smarpod_combobox_chuck_center.currentTextChanged.connect(partial(self.clb_gonio_combobox_change_center, self.smarpod_combobox_chuck_center, self.pm_robot_config.smarpod_station))
        
    def set_config_mode_label(self, mode_text:str):
        self.config_mode_label.setText(f"Configuration Mode: {mode_text}")
        self.config_mode_label.setToolTip(f"Path: {self.pm_robot_config.get_active_bringup_config_path()}")

    def reload_config(self):
        self._disconnect_signals()
        self.pm_robot_config.reload_config()

        # Refresh smarpod comboboxes with available items BEFORE reconnecting signals.
        # Otherwise the auto-select-first-item behavior after addItems() would
        # fire currentTextChanged and overwrite the in-memory use_chuck /
        # use_chuck_center values with the first available item (e.g. "empty"),
        # effectively reverting the saved configuration to defaults.
        self.smarpod_combobox.clear()
        self.smarpod_combobox.addItems(self.pm_robot_config.smarpod_station.get_available_chucks())
        self.smarpod_combobox.setCurrentText(self.pm_robot_config.smarpod_station.get_current_chuck())

        self.smarpod_combobox_chuck_center.clear()
        self.smarpod_combobox_chuck_center.addItems(self.pm_robot_config.smarpod_station.get_available_chucks_center())
        self.smarpod_combobox_chuck_center.setCurrentText(self.pm_robot_config.smarpod_station.get_current_chuck_center())

        self._reconnect_signals()
        self.refresh_gripper_checkboxes()
        self.box_activate_gonio_left.setChecked(self.pm_robot_config.gonio_left.get_activate_status())
        self.box_activate_gonio_right.setChecked(self.pm_robot_config.gonio_right.get_activate_status())
        self.box_activate_smarpod.setChecked(self.pm_robot_config.smarpod_station.get_activate_status())
        self.vacuum_tool_combobox.setCurrentText(self.pm_robot_config.tool._gripper_vacuum.get_current_tool())
        self.vacuum_tip_combobox.setCurrentText(self.pm_robot_config.tool._gripper_vacuum.get_current_tool_attachment())
        self.gripper_1_jaw_combobox.setCurrentText(self.pm_robot_config.tool._gripper_1_jaw.get_current_tool())
        self.gripper_1_jaw_tip_combobox.setCurrentText(self.pm_robot_config.tool._gripper_1_jaw.get_current_tool_attachment())
        self.gripper_2_jaw_combobox.setCurrentText(self.pm_robot_config.tool._gripper_2_jaw.get_current_tool())
        self.gripper_2_jaw_tip_combobox.setCurrentText(self.pm_robot_config.tool._gripper_2_jaw.get_current_tool_attachment())
        self.dispenser_tip_combobox.setCurrentText(self.pm_robot_config.dispenser_1k.get_current_dispenser_tip())
        self.gonio_left_combobox.setCurrentText(self.pm_robot_config.gonio_left.get_current_chuck())
        self.gonio_right_combobox.setCurrentText(self.pm_robot_config.gonio_right.get_current_chuck())

        # Smarpod comboboxes were already refreshed while signals were
        # disconnected above, so we only need to set the current text here
        # to make sure the in-memory and the UI stay in sync without
        # triggering the change handlers.

        #self.node.get_logger().error(f"Text: {self.pm_robot_config.smarpod_station.get_available_chucks_center()}")

class DummyMain(QMainWindow):

    def __init__(self):
        super().__init__()
        self.w = None  # No external window yet.
        self.button = QPushButton("Push for Window")
        self.button.clicked.connect(self.show_new_window)
        self.setCentralWidget(self.button)

    def show_new_window(self, checked):
        if self.w is None:
            self.w = PmRobotConfigWidget()
        self.w.show()


def main():
    app = QApplication(sys.argv)
    ex = DummyMain()
    ex.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
