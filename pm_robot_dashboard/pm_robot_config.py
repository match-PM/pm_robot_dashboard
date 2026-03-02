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
        #self.pm_robot_config = PmRobotConfig()
        self._pm_robot_utills = PmRobotUtils(ros_node)
        self.pm_robot_config = self._pm_robot_utills.pm_robot_config
        self.init_ui()
        

    def init_ui(self):
        self.central_widget = QWidget(self)
        main_layout = QGridLayout()
        #main_layout = QVBoxLayout()

        # Create configuration mode label at the top
        self.config_mode_label = QLabel()
        self.config_mode_label.setStyleSheet("font-weight: bold; font-size: 12pt; color: #0055AA;")
        self.update_config_mode_label()
        main_layout.addWidget(self.config_mode_label, 0, 0, 1, 6)

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
        main_layout.addWidget(vacuum_gripper_label, 1, 0, 1, 4)  # Header with bold and bigger font

        activate_vacuum_label = QLabel("<b>Activation:</b>")
        activate_vacuum_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(activate_vacuum_label, 2, 0)  # Regular label
        main_layout.addWidget(self.box_activate_vacuum, 2, 1)  # Checkbox for activation

        
        tools_label = QLabel("<b>Tools:</b>")
        tools_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(tools_label, 2, 2)  # Regular label
        main_layout.addWidget(self.vacuum_tool_combobox, 2, 3)
        
        attachments_label = QLabel("<b>Attachments:</b>")
        attachments_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(attachments_label, 2, 4)  # Regular label
        main_layout.addWidget(self.vacuum_tip_combobox, 2, 5)


        # --- Section: Gripper 1 Jaw ---
        gripper1_label = QLabel("<b>Gripper 1 Jaw Settings</b>")
        gripper1_label.setStyleSheet(header_style)  # Apply header style
        main_layout.addWidget(gripper1_label, 3, 0, 1, 4)  # Header with bold and bigger font
        
        activate_gripper1_label = QLabel("<b>Activation:</b>")
        activate_gripper1_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(activate_gripper1_label, 4, 0)  # Regular label
        main_layout.addWidget(self.box_activate_gripper_1_jaw, 4, 1)  # Checkbox for activation
        
        
        tools_label = QLabel("<b>Tools:</b>")
        tools_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(tools_label, 4, 2)  # Regular label
        main_layout.addWidget(self.gripper_1_jaw_combobox, 4, 3)
        
        attachments_label = QLabel("<b>Attachments:</b>")
        attachments_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(attachments_label, 4, 4)  # Regular label
        main_layout.addWidget(self.gripper_1_jaw_tip_combobox, 4, 5)

        # --- Section: Gripper 2 Jaw ---
        gripper2_label = QLabel("<b>Gripper 2 Jaw Settings</b>")
        gripper2_label.setStyleSheet(header_style)  # Apply header style
        main_layout.addWidget(gripper2_label, 5, 0, 1, 4)  # Header with bold and bigger font
        
        activate_gripper2_label = QLabel("<b>Activation:</b>")
        activate_gripper2_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(activate_gripper2_label, 6, 0)  # Regular label
        main_layout.addWidget(self.box_activate_gripper_2_jaw, 6, 1)  # Checkbox for activation
        
        tools_label = QLabel("<b>Tools:</b>")
        tools_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(tools_label, 6, 2)  # Regular label
        main_layout.addWidget(self.gripper_2_jaw_combobox, 6, 3)
        
        attachments_label = QLabel("<b>Attachments:</b>")
        attachments_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(attachments_label, 6, 4)  # Regular label
        main_layout.addWidget(self.gripper_2_jaw_tip_combobox, 6, 5)

        # --- Section: Dispenser Tip ---
        dispenser_label = QLabel("<b>Dispenser Tip Settings</b>")
        dispenser_label.setStyleSheet(header_style)  # Apply header style
        main_layout.addWidget(dispenser_label, 7, 0, 1, 4)  # Header with bold and bigger font
        
        attachments_label = QLabel("<b>Attachments:</b>")
        attachments_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(attachments_label, 8, 4)  # Regular label
        main_layout.addWidget(self.dispenser_tip_combobox, 8, 5)

        # --- Section: Gonio Left ---
        gonio_left_label = QLabel("<b>Gonio Left Settings</b>")
        gonio_left_label.setStyleSheet(header_style)  # Apply header style
        activate_gonio_left_label = QLabel("<b>Activation:</b>")
        activate_gonio_left_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(gonio_left_label, 9, 0, 1, 4)  # Header with bold and bigger font
        main_layout.addWidget(activate_gonio_left_label, 10, 0)  # Regular label
        main_layout.addWidget(self.box_activate_gonio_left, 10, 1)  # Checkbox for activation        
        chucks_label = QLabel("<b>Chucks:</b>")
        chucks_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(chucks_label, 10, 4)  # Regular label
        main_layout.addWidget(self.gonio_left_combobox, 10, 5)

        # --- Section: Gonio Right ---
        gonio_right_label = QLabel("<b>Gonio Right Settings</b>")
        gonio_right_label.setStyleSheet(header_style)  # Apply header style
        activate_gonio_right_label = QLabel("<b>Activation:</b>")
        activate_gonio_right_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(gonio_right_label, 11, 0, 1, 4)  # Header with bold and bigger font
        main_layout.addWidget(activate_gonio_right_label, 12, 0)  # Regular label
        main_layout.addWidget(self.box_activate_gonio_right, 12, 1)  # Checkbox for activation
        chucks_label = QLabel("<b>Chucks:</b>")
        chucks_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(chucks_label, 12, 4)  # Regular label
        main_layout.addWidget(self.gonio_right_combobox, 12, 5)

        # --- Section: Smarpod ---
        smarpod_label = QLabel("<b>Smarpod Settings</b>")
        smarpod_label.setStyleSheet(header_style)  # Apply header style
        activate_smarpod_label = QLabel("<b>Activation:</b>")
        activate_smarpod_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(smarpod_label, 13, 0, 1, 4)  # Header with bold and bigger font
        main_layout.addWidget(activate_smarpod_label, 14, 0)  # Regular label
        main_layout.addWidget(self.box_activate_smarpod, 14, 1)  # Checkbox for activation
        chucks_label = QLabel("<b>Chucks:</b>")
        chucks_label.setStyleSheet(regular_style)  # Apply regular label style
        main_layout.addWidget(chucks_label, 14, 4)  # Regular label
        main_layout.addWidget(self.smarpod_combobox, 14, 5)

        # Spacer and Save Button
        main_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        save_button = QPushButton("Save Config")
        save_button.setStyleSheet("padding: 8px 16px; font-weight: bold;")
        save_button.clicked.connect(self.save_config)
        main_layout.addWidget(save_button, 15, 0, 1, 6)
        reload_button = QPushButton("Reload Config")
        reload_button.setStyleSheet("padding: 8px 16px; font-weight: bold;")
        reload_button.clicked.connect(self.reload_config)
        main_layout.addWidget(reload_button, 16, 0, 1, 6)

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

    def clb_set_dispenser_tip(self, combobox: QComboBox):
        self.pm_robot_config.dispenser_1k.set_currrent_dispenser_tip(combobox.currentText())
    
    def refresh_gripper_checkboxes(self):
        self.box_activate_vacuum.setChecked(self.pm_robot_config.tool._gripper_vacuum.get_activate_status())
        self.box_activate_gripper_1_jaw.setChecked(self.pm_robot_config.tool._gripper_1_jaw.get_activate_status())
        self.box_activate_gripper_2_jaw.setChecked(self.pm_robot_config.tool._gripper_2_jaw.get_activate_status())

    def save_config(self):
        self.pm_robot_config.save_config()
    
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
    
    def update_config_mode_label(self):
        try:
            current_mode = self._pm_robot_utills.get_mode()

            if current_mode == self._pm_robot_utills.UNITY_MODE:
                mode_text = "UNITY_MODE"
            if current_mode == self._pm_robot_utills.REAL_MODE:
                mode_text = "REAL_HARDWARE"
            if current_mode == self._pm_robot_utills.GAZEBO_MODE:
                mode_text = "GAZEBO_MODE"
            else:
                mode_text = "UNKNOWN"
            
            self.config_mode_label.setText(f"Configuration Mode: {mode_text}")
            self.config_mode_label.setToolTip(f"Path: {self.pm_robot_config.get_active_bringup_config_path()}")
        except Exception as e:
            self.config_mode_label.setText(f"Configuration Mode: ERROR")
            self.node.get_logger().warning(f"Failed to update configuration mode label: {e}")
        
    def reload_config(self):
        self._disconnect_signals()
        self._pm_robot_utills.update_pm_robot_config()
        self.pm_robot_config.reload_config()
        self._reconnect_signals()
        self.update_config_mode_label()
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
        self.smarpod_combobox.setCurrentText(self.pm_robot_config.smarpod_station.get_current_chuck())

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
