import sys
import math
from collections import defaultdict
from enum import Enum

from PyQt6.QtWidgets import (
    QApplication, QGraphicsView, QGraphicsScene,
    QGraphicsRectItem, QGraphicsTextItem, QDialog,
    QVBoxLayout, QHBoxLayout, QPushButton, QLabel
)
from PyQt6.QtGui import QPen, QBrush, QPainter, QColor, QPolygonF
from PyQt6.QtCore import Qt, QPointF
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
from rclpy.node import Node as ROS2Node


# -------------------------
# Node Groups
# -------------------------
class NodeGroup(Enum):
    """Enum for node groups with associated colors."""
    NONE = ("#f4a261", "No Group")
    GROUP_1 = ("#FF6B6B", "Group 1")
    GROUP_2 = ("#4ECDC4", "Group 2")
    GROUP_3 = ("#45B7D1", "Group 3")
    GROUP_4 = ("#FFA07A", "Group 4")
    GROUP_5 = ("#98D8C8", "Group 5")
    GROUP_6 = ("#F7DC6F", "Group 6")
    GROUP_7 = ("#BB8FCE", "Group 7")
    GROUP_8 = ("#85C1E2", "Group 8")
    GROUP_9 = ("#F8B88B", "Group 9")
    GROUP_10 = ("#AAE3F5", "Group 10")
    
    @property
    def color(self) -> str:
        """Get the hex color for this group."""
        return self.value[0]
    
    @property
    def label(self) -> str:
        """Get the label for this group."""
        return self.value[1]





# -------------------------
# Node Configuration
# -------------------------
class NodeConfig:
    """Configuration class for node properties."""
    
    def __init__(self, name, display_text=None, service_type=None, service_client=None, 
                 dialog_class=None, ros_node: ROS2Node = None, group: NodeGroup = NodeGroup.NONE):
        """
        Initialize a node configuration.
        
        Args:
            name: The name of the node (used internally)
            display_text: Custom text to display on the node (defaults to name)
            service_type: ROS service type as string (e.g., "/xy/camera_top")
            service_client: Service client identifier as string
            dialog_class: Dialog class to instantiate on node click
            ros_node: The ROS2 node instance
            group: NodeGroup enum for this node (defaults to NONE)
        """
        self.name = name
        self.display_text = display_text or name
        self.service_type = service_type
        self.service_client = service_client
        self.dialog_class = dialog_class
        self.ros_node = ros_node
        self.group = group
        self.action: ServiceAction = None

        if ros_node is not None and service_client is not None and service_type is not None:
            self.action = ServiceAction(
                node=self.ros_node,
                client=self.service_client,
                service_type=self.service_type
            )


# -------------------------
# Group Box
# -------------------------
class GroupBox(QGraphicsRectItem):
    def __init__(self, x, y, w, h, title):
        super().__init__(x, y, w, h)

        self.setBrush(QBrush(QColor(240, 240, 240)))
        self.setPen(QPen(Qt.GlobalColor.black, 2))
        self.setZValue(-10)

        self.title = QGraphicsTextItem(title, self)
        self.title.setZValue(20)

        font = self.title.font()
        font.setPointSize(14)
        font.setBold(True)
        self.title.setFont(font)

        text_rect = self.title.boundingRect()
        self.title.setPos(
            x + (w - text_rect.width()) / 2,
            y - 30
        )


# -------------------------
# Calibration Node
# -------------------------
class CalibrationNode(QGraphicsRectItem):
    """Graphical representation of a calibration node."""
    WIDTH = 140
    HEIGHT = 50

    def __init__(self, x, y, config: NodeConfig):
        """
        Initialize a calibration node.
        
        Args:
            x: X position
            y: Y position
            config: NodeConfig instance with node configuration
        """
        super().__init__(0, 0, self.WIDTH, self.HEIGHT)
        self.setPos(x, y)
        self.config = config

        # Set background color based on group
        self.setBrush(QBrush(QColor(config.group.color)))
        self.default_pen = QPen(Qt.GlobalColor.black, 2)
        self.setPen(self.default_pen)
        self.setZValue(10)

        self.label = QGraphicsTextItem(self.config.display_text, self)
        self._center_text()
        
        # Only accept mouse events if node is clickable
        self.setAcceptedMouseButtons(self._is_clickable() and Qt.MouseButton.LeftButton or Qt.MouseButton.NoButton)

    def _is_clickable(self) -> bool:
        """Check if the node is clickable (has dialog or service configured)."""
        has_service = self.config.service_type and self.config.service_client
        has_dialog = self.config.dialog_class is not None
        return has_service or has_dialog

    def _center_text(self):
        """Center the text within the node."""
        rect = self.rect()
        text_rect = self.label.boundingRect()
        self.label.setPos(
            (rect.width() - text_rect.width()) / 2,
            (rect.height() - text_rect.height()) / 2
        )

    def get_bottom_connection_point(self, index=0, total=1):
        """Get connection point at bottom of node, distributed across width if multiple."""
        r = self.sceneBoundingRect()
        if total == 1:
            x = r.center().x()
        else:
            x = r.left() + (r.width() / (total + 1)) * (index + 1)
        return QPointF(x, r.bottom())

    def get_top_connection_point(self, index=0, total=1):
        """Get connection point at top of node, distributed across width if multiple."""
        r = self.sceneBoundingRect()
        if total == 1:
            x = r.center().x()
        else:
            x = r.left() + (r.width() / (total + 1)) * (index + 1)
        return QPointF(x, r.top())

    def set_success_color(self):
        """Set the node border color to green (success)."""
        success_pen = QPen(Qt.GlobalColor.green, 2)
        self.setPen(success_pen)

    def set_failure_color(self):
        """Set the node border color to red (failure)."""
        failure_pen = QPen(Qt.GlobalColor.red, 2)
        self.setPen(failure_pen)

    def reset_color(self):
        """Reset the node border color to default (black)."""
        self.setPen(self.default_pen)

    def execute_node_action(self)->bool:
        """Execute the node's action (service call)."""
        if self.config.action:
            success = self.config.action.execute()
        else:
            success = False
        
        # Update border color based on success
        if success:
            self.set_success_color()
        else:
            self.set_failure_color()
        
        return success

    def mousePressEvent(self, event):
        """Handle mouse press on the node."""
        if not self._is_clickable():
            super().mousePressEvent(event)
            return
        
        print(f"[CLICK] {self.config.name}")

        if self.config.dialog_class:
            dialog = self.config.dialog_class(self)
            dialog.exec()
        else:
            self.execute_node_action()

        super().mousePressEvent(event)



# -------------------------
# Confirmation Dialog
# -------------------------
class ExampleDialog(QDialog):
    def __init__(self, node_item: CalibrationNode = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Confirm Calibration")
        self.node_item = node_item
        self.setMinimumWidth(550)
        self.setMaximumWidth(700)
        
        main_layout = QVBoxLayout()
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        if node_item and node_item.config.service_type:
            # Title/Node name
            title_label = QLabel(f"Execute Calibration for: <b>{node_item.config.name}</b>")
            title_label.setStyleSheet("font-size: 12pt; font-weight: bold;")
            main_layout.addWidget(title_label)
            
            # Service details
            details_layout = QVBoxLayout()
            details_layout.setSpacing(8)
            details_layout.setContentsMargins(10, 10, 10, 10)
            
            service_label = QLabel("<b>Service Client:</b>")
            service_value = QLabel(node_item.config.service_client or "N/A")
            service_value.setWordWrap(True)
            service_value.setStyleSheet("color: #666; padding-left: 10px;")
            
            type_label = QLabel("<b>Service Type:</b>")
            type_value = QLabel(node_item.config.service_type or "N/A")
            type_value.setWordWrap(True)
            type_value.setStyleSheet("color: #666; padding-left: 10px;")
            
            details_layout.addWidget(service_label)
            details_layout.addWidget(service_value)
            details_layout.addWidget(type_label)
            details_layout.addWidget(type_value)
            
            details_widget_layout = QVBoxLayout()
            details_widget_layout.addLayout(details_layout)
            main_layout.addLayout(details_widget_layout)
            
            # Confirmation question
            question_label = QLabel("Do you want to proceed with this action?")
            question_label.setStyleSheet("font-style: italic; color: #333;")
            main_layout.addWidget(question_label)
            
            # Buttons
            button_layout = QHBoxLayout()
            button_layout.setSpacing(10)
            
            yes_button = QPushButton("Yes, Execute")
            yes_button.setMinimumWidth(120)
            yes_button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 8px;")
            yes_button.clicked.connect(self.execute_and_accept)
            
            no_button = QPushButton("Cancel")
            no_button.setMinimumWidth(120)
            no_button.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 8px;")
            no_button.clicked.connect(self.reject)
            
            button_layout.addStretch()
            button_layout.addWidget(yes_button)
            button_layout.addWidget(no_button)
            button_layout.addStretch()
            
            main_layout.addLayout(button_layout)
        else:
            label = QLabel("Action Confirmation")
            label.setStyleSheet("font-size: 12pt; font-weight: bold;")
            main_layout.addWidget(label)
            
            ok_button = QPushButton("OK")
            ok_button.setMinimumWidth(80)
            ok_button.clicked.connect(self.accept)
            
            button_layout = QHBoxLayout()
            button_layout.addStretch()
            button_layout.addWidget(ok_button)
            button_layout.addStretch()
            main_layout.addLayout(button_layout)
        
        main_layout.addStretch()
        self.setLayout(main_layout)
    
    def execute_and_accept(self):
        """Execute the node action and accept the dialog."""
        if self.node_item:
            self.node_item.execute_node_action()
        self.accept()

# -------------------------
# Arrow
# -------------------------
class Arrow(QGraphicsRectItem):
    BASE_SPACING = 20
    PADDING = 30

    def __init__(self, start, end, grid_level, total_grid_levels, label, all_nodes=None, start_index=0, start_total=1, end_index=0, end_total=1, grid_top_y=130, grid_bottom_y=250):
        super().__init__()
        self.start = start
        self.end = end
        self.grid_level = grid_level
        self.total_grid_levels = total_grid_levels
        self.label = label
        self.all_nodes = all_nodes or {}
        self.start_index = start_index
        self.start_total = start_total
        self.end_index = end_index
        self.end_total = end_total
        self.grid_top_y = grid_top_y
        self.grid_bottom_y = grid_bottom_y

        self.setZValue(0)
        self.pen = QPen(Qt.GlobalColor.black, 2)

    def paint(self, painter, option, widget=None):
        painter.setPen(self.pen)

        # Determine connection points based on node positions
        start_rect = self.start.sceneBoundingRect()
        end_rect = self.end.sceneBoundingRect()
        
        # Determine connection points based on relative positions
        # If nodes are at approximately the same level, connect bottom-to-bottom
        y_diff = abs(start_rect.center().y() - end_rect.center().y())
        if y_diff < 20:  # Same level (within 20 pixels)
            # Both at same level: go out bottom and come in bottom
            p1 = self.start.get_bottom_connection_point(self.start_index, self.start_total)
            p2 = self.end.get_bottom_connection_point(self.end_index, self.end_total)
        elif start_rect.center().y() < end_rect.center().y():
            # Normal: start above end - connect bottom-to-top
            p1 = self.start.get_bottom_connection_point(self.start_index, self.start_total)
            p2 = self.end.get_top_connection_point(self.end_index, self.end_total)
        else:
            # Reverse: start below end - connect top-to-bottom
            p1 = self.start.get_top_connection_point(self.start_index, self.start_total)
            p2 = self.end.get_bottom_connection_point(self.end_index, self.end_total)

        # Calculate mid_y based on horizontal grid
        available_height = self.grid_bottom_y - self.grid_top_y
        step_y = available_height / self.total_grid_levels
        mid_y = self.grid_top_y + (self.grid_level + 0.5) * step_y

        path = [
            p1,
            QPointF(p1.x(), mid_y),
            QPointF(p2.x(), mid_y),
            p2
        ]

        for i in range(len(path) - 1):
            painter.drawLine(path[i], path[i + 1])

        self._draw_arrow_head(painter, path[-2], path[-1])

        if self.label:
            # Position label at a fixed distance from the node box
            label_x = path[1].x() + 5
            
            # Determine if vertical segment goes down or up
            if path[1].y() > path[0].y():
                # Vertical segment goes downward
                label_y = path[0].y() + 15
            else:
                # Vertical segment goes upward
                label_y = path[0].y() - 15
            
            label_pos = QPointF(label_x, label_y)
            painter.drawText(label_pos, self.label)

    def _draw_arrow_head(self, painter, p1, p2):
        angle = math.atan2(-(p2.y() - p1.y()), p2.x() - p1.x())
        size = 8

        p_arrow1 = QPointF(
            p2.x() - size * math.cos(angle + math.pi / 6),
            p2.y() + size * math.sin(angle + math.pi / 6)
        )
        p_arrow2 = QPointF(
            p2.x() - size * math.cos(angle - math.pi / 6),
            p2.y() + size * math.sin(angle - math.pi / 6)
        )

        painter.setBrush(Qt.GlobalColor.black)
        painter.drawPolygon(QPolygonF([p2, p_arrow1, p_arrow2]))


# -------------------------
# Legend
# -------------------------
class Legend(QGraphicsRectItem):
    """Legend showing node groups and their colors."""
    
    def __init__(self, x, y):
        """
        Initialize the legend.
        
        Args:
            x: X position
            y: Y position
        """
        # Calculate size based on number of groups
        item_height = 25
        num_items = len(NodeGroup)
        padding = 15
        width = 200
        height = padding + (num_items * item_height) + padding
        
        super().__init__(x, y, width, height)
        self.setBrush(QBrush(QColor(255, 255, 255, 240)))
        self.setPen(QPen(Qt.GlobalColor.black, 1))
        self.setZValue(15)
        
        self.x = x
        self.y = y
        self.padding = padding
        self.item_height = item_height
        
        # Store items to add later
        self.items_to_add = []
        
        # Add title
        title = QGraphicsTextItem("Groups")
        title_font = title.font()
        title_font.setBold(True)
        title_font.setPointSize(10)
        title.setFont(title_font)
        title.setPos(x + padding, y + padding - 8)
        self.items_to_add.append(title)
        
        # Add legend items
        y_offset = padding + 15
        for group in NodeGroup:
            # Color box
            color_box = QGraphicsRectItem(x + padding, y + y_offset, 15, 15)
            color_box.setBrush(QBrush(QColor(group.color)))
            color_box.setPen(QPen(Qt.GlobalColor.black, 1))
            color_box.setZValue(16)
            self.items_to_add.append(color_box)
            
            # Label
            label = QGraphicsTextItem(group.label)
            label.setPos(x + padding + 25, y + y_offset - 5)
            label.setZValue(16)
            self.items_to_add.append(label)
            
            y_offset += item_height
    
    def add_items_to_scene(self, scene):
        """Add all child items to the scene."""
        for item in self.items_to_add:
            scene.addItem(item)


# -------------------------
# Graph View
# -------------------------
class GraphView(QGraphicsView):
    def __init__(self, ros_node: ROS2Node = None):
        super().__init__()

        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.ros_node = ros_node
        self.setRenderHints(
            QPainter.RenderHint.Antialiasing |
            QPainter.RenderHint.TextAntialiasing
        )

        self.nodes = {}
        self.connections = []

        self.build()

    def add_node(self, group, config, x, y):
        """
        Add a node to the graph.
        
        Args:
            group: Group identifier (e.g., "xy", "z")
            config: NodeConfig instance
            x: X position
            y: Y position
        """
        key = f"{group}:{config.name}"
        node = CalibrationNode(x, y, config)
        self.scene.addItem(node)
        self.nodes[key] = node

    def connect(self, group, start, end, label):
        self.connections.append((group, start, end, label))

    def build(self):
        spacing_x = 180
        start_x = 40
        bottom_offset = spacing_x / 2  # Shift bottom nodes horizontally

        # Define top nodes with their configurations
        top_nodes_xy = [
            NodeConfig("2K Dispenser", service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="NOT IMPLEMENTED", 
                      dialog_class=ExampleDialog, 
                      ros_node=self.ros_node, 
                      group=NodeGroup.GROUP_6),
            NodeConfig("1K Dispenser", service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_1K_dispenser_xy_on_cam_bottom", 
                      dialog_class=ExampleDialog, 
                      ros_node=self.ros_node, 
                      group=NodeGroup.GROUP_6),
            NodeConfig("Camera Top", service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_cameras", 
                      dialog_class=ExampleDialog, 
                      ros_node=self.ros_node, 
                      group=NodeGroup.GROUP_1),
            NodeConfig("Gripper", service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_gripper_xyt_on_camera_bottom", 
                      dialog_class=ExampleDialog, 
                      ros_node=self.ros_node, 
                      group=NodeGroup.GROUP_5),
            NodeConfig("Laser", service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_laser_xy_on_camera_bottom", 
                      dialog_class=ExampleDialog, 
                      ros_node=self.ros_node, 
                      group=NodeGroup.GROUP_3),
            NodeConfig("Confocal Top", service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_confocal_top_xy_on_camera_bottom", 
                      dialog_class=ExampleDialog, ros_node=self.ros_node, group=NodeGroup.GROUP_3),
        ]

        # Define bottom nodes with their configurations
        bottom_nodes_xy = [
            NodeConfig("Camera Bottom", group=NodeGroup.NONE),
            NodeConfig("Confocal Bottom", 
                       service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_confocal_bottom_xy_on_cam_top", 
                      dialog_class=ExampleDialog,
                      ros_node=self.ros_node, group=NodeGroup.GROUP_3),
            NodeConfig("Calibration Cube", 
                       service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_calibration_cube_xy_on_camera_top", 
                      dialog_class=ExampleDialog,
                      ros_node=self.ros_node, group=NodeGroup.GROUP_3),
        ]

        # -------------------------
        # XY GROUP
        # -------------------------
        self.scene.addItem(GroupBox(20, 40, 1100, 300, "XY Calibration"))

        for i, config in enumerate(top_nodes_xy):
            self.add_node("xy", config, start_x + i * spacing_x, 80)

        for i, config in enumerate(bottom_nodes_xy):
            self.add_node("xy", config, start_x + (i + 1) * spacing_x + bottom_offset, 250)

        self.connect("xy", "2K Dispenser", "Camera Bottom", "-15-")
        self.connect("xy", "1K Dispenser", "Camera Bottom", "-14-")
        self.connect("xy", "Camera Top", "Camera Bottom", "-1-")
        self.connect("xy", "Gripper", "Camera Bottom", "-13-")
        self.connect("xy", "Laser", "Camera Bottom", "-7-")
        self.connect("xy", "Confocal Top", "Camera Bottom", "-8-")
        self.connect("xy", "Confocal Bottom", "Camera Top", "-5-")
        self.connect("xy", "Calibration Cube", "Camera Top", "-6-")


        # Define top nodes for Z group
        top_nodes_z = [
            NodeConfig("2K Dispenser", service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="NOT IMPLEMENTED", 
                      dialog_class=ExampleDialog, 
                      ros_node=self.ros_node, 
                      group=NodeGroup.GROUP_5),
            NodeConfig("1K Dispenser", service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_1K_dispenser_z_on_calibration_cube", 
                      dialog_class=ExampleDialog, 
                      ros_node=self.ros_node, 
                      group=NodeGroup.GROUP_5),
            NodeConfig("Camera Top", group=NodeGroup.NONE),
            NodeConfig("Gripper", service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_gripper_plane", 
                      dialog_class=ExampleDialog, 
                      ros_node=self.ros_node, 
                      group=NodeGroup.GROUP_4),
            NodeConfig("Laser", group=NodeGroup.NONE),
            NodeConfig("Confocal Top", service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_confocal_top_z_on_laser", 
                      dialog_class=ExampleDialog, 
                      ros_node=self.ros_node, 
                      group=NodeGroup.GROUP_3),
        ]

        # Define bottom nodes for Z group
        bottom_nodes_z = [
            NodeConfig("Camera Bottom", 
                       group=NodeGroup.NONE),
            NodeConfig("Confocal Bottom", 
                       service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_confocal_bottom_z_on_laser", 
                    dialog_class=ExampleDialog,
                      ros_node=self.ros_node, group=NodeGroup.GROUP_3),
            NodeConfig("Calibration Cube", 
                       service_type="pm_msgs/srv/EmptyWithSuccess", 
                      service_client="/pm_robot_calibration/calibrate_calibration_cube_z_on_confocal_top", 
                      dialog_class=ExampleDialog,
                      ros_node=self.ros_node, group=NodeGroup.GROUP_4),
        ]

        # -------------------------
        # Z GROUP
        # -------------------------
        self.scene.addItem(GroupBox(20, 380, 1100, 300, "Z Calibration"))

        for i, config in enumerate(top_nodes_z):
            self.add_node("z", config, start_x + i * spacing_x, 420)

        for i, config in enumerate(bottom_nodes_z):
            self.add_node("z", config, start_x + (i + 1) * spacing_x + bottom_offset, 590)

        self.connect("z", "2K Dispenser", "Calibration Cube", "-12-")
        self.connect("z", "1K Dispenser", "Calibration Cube", "-11-")
        self.connect("z", "Camera Top", "Camera Bottom", "-5-")
        self.connect("z", "Camera Bottom", "Laser", "-2-")
        self.connect("z", "Gripper", "Confocal Bottom", "-9-")
        self.connect("z", "Confocal Top", "Laser", "-4-")
        self.connect("z", "Confocal Bottom", "Laser", "-3-")
        self.connect("z", "Calibration Cube", "Confocal Top", "-10-")
        self._build_arrows()
        self._add_legend()

    def _build_arrows(self):
        # Group arrows by major group (xy, z)
        major_groups = defaultdict(list)
        for group, start, end, label in self.connections:
            major_groups[group].append((group, start, end, label))

        # Count connections using each node's edges
        node_bottom_connections = defaultdict(list)
        node_top_connections = defaultdict(list)
        
        for group, start, end, label in self.connections:
            start_node = self.nodes[f"{group}:{start}"]
            end_node = self.nodes[f"{group}:{end}"]
            start_rect = start_node.sceneBoundingRect()
            end_rect = end_node.sceneBoundingRect()
            
            # Determine which edges are used based on relative positions
            y_diff = abs(start_rect.center().y() - end_rect.center().y())
            if y_diff < 20:  # Same level
                # Both at same level: both use bottom
                node_bottom_connections[f"{group}:{start}"].append((group, start, end, label))
                node_bottom_connections[f"{group}:{end}"].append((group, start, end, label))
            elif start_rect.center().y() < end_rect.center().y():
                # Normal flow: start node uses bottom, end node uses top
                node_bottom_connections[f"{group}:{start}"].append((group, start, end, label))
                node_top_connections[f"{group}:{end}"].append((group, start, end, label))
            else:
                # Reverse flow: start node uses top, end node uses bottom
                node_top_connections[f"{group}:{start}"].append((group, start, end, label))
                node_bottom_connections[f"{group}:{end}"].append((group, start, end, label))

        # Track which index each connection is for each node's edges
        node_bottom_indices = defaultdict(int)
        node_top_indices = defaultdict(int)

        # Process each major group (xy, z) with its own grid
        for major_group in sorted(major_groups.keys()):
            connections = major_groups[major_group]
            total_arrows = len(connections)
            
            # Find y-range for this group by locating nodes
            top_y = None
            bottom_y = None
            
            for node_key, node in self.nodes.items():
                if not node_key.startswith(major_group):
                    continue
                
                node_name = node_key.split(":")[ 1]
                node_rect = node.sceneBoundingRect()
                node_bottom = node_rect.bottom()
                node_top = node_rect.top()
                
                # Check if this is a top node or bottom node
                is_top_node = node_name in ["2K Dispenser", "1K Dispenser", "Camera Top", 
                                           "Gripper", "Laser", "Confocal Top"]
                
                if is_top_node:
                    if top_y is None:
                        top_y = node_bottom
                    else:
                        top_y = max(top_y, node_bottom)
                else:
                    if bottom_y is None:
                        bottom_y = node_top
                    else:
                        bottom_y = min(bottom_y, node_top)
            
            # Fallback values if nodes not found
            if top_y is None or bottom_y is None:
                if major_group == "xy":
                    top_y, bottom_y = 130, 250
                else:
                    top_y, bottom_y = 470, 590
            
            # Add clearance to top and bottom rows
            clearance = 25
            grid_top_y = top_y + clearance
            grid_bottom_y = bottom_y - clearance
            
            # Build arrows for this group
            for grid_level, (group, start, end, label) in enumerate(connections):
                start_node = self.nodes[f"{group}:{start}"]
                end_node = self.nodes[f"{group}:{end}"]
                start_rect = start_node.sceneBoundingRect()
                end_rect = end_node.sceneBoundingRect()

                # Get indices for connection points based on actual edge usage
                start_key = f"{group}:{start}"
                end_key = f"{group}:{end}"
                
                y_diff = abs(start_rect.center().y() - end_rect.center().y())
                if y_diff < 20:  # Same level
                    # Both at same level: both use bottom
                    start_idx = node_bottom_indices[start_key]
                    start_total = len(node_bottom_connections[start_key])
                    node_bottom_indices[start_key] += 1
                    
                    end_idx = node_bottom_indices[end_key]
                    end_total = len(node_bottom_connections[end_key])
                    node_bottom_indices[end_key] += 1
                elif start_rect.center().y() < end_rect.center().y():
                    # Normal flow
                    start_idx = node_bottom_indices[start_key]
                    start_total = len(node_bottom_connections[start_key])
                    node_bottom_indices[start_key] += 1
                    
                    end_idx = node_top_indices[end_key]
                    end_total = len(node_top_connections[end_key])
                    node_top_indices[end_key] += 1
                else:
                    # Reverse flow
                    start_idx = node_top_indices[start_key]
                    start_total = len(node_top_connections[start_key])
                    node_top_indices[start_key] += 1
                    
                    end_idx = node_bottom_indices[end_key]
                    end_total = len(node_bottom_connections[end_key])
                    node_bottom_indices[end_key] += 1

                arrow = Arrow(
                    start_node,
                    end_node,
                    grid_level,
                    total_arrows,
                    label,
                    self.nodes,
                    start_idx,
                    start_total,
                    end_idx,
                    end_total,
                    grid_top_y,
                    grid_bottom_y
                )
                self.scene.addItem(arrow)

    def _add_legend(self):
        """Add a legend showing node groups and their colors."""
        legend = Legend(1180, 40)
        self.scene.addItem(legend)
        legend.add_items_to_scene(self.scene)

if __name__ == "__main__":
    # -------------------------
    # Run
    # -------------------------
    app = QApplication(sys.argv)
    view = GraphView()
    view.resize(1200, 800)
    view.show()
    sys.exit(app.exec())