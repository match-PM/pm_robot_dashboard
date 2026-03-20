import sys
import math
from collections import defaultdict

from PyQt6.QtWidgets import (
    QApplication, QGraphicsView, QGraphicsScene,
    QGraphicsRectItem, QGraphicsTextItem, QDialog
)
from PyQt6.QtGui import QPen, QBrush, QPainter, QColor, QPolygonF
from PyQt6.QtCore import Qt, QPointF


# -------------------------
# Dummy Dialog
# -------------------------
class ExampleDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Example Dialog")


# -------------------------
# ROS2 Stub
# -------------------------
class ROSInterface:
    def call_service(self, service_name: str):
        print(f"[ROS2] Calling service: {service_name}")
        # TODO: replace with real ROS2 client


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
# Node
# -------------------------
class Node(QGraphicsRectItem):
    WIDTH = 140
    HEIGHT = 50

    def __init__(self, x, y, name, service_name=None, dialog_class=None, ros=None):
        super().__init__(0, 0, self.WIDTH, self.HEIGHT)
        self.setPos(x, y)

        self.name = name
        self.service_name = service_name
        self.dialog_class = dialog_class
        self.ros = ros

        self.setBrush(QBrush(QColor("#f4a261")))
        self.setPen(QPen(Qt.GlobalColor.black, 2))
        self.setZValue(10)

        self.label = QGraphicsTextItem(name, self)
        self._center_text()

    def _center_text(self):
        rect = self.rect()
        text_rect = self.label.boundingRect()
        self.label.setPos(
            (rect.width() - text_rect.width()) / 2,
            (rect.height() - text_rect.height()) / 2
        )

    def center_top(self):
        r = self.sceneBoundingRect()
        return QPointF(r.center().x(), r.top())

    def center_bottom(self):
        r = self.sceneBoundingRect()
        return QPointF(r.center().x(), r.bottom())

    def get_bottom_connection_point(self, index=0, total=1):
        """Get connection point at bottom of node, distributed across width if multiple connections."""
        r = self.sceneBoundingRect()
        if total == 1:
            x = r.center().x()
        else:
            # Distribute points along the width
            x = r.left() + (r.width() / (total + 1)) * (index + 1)
        return QPointF(x, r.bottom())

    def get_top_connection_point(self, index=0, total=1):
        """Get connection point at top of node, distributed across width if multiple connections."""
        r = self.sceneBoundingRect()
        if total == 1:
            x = r.center().x()
        else:
            # Distribute points along the width
            x = r.left() + (r.width() / (total + 1)) * (index + 1)
        return QPointF(x, r.top())

    def mousePressEvent(self, event):
        print(f"[CLICK] {self.name}")

        if self.service_name and self.ros:
            self.ros.call_service(self.service_name)

        if self.dialog_class:
            dialog = self.dialog_class()
            dialog.exec()

        super().mousePressEvent(event)


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
        
        # If start is above end, connect bottom-to-top (normal flow)
        # If start is below end, connect top-to-bottom (reverse flow)
        if start_rect.center().y() < end_rect.center().y():
            # Normal: top nodes to bottom nodes
            p1 = self.start.get_bottom_connection_point(self.start_index, self.start_total)
            p2 = self.end.get_top_connection_point(self.end_index, self.end_total)
        else:
            # Reverse: bottom nodes to top nodes
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
            label_pos = QPointF((path[1].x() + path[2].x()) / 2, mid_y - 5)
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
# Graph View
# -------------------------
class GraphView(QGraphicsView):
    def __init__(self):
        super().__init__()

        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        self.setRenderHints(
            QPainter.RenderHint.Antialiasing |
            QPainter.RenderHint.TextAntialiasing
        )

        self.nodes = {}
        self.connections = []
        self.ros = ROSInterface()

        self.build()

    def add_node(self, group, name, x, y, service=None, dialog=None):
        key = f"{group}:{name}"
        node = Node(x, y, name, service, dialog, self.ros)
        self.scene.addItem(node)
        self.nodes[key] = node

    def connect(self, group, start, end, label):
        self.connections.append((group, start, end, label))

    def build(self):
        spacing_x = 180
        start_x = 40
        bottom_offset = spacing_x / 2  # Shift bottom nodes horizontally

        top_names = ["2K Dispenser", "1K Dispenser", "Camera Top",
                     "Gripper", "Laser", "Confocal Top"]

        bottom_names = ["Camera Bottom", "Confocal Bottom", "Calibration Cube"]

        # -------------------------
        # XY GROUP
        # -------------------------
        self.scene.addItem(GroupBox(20, 40, 1100, 300, "XY Calibration"))

        for i, name in enumerate(top_names):
            self.add_node("xy", name, start_x + i * spacing_x, 80,
                          service=f"/xy/{name.replace(' ', '_').lower()}",
                          dialog=ExampleDialog)

        for i, name in enumerate(bottom_names):
            self.add_node("xy", name, start_x + (i + 1) * spacing_x + bottom_offset, 250)

        self.connect("xy", "2K Dispenser", "Camera Bottom", "-15-")
        self.connect("xy", "1K Dispenser", "Camera Bottom", "-14-")
        self.connect("xy", "Camera Top", "Camera Bottom", "-1")
        self.connect("xy", "Gripper", "Camera Bottom", "-13")
        self.connect("xy", "Laser", "Camera Bottom", "-7")
        self.connect("xy", "Confocal Top", "Camera Bottom", "-8")
        self.connect("xy", "Confocal Bottom", "Camera Top", "-5")
        self.connect("xy", "Calibration Cube", "Camera Top", "-6")


        # -------------------------
        # Z GROUP
        # -------------------------
        self.scene.addItem(GroupBox(20, 380, 1100, 300, "Z Calibration"))

        for i, name in enumerate(top_names):
            self.add_node("z", name, start_x + i * spacing_x, 420,
                          service=f"/z/{name.replace(' ', '_').lower()}")

        for i, name in enumerate(bottom_names):
            self.add_node("z", name, start_x + (i + 1) * spacing_x + bottom_offset, 590)

        self.connect("z", "2K Dispenser", "Calibration Cube", "-0-")
        self.connect("z", "1K Dispenser", "Calibration Cube", "-14-")
        self.connect("z", "Camera Top", "Camera Bottom", "-5")
        self.connect("z", "Gripper", "Confocal Bottom", "-13")
        self._build_arrows()

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
            if start_rect.center().y() < end_rect.center().y():
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
                
                if start_rect.center().y() < end_rect.center().y():
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

if __name__ == "__main__":
    # -------------------------
    # Run
    # -------------------------
    app = QApplication(sys.argv)
    view = GraphView()
    view.resize(1200, 800)
    view.show()
    sys.exit(app.exec())