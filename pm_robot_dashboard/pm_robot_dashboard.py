import threading

from ament_index_python import get_package_share_directory
import rclpy

from PyQt6.QtGui import QIcon, QPixmap
from PyQt6.QtCore import Qt
import PyQt6.QtWidgets as Q

from pm_robot_dashboard.node import PmJogToolNode
from pm_robot_dashboard.nozzle import NozzleControlWidget
from pm_robot_dashboard.pneumatic import PneumaticControlWidget
from pm_robot_dashboard.joints import JointsControlWidget
from pm_robot_dashboard.ik_control import IkControlWidget
from pm_robot_dashboard.pm_robot_config import PmRobotConfigWidget
from pm_robot_dashboard.launch_control import LaunchControlWidget
from pm_robot_dashboard.pm_robot_tool_changer import ToolChangerService
from rclpy.executors import MultiThreadedExecutor

try:
    from assembly_scene_viewer.py_modules.AssemblySceneViewer import AssemblyScenceViewerWidget
    from assembly_scene_viewer.py_modules.AssemblyJsonModifier import AssemblyModifierWidget

except Exception as e:
    print("Failed to import AssemblySceneViewerWidget, the 3D viewer will not be available.")
    print(e)

try:
    from pm_tf_viewer.submodules.TfViewerApp import TfViewerApp

except Exception as e:
    print("Failed to import TfViewerApp, the TF viewer will not be available.")
    print(e)
class PmJogToolUi(Q.QMainWindow):
    node: PmJogToolNode

    
    def __init__(self, node: PmJogToolNode):
        super().__init__(parent=None)
        self.node = node
        self.setWindowIcon(QIcon(f"{get_package_share_directory('pm_robot_dashboard')}/app_icon.png"))

        self.setWindowTitle("PM Robot Dashboard")

        # Create central widget with vertical layout
        central_widget = Q.QWidget()
        main_layout = Q.QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)  # Remove margins for full-width image
        
        # Add header image
        header_image_label = Q.QLabel()
        image_path = f"{get_package_share_directory('pm_robot_dashboard')}/match_Logo_cut.png"
        pixmap = QPixmap(image_path)
        if not pixmap.isNull():
            # Scale image to fit window width while maintaining aspect ratio
            header_image_label.setPixmap(pixmap.scaledToWidth(self.width() or 800, Qt.TransformationMode.SmoothTransformation))
            header_image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        else:
            header_image_label.setText("Image not found")
        main_layout.addWidget(header_image_label)

        # Create main tab widget
        main_tabs = Q.QTabWidget(self)

        # Create Robot Control tab with subtabs
        robot_control_tabs = Q.QTabWidget()
        robot_control_tabs.addTab(JointsControlWidget(self, node), "Joints")
        robot_control_tabs.addTab(NozzleControlWidget(self, node), "Nozzles")
        robot_control_tabs.addTab(PneumaticControlWidget(self, node), "Pneumatics")
        #robot_control_tabs.addTab(IkControlWidget(self, node), "IK")
        robot_control_tabs.addTab(PmRobotConfigWidget(node), "Robot Config")
        robot_control_tabs.addTab(LaunchControlWidget(node, self), "Launch Files")

        main_tabs.addTab(robot_control_tabs, "Robot Control")

        # Create Info Helpers tab with subtabs
        info_helpers_tabs = Q.QTabWidget()
        
        try:
            info_helpers_tabs.addTab(AssemblyScenceViewerWidget(node), "Assembly Scene Viewer")
            info_helpers_tabs.addTab(AssemblyModifierWidget(), "Assembly JSON Modifier")
        except Exception as e:
            print("Failed to add AssemblySceneViewerWidget and AssemblyModifierWidget tabs, the 3D viewer will not be available.")
            print(e)

        try:
            info_helpers_tabs.addTab(TfViewerApp(node), "TF Viewer")
        except Exception as e:
            print("Failed to add TfViewerApp tab, the TF viewer will not be available.")
            print(e)
        
        main_tabs.addTab(info_helpers_tabs, "Info Helpers")
        
        # Add tabs to layout
        main_layout.addWidget(main_tabs)
        
        # Set central widget
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)


class PmJogToolApp:
    app: Q.QApplication
    ui: PmJogToolUi
    node: PmJogToolNode
    ros_thread: threading.Thread

    executor: MultiThreadedExecutor # TO EVENT THE ROS2 SPIN BLOCKING THE UI


    def __init__(self):
        self.app = Q.QApplication([])
        self.node = PmJogToolNode()
        ToolChangerService(self.node, self.node.pm_robot_utils)

        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.node)

        self.ui = PmJogToolUi(self.node)

    def exec(self):
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()
        self.ui.show()
        self.app.exec()

        self.executor.shutdown()
        self.node.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    app = PmJogToolApp()
    app.exec()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
