import threading

from pm_robot_dashboard import node
import rclpy

import PyQt6.QtWidgets as Q

from pm_robot_dashboard.node import PmJogToolNode
from pm_robot_dashboard.nozzle import NozzleControlWidget
from pm_robot_dashboard.pneumatic import PneumaticControlWidget
from pm_robot_dashboard.joints import JointsControlWidget
from pm_robot_dashboard.ik_control import IkControlWidget
class PmJogToolUi(Q.QMainWindow):
    node: PmJogToolNode

    def __init__(self, node: PmJogToolNode):
        super().__init__(parent=None)
        self.node = node

        tabs = Q.QTabWidget(self)
        ik_tab = IkControlWidget(node)
        tabs.addTab(ik_tab, "IK Control")
        tabs.addTab(JointsControlWidget(self, node), "Joints")
        tabs.addTab(NozzleControlWidget(self, node), "Nozzles")
        tabs.addTab(PneumaticControlWidget(self, node), "Pneumatics")
        
        
        
        self.setCentralWidget(tabs)


class PmJogToolApp:
    app: Q.QApplication
    ui: PmJogToolUi
    node: PmJogToolNode
    ros_thread: threading.Thread

    def __init__(self):
        self.app = Q.QApplication([])
        self.node = PmJogToolNode()
        self.ui = PmJogToolUi(self.node)

    def exec(self):
        self.ros_thread = threading.Thread(target=lambda: rclpy.spin(self.node))
        self.ros_thread.start()
        self.ui.show()
        self.app.exec()


def main(args=None):
    rclpy.init(args=args)

    app = PmJogToolApp()
    app.exec()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
