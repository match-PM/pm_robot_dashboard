import yaml
from typing import List
from dataclasses import dataclass

from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils

@dataclass
class Configuration:
    nozzles: List[str]
    pneumatics: List[str]


class PmJogToolNode(Node):
    config: Configuration

    def __init__(self):
        super().__init__("pm_robot_dashboard")

        self.get_logger().info("Loading config...")
        self.config = self.load_config()

        self.get_logger().info("Found the following nozzle controllers:")
        for nozzle in self.config.nozzles:
            self.get_logger().info(f"- {nozzle}")

        self.get_logger().info("Found the following pneumatic controllers:")
        for pneumatic in self.config.pneumatics:
            self.get_logger().info(f"- {pneumatic}")

        self.pm_robot_utils = PmRobotUtils(self)


    def load_config(self) -> Configuration:
        try:
            package_path = get_package_share_directory("pm_robot_description")
            file_path = package_path + "/config/pm_robot_control_real_HW.yaml"
            with open(file_path, "r") as file:
                yaml_dict = yaml.safe_load(file)

            nozzles = yaml_dict["pm_nozzle_controller"]["ros__parameters"]["nozzles"]
            pneumatics = yaml_dict["pm_pneumatic_controller"]["ros__parameters"][
                "cylinders"
            ]

            return Configuration(nozzles=nozzles, pneumatics=pneumatics)
        except Exception as e:
            self.get_logger().error(f"Error reading controller config: {e}")
            raise
