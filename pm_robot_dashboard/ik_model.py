
from functools import reduce
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Vector3

from tf2_ros import Buffer, TransformListener
from rosidl_runtime_py.convert import message_to_ordereddict

from pm_moveit_interfaces.srv import MoveToPose
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction
import yaml
import threading


def rgetattr(obj, attr, *args):
    def _getattr(o, name):
        return getattr(o, name, *args)
    return reduce(_getattr, [obj] + attr.split('.'))


def rsetattr(obj, attr, val):
    
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)


class IkControlModel:
 
    # copied / simplified from old PmRobotAxisControl
    BLACKLIST = [
        '1K_Dispenser_Flap', 'Z_Axis', '1K_Dispenser', '2K_Dispenser_Cartridge',
        'Camera_Station', 'Camera_Calibration_Platelet', 'Gonio_Left_Stage_1_Top',
        'Gonio_Left_Base', 'Gonio_Left_Stage_2_Bottom', 'Gonio_Right_Stage_1_Top',
        'Gonio_Right_Stage_1_Bottom', 'Gonio_Right_Stage_2_Bottom', 'Gripper_Rot_Plate',
        'UV_LED_Back', 'UV_Slider_X_Back', 'UV_LED_Front', 'UV_Slider_X_Front',
        'X_Axis', 'axis_base', 'Y_Axis', '1K_Dispenser_TCP', '1K_Dispenser_Tip',
        '2K_Dispenser_TCP', 'Cam1_Toolhead_TCP', 'Camera_Bottom_View_Link',
        'Camera_Bottom_View_Link_Optical', 'pm_robot_base_link',
        'Camera_Top_View_Link', 'Camera_Top_View_Link_Optical',
        'Gonio_Base_Right', 'Laser_Toolhead_TCP', 'PM_Robot_Tool_TCP',
        'PM_Robot_Vacuum_Tool', 'PM_Robot_Vacuum_Tool_Tip', 'housing_hl',
        'base_link_empthy', 'housing_hr', 'housing', 'housing_vl', 'housing_vr',
        'laser_top_link', 'left_match_logo_font', 'left_match_logo_background',
        'match_logo_link', 't_axis_toolchanger'
    ]

    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        ### Cartesian poses (in mm like original code)
        self.current_pose = Pose()
        self.target_pose = Pose()

        #### relative jog in mm
        self.rel_movement = Vector3()

        #### tools
        self.tools: List[str] = [
            'PM_Robot_Tool_TCP',
            '1K_Dispenser_TCP',
            'Cam1_Toolhead_TCP',
            'Laser_Toolhead_TCP',
        ]
        self._active_tool = 'PM_Robot_Tool_TCP'

        # TF buffer & listener
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, node, spin_thread=True)

        # Frames
        self.available_frames: List[str] = []
        self.frame_added = False

        # Joint state stuff
        self.joint_state_msg_received = False
        self.joint_state_list = []  # list of (name, pos, vel, effort)

        # This is acctually not needed.
        # self.joint_state_sub = node.create_subscription(
        #     JointState,
        #     '/joint_states',
        #     self._joint_state_callback,
        #     10,
        #     callback_group=self.callback_group,
        # )

        # This has to be done...
        # self.update_timer = node.create_timer(
        #     0.1, self._update_target, callback_group=self.callback_group
        # )

    ######  getters/setters 

    def set_active_tool(self, tool: str):
        if tool in self.tools:
            self._active_tool = tool
        else:
            self.logger.warn(f"Requested unknown tool '{tool}'")

    def get_active_tool(self) -> str:
        return self._active_tool


    def set_target_from_frame(self, frame: str):

        if not frame:
            self.logger.error("set_target_from_frame: empty frame name")
            return

        try:
            # Try with current node time first
            tf = self.tf_buffer.lookup_transform(
                "world",
                frame,
                self.node.get_clock().now(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            # If that fails (e.g., extrapolation error), try with a slightly older time
            try:
                older_time = self.node.get_clock().now() - rclpy.duration.Duration(seconds=0.5)
                tf = self.tf_buffer.lookup_transform(
                    "world",
                    frame,
                    older_time,
                    timeout=rclpy.duration.Duration(seconds=1.0),
                )
            except Exception as e2:
                self.logger.error(f"[IkControlModel] Failed to lookup TF world->{frame}: {e2}")
                return

        self.target_pose.position.x = tf.transform.translation.x * 1000.0
        self.target_pose.position.y = tf.transform.translation.y * 1000.0
        self.target_pose.position.z = tf.transform.translation.z * 1000.0
        self.target_pose.orientation = tf.transform.rotation

    def update_current_pose_from_active_tool(self):
        tool = self.get_active_tool()

        try:
            # Try with current node time first
            tf = self.tf_buffer.lookup_transform(
                "world",
                tool,
                self.node.get_clock().now(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            # If that fails (e.g., extrapolation error), try with a slightly older time
            try:
                older_time = self.node.get_clock().now() - rclpy.duration.Duration(seconds=0.5)
                tf = self.tf_buffer.lookup_transform(
                    "world",
                    tool,
                    older_time,
                    timeout=rclpy.duration.Duration(seconds=1.0),
                )
            except Exception as e2:
                self.logger.error(f"[IkControlModel] Failed to lookup TF world->{tool}: {e2}")
                return

        self.current_pose.position.x = tf.transform.translation.x * 1000.0
        self.current_pose.position.y = tf.transform.translation.y * 1000.0
        self.current_pose.position.z = tf.transform.translation.z * 1000.0
        self.current_pose.orientation = tf.transform.rotation

    def copy_current_to_target(self):
        self.target_pose.position.x = self.current_pose.position.x
        self.target_pose.position.y = self.current_pose.position.y
        self.target_pose.position.z = self.current_pose.position.z
        self.target_pose.orientation = self.current_pose.orientation


    def _update_target(self):
        # if not self.joint_state_msg_received:
        #     return

        self.update_current_pose_from_active_tool()

        self.target_pose.position.x += float(self.rel_movement.x)
        self.target_pose.position.y += float(self.rel_movement.y)
        self.target_pose.position.z += float(self.rel_movement.z)

        self.rel_movement.x = 0.0
        self.rel_movement.y = 0.0
        self.rel_movement.z = 0.0

        # Run frame list update in a background thread to avoid blocking the UI
        self._update_frame_list()

        #threading.Thread(target=self._update_frame_list, daemon=True).start()

    def _update_frame_list(self):

        try:
            frame_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        except Exception as e:
            self.logger.error(f"Failed to parse TF YAML: {e}")
            return

        if not frame_dict:
            return

        for frame in frame_dict.keys():
            if frame not in self.available_frames and frame not in self.BLACKLIST:
                self.available_frames.append(frame)
                self.frame_added = True

    def _joint_state_callback(self, msg: JointState):
        self.joint_state_msg_received = True
        self.joint_state_list = list(zip(msg.name, msg.position, msg.velocity, msg.effort))

    def move_to_target(self) -> bool:
       
        tool = self.get_active_tool()

        if tool == 'PM_Robot_Tool_TCP':
            client_name = '/pm_moveit_server/move_tool_to_pose'
        elif tool == 'Cam1_Toolhead_TCP':
            client_name = '/pm_moveit_server/move_cam1_to_pose'
        elif tool == 'Laser_Toolhead_TCP':
            client_name = '/pm_moveit_server/move_laser_to_pose'
        else:
            self.logger.error(f"Service for moving '{tool}' not implemented!")
            return False

        service_action = ServiceAction(
            self.node,
            client=client_name,
            service_type='pm_moveit_interfaces/srv/MoveToPose',
        )
        service_action.set_success_identifier('success')

        req = MoveToPose.Request()
        req.move_to_pose.position.x = self.target_pose.position.x / 1000.0
        req.move_to_pose.position.y = self.target_pose.position.y / 1000.0
        req.move_to_pose.position.z = self.target_pose.position.z / 1000.0
        req.execute_movement = True

        req_dict = message_to_ordereddict(req)
        service_action.set_request_from_dict(req_dict)

        success = service_action.execute()
        self.logger.info(f"Result of {tool}: {success}")
        return success
