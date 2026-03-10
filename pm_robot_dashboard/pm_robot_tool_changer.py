import os
import subprocess
import time

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from pm_msgs.srv import ChangeToolAndRestart, EmptyWithSuccess

from pm_robot_dashboard.launch_control import (
    LAUNCH_FILES,
    _pid_file,
    _pid_alive,
    _read_pid,
    _kill_pid,
    _ros_setup_with_env,
)

# Build a slug→config lookup from the shared LAUNCH_FILES list
_LAUNCH_CONFIGS: dict = {cfg["slug"]: cfg for cfg in LAUNCH_FILES}


class ToolChangerService:
    """Registers and handles the ``change_tool_and_restart`` ROS 2 service.

    Instantiate once inside a ROS 2 node ``__init__``::

        self._tool_changer = ToolChangerService(
            self, self.pm_robot_utils,
            callback_group=self.callback_group_me,
        )

    The service is advertised as ``<node_name>/change_tool_and_restart``
    (type ``pm_msgs/srv/ChangeToolAndRestart``).
    """

    def __init__(self, node, pm_robot_utils, callback_group=None):
        self._node = node
        self._pm_robot_utils = pm_robot_utils

        if callback_group is None:
            callback_group = MutuallyExclusiveCallbackGroup()

        node.create_service(
            ChangeToolAndRestart,
            f"{node.get_name()}/change_tool_and_restart",
            self._change_tool_and_restart_callback,
            callback_group=callback_group,
        )

        # Client to trigger Unity scene reconfiguration after a tool change
        self._configure_unity_client = node.create_client(
            EmptyWithSuccess,
            "/configure_robot_node/configure_robot",
        )

    # ---------------------------------------------------------------------- #
    # Service callback                                                        #
    # ---------------------------------------------------------------------- #

    def _change_tool_and_restart_callback(
        self,
        request: ChangeToolAndRestart.Request,
        response: ChangeToolAndRestart.Response,
    ) -> ChangeToolAndRestart.Response:
        """Change the active tool in the robot config and restart the running launch.

        Request fields:
            tool_name     -- name of the tool to activate (must match an entry in the config)
            launch_slug   -- 'real_hw' / 'sim_hw' / 'unity_hw', or empty to auto-detect
            restart_delay -- seconds between stop and start (0 = default 10 s)
        """
        tool_name = request.tool_name.strip()
        launch_slug = request.launch_slug.strip()
        restart_delay = float(request.restart_delay) if request.restart_delay > 0 else 10.0

        self._node.get_logger().info(
            f"change_tool_and_restart: tool='{tool_name}' "
            f"launch='{launch_slug or 'auto'}' delay={restart_delay}s"
        )
        
        # ------------------------------------------------------------------ #
        # 2. Find and apply the tool in the config                            #
        # ------------------------------------------------------------------ #
        pm_config = self._pm_robot_utils.pm_robot_config

        grippers = [
            ("vacuum_gripper", pm_config.tool._gripper_vacuum),
            ("gripper_1_jaw",  pm_config.tool._gripper_1_jaw),
            ("gripper_2_jaw",  pm_config.tool._gripper_2_jaw),
        ]

        found_gripper = None
        found_gripper_name = ""
        for gripper_name, gripper in grippers:
            available = [t[0] for t in gripper.get_available_tools()]
            if tool_name in available:
                found_gripper = gripper
                found_gripper_name = gripper_name
                break

        if found_gripper is None:
            all_tools = []
            for _, gripper in grippers:
                all_tools.extend([t[0] for t in gripper.get_available_tools()])
            response.success = False
            response.message = (
                f"Tool '{tool_name}' not found in config. Available: {all_tools}"
            )
            self._node.get_logger().error(response.message)
            return response

        self._node.get_logger().info(
            f"Tool '{tool_name}' found in gripper '{found_gripper_name}'. Updating config..."
        )

        # Deactivate all grippers, activate the right one, set the tool
        for _, gripper in grippers:
            gripper.deactivate()
        found_gripper.activate()
        found_gripper.set_current_tool(tool=tool_name)
        pm_config.save_config()
        self._node.get_logger().info("Config saved.")

        # Notify Unity to reload the config and update the scene
        self._trigger_unity_configure()

        ## wait a moment to ensure the config file timestamp is updated before the launch tries to read it
        time.sleep(1)

        # ------------------------------------------------------------------ #
        # 1. Resolve which launch to restart                                  #
        # ------------------------------------------------------------------ #
        if launch_slug:
            if launch_slug not in _LAUNCH_CONFIGS:
                response.success = False
                response.message = (
                    f"Unknown launch_slug '{launch_slug}'. "
                    f"Valid: {list(_LAUNCH_CONFIGS.keys())}"
                )
                self._node.get_logger().error(response.message)
                return response
            cfg = _LAUNCH_CONFIGS[launch_slug]
        else:
            # Auto-detect: find the first running launch
            cfg = None
            for slug, c in _LAUNCH_CONFIGS.items():
                pid = _read_pid(_pid_file(c["label"]))
                if pid is not None and _pid_alive(pid):
                    cfg = c
                    launch_slug = slug
                    break
            if cfg is None:
                response.success = False
                response.message = (
                    "No running launch detected. "
                    "Start a launch first or specify launch_slug."
                )
                self._node.get_logger().error(response.message)
                return response

        self._node.get_logger().info(f"Target launch: '{cfg['label']}'")


        # ------------------------------------------------------------------ #
        # 3. Stop the running launch                                          #
        # ------------------------------------------------------------------ #
        pid_file = _pid_file(cfg["label"])
        pid = _read_pid(pid_file)
        if pid is not None and _pid_alive(pid):
            self._node.get_logger().info(
                f"Stopping launch '{cfg['label']}' (PID {pid})..."
            )
            _kill_pid(pid)
            try:
                os.remove(pid_file)
            except Exception:
                pass
        else:
            self._node.get_logger().warn(
                f"Launch '{cfg['label']}' was not running – will start it anyway."
            )

        # ------------------------------------------------------------------ #
        # 4. Wait, then restart                                               #
        # ------------------------------------------------------------------ #
        self._node.get_logger().info(f"Waiting {restart_delay} s before restarting...")
        time.sleep(restart_delay)

        tab_title = f"robot_launch_{cfg['label'].lower().replace(' ', '_')}"
        ros_setup = _ros_setup_with_env()
        bash_cmd = (
            f"{ros_setup} && "
            f"echo $$ > {pid_file} && "
            f"ros2 launch {cfg['package']} {cfg['launch_file']}; "
            f"rm -f {pid_file}; exec bash"
        )
        cmd = [
            "terminator", "--new-tab",
            "--title", tab_title,
            "-e", f"bash -c '{bash_cmd}'",
        ]
        try:
            subprocess.Popen(cmd)
            response.success = True
            response.message = (
                f"Tool changed to '{tool_name}' (gripper: {found_gripper_name}), "
                f"config saved, launch '{cfg['label']}' restarted."
            )
            self._node.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = f"Config updated but failed to restart launch: {exc}"
            self._node.get_logger().error(response.message)

        return response

    # ---------------------------------------------------------------------- #
    # Unity integration                                                       #
    # ---------------------------------------------------------------------- #

    def _trigger_unity_configure(self) -> None:
        """Call the Unity configure_robot service to update the scene.

        The call is best-effort: if Unity is not running or the service is
        unavailable the warning is logged and execution continues normally.
        """
        if not self._configure_unity_client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warn(
                "Unity configure_robot service not available – skipping Unity scene update."
            )
            return

        req = EmptyWithSuccess.Request()
        try:
            future = self._configure_unity_client.call_async(req)
            # Spin briefly so the response arrives inside this callback context
            import rclpy
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            if future.done():
                resp: EmptyWithSuccess.Response = future.result()
                if resp.success:
                    self._node.get_logger().info("Unity scene reconfigured successfully.")
                else:
                    self._node.get_logger().warn(
                        f"Unity configure_robot returned success=False: {getattr(resp, 'message', '')}"
                    )
            else:
                self._node.get_logger().warn("Unity configure_robot call timed out.")
        except Exception as exc:
            self._node.get_logger().error(f"Error calling Unity configure_robot: {exc}")
