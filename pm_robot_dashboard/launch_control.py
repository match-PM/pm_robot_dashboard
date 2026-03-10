import os
import signal
import subprocess
from typing import Optional

import PyQt6.QtWidgets as Q
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont

try:
    from rclpy.node import Node
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from pm_msgs.srv import EmptyWithSuccess
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False


# --------------------------------------------------------------------------- #
# Descriptor for a single launch file entry                                   #
# --------------------------------------------------------------------------- #
LAUNCH_FILES = [
    {
        "slug": "real_hw",
        "label": "Real HW",
        "package": "pm_robot_bringup",
        "launch_file": "pm_robot_real_HW.launch.py",
        "description": "Launch real hardware setup",
    },
    {
        "slug": "sim_hw",
        "label": "Simulation HW",
        "package": "pm_robot_bringup",
        "launch_file": "pm_robot_sim_HW.launch.py",
        "description": "Launch Gazebo simulation setup",
    },
    {
        "slug": "unity_hw",
        "label": "Unity HW",
        "package": "pm_robot_bringup",
        "launch_file": "pm_robot_unity_HW.launch.py",
        "description": "Launch Unity simulation setup",
    },
]

ROS_SETUP = (
    "source /opt/ros/humble/setup.bash && "
    "source ~/ros2_ws/install/setup.bash && "
    "source ~/Documents/ros2-for-unity/install/setup.bash"
)


def _ros_setup_with_env() -> str:
    """Build the setup string, forwarding key ROS env vars from the current process."""
    env_exports = []
    for var in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION"):
        val = os.environ.get(var)
        if val is not None:
            env_exports.append(f"export {var}={val}")
    prefix = " && ".join(env_exports)
    if prefix:
        return f"{prefix} && {ROS_SETUP}"
    return ROS_SETUP


# --------------------------------------------------------------------------- #
# Helpers                                                                      #
# --------------------------------------------------------------------------- #
def _pid_file(label: str) -> str:
    return f"/tmp/pm_dashboard_launch_{label.lower().replace(' ', '_')}.pid"


def _pid_alive(pid: int) -> bool:
    """Return True if a process with this PID still exists."""
    try:
        os.kill(pid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def _read_pid(pid_file: str) -> Optional[int]:
    try:
        with open(pid_file) as f:
            return int(f.read().strip())
    except Exception:
        return None


def _kill_pid(pid: int) -> None:
    """
    Send SIGINT to the whole process group of pid (equivalent to Ctrl+C),
    then cleanly fall back to SIGTERM.
    """
    # Try to get PGID via `ps` so it works cross-session
    try:
        pgid_str = subprocess.check_output(
            ["ps", "-o", "pgid=", "-p", str(pid)],
            stderr=subprocess.DEVNULL,
        ).strip().decode()
        pgid = int(pgid_str)
        os.killpg(pgid, signal.SIGINT)
        return
    except Exception:
        pass
    # Fallback: kill only the PID directly
    try:
        os.kill(pid, signal.SIGINT)
    except Exception:
        pass


# --------------------------------------------------------------------------- #
# Per-launch-file row widget                                                   #
# --------------------------------------------------------------------------- #
class _LaunchRow(Q.QFrame):
    """A single row containing launch/stop controls for one launch file."""

    def __init__(self, cfg: dict, parent: Q.QWidget | None = None):
        super().__init__(parent=parent)
        self._cfg = cfg
        self._pid_file = _pid_file(cfg["label"])

        self.setFrameShape(Q.QFrame.Shape.StyledPanel)

        outer = Q.QHBoxLayout()
        outer.setContentsMargins(4, 4, 4, 4)

        # ---- info column -------------------------------------------------- #
        info_col = Q.QVBoxLayout()
        lbl = Q.QLabel(cfg["label"])
        lbl.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        desc = Q.QLabel(cfg["description"])
        desc.setStyleSheet("color: grey;")
        info_col.addWidget(lbl)
        info_col.addWidget(desc)
        outer.addLayout(info_col, stretch=3)

        # ---- status indicator --------------------------------------------- #
        self._status_label = Q.QLabel("● Stopped")
        self._status_label.setStyleSheet("color: grey; font-weight: bold;")
        self._status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        outer.addWidget(self._status_label, stretch=1)

        # ---- buttons ------------------------------------------------------ #
        btn_col = Q.QVBoxLayout()

        self._launch_btn = Q.QPushButton("Launch")
        self._launch_btn.setMinimumWidth(90)
        self._launch_btn.setStyleSheet("background-color: #2e7d32; color: white;")
        self._launch_btn.clicked.connect(self._on_launch)
        btn_col.addWidget(self._launch_btn)

        self._stop_btn = Q.QPushButton("Stop")
        self._stop_btn.setMinimumWidth(90)
        self._stop_btn.setStyleSheet("background-color: #c62828; color: white;")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop)
        btn_col.addWidget(self._stop_btn)

        outer.addLayout(btn_col, stretch=1)
        self.setLayout(outer)

    # ---------------------------------------------------------------------- #
    def _is_running(self) -> bool:
        """Check PID file existence and process liveness."""
        pid = _read_pid(self._pid_file)
        if pid is None:
            return False
        if _pid_alive(pid):
            return True
        # PID file exists but process is gone – clean up stale file
        try:
            os.remove(self._pid_file)
        except Exception:
            pass
        return False

    def _on_launch(self) -> None:
        if self._is_running():
            Q.QMessageBox.warning(
                self,
                "Already running",
                f"'{self._cfg['label']}' is already running.\nStop it first.",
            )
            return

        # Remove stale PID file if present
        try:
            os.remove(self._pid_file)
        except FileNotFoundError:
            pass

        tab_title = f"robot_launch_{self._cfg['label'].lower().replace(' ', '_')}"
        bash_cmd = (
            f"{_ros_setup_with_env()} && "
            f"echo $$ > {self._pid_file} && "
            f"ros2 launch {self._cfg['package']} {self._cfg['launch_file']}; "
            f"rm -f {self._pid_file}; exec bash"
        )
        cmd = [
            "terminator",
            "--new-tab",
            "--title", tab_title,
            "-e", f"bash -c '{bash_cmd}'",
        ]

        try:
            subprocess.Popen(cmd)
        except FileNotFoundError:
            Q.QMessageBox.critical(self, "Launch failed", "terminator not found.")
            return
        except Exception as exc:
            Q.QMessageBox.critical(self, "Launch failed", str(exc))
            return

        # Wait briefly for bash to write the PID file, then update UI
        QTimer.singleShot(1500, self._check_started)

    def _check_started(self) -> None:
        """Called ~1.5 s after launch to confirm PID file appeared."""
        if self._is_running():
            self._set_running(True)
        else:
            # Give it one more second
            QTimer.singleShot(1500, self._check_started_final)

    def _check_started_final(self) -> None:
        if self._is_running():
            self._set_running(True)
        else:
            self._set_running(False)

    def _on_stop(self) -> None:
        reply = Q.QMessageBox.question(
            self,
            "Stop launch",
            f"Stop '{self._cfg['label']}'?",
            Q.QMessageBox.StandardButton.Yes | Q.QMessageBox.StandardButton.No,
        )
        if reply != Q.QMessageBox.StandardButton.Yes:
            return

        pid = _read_pid(self._pid_file)
        if pid is not None:
            _kill_pid(pid)
            try:
                os.remove(self._pid_file)
            except Exception:
                pass

        self._set_running(False)

    def _set_running(self, running: bool) -> None:
        if running:
            self._status_label.setText("● Running")
            self._status_label.setStyleSheet("color: #2e7d32; font-weight: bold;")
            self._launch_btn.setEnabled(False)
            self._stop_btn.setEnabled(True)
        else:
            self._status_label.setText("● Stopped")
            self._status_label.setStyleSheet("color: grey; font-weight: bold;")
            self._launch_btn.setEnabled(True)
            self._stop_btn.setEnabled(False)

    def poll(self) -> None:
        """Called by the parent timer to sync UI with actual process state."""
        running = self._is_running()
        # Only update if state changed (avoid needless redraws)
        currently_running = self._stop_btn.isEnabled()
        if running != currently_running:
            self._set_running(running)


# --------------------------------------------------------------------------- #
# Main LaunchControlWidget                                                     #
# --------------------------------------------------------------------------- #
class LaunchControlWidget(Q.QScrollArea):
    """Widget that lets the user launch / stop ROS 2 launch files.

    When a ROS 2 node is supplied, the widget also advertises
    ``<node_name>/launch/<slug>/start`` and ``<node_name>/launch/<slug>/stop``
    services (type ``pm_msgs/srv/EmptyWithSuccess``) so that external nodes can
    trigger launch files programmatically.
    """

    def __init__(self, node=None, parent: Q.QWidget | None = None):
        super().__init__(parent=parent)
        self._node = node
        self.setWidgetResizable(True)

        container = Q.QWidget()
        layout = Q.QVBoxLayout(container)
        layout.setSpacing(8)
        layout.setContentsMargins(12, 12, 12, 12)

        title = Q.QLabel("Launch File Control")
        title.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        note = Q.QLabel(
            "Each launch opens a new Terminator tab. "
            "'Stop' sends SIGINT to the process group."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: grey; font-style: italic;")
        note.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(note)

        layout.addSpacing(8)

        self._rows: list[_LaunchRow] = []
        for cfg in LAUNCH_FILES:
            row = _LaunchRow(cfg)
            layout.addWidget(row)
            self._rows.append(row)

        layout.addStretch()
        self.setWidget(container)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(2000)

        # Register ROS 2 services when a node is available
        if node is not None and _ROS_AVAILABLE:
            self._cb_group = MutuallyExclusiveCallbackGroup()
            for cfg in LAUNCH_FILES:
                slug = cfg["slug"]
                node.create_service(
                    EmptyWithSuccess,
                    f"{node.get_name()}/launch/{slug}/start",
                    lambda req, res, c=cfg: self._start_launch_callback(req, res, c),
                    callback_group=self._cb_group,
                )
                node.create_service(
                    EmptyWithSuccess,
                    f"{node.get_name()}/launch/{slug}/stop",
                    lambda req, res, c=cfg: self._stop_launch_callback(req, res, c),
                    callback_group=self._cb_group,
                )

    # ---------------------------------------------------------------------- #
    # ROS 2 service callbacks                                                 #
    # ---------------------------------------------------------------------- #

    def _start_launch_callback(
        self,
        request: "EmptyWithSuccess.Request",
        response: "EmptyWithSuccess.Response",
        cfg: dict,
    ) -> "EmptyWithSuccess.Response":
        """ROS 2 service: start the given launch file in a new Terminator tab."""
        pid_file = _pid_file(cfg["label"])
        pid = _read_pid(pid_file)
        if pid is not None and _pid_alive(pid):
            response.success = False
            response.message = (
                f"Launch '{cfg['label']}' is already running (PID {pid})."
            )
            if self._node:
                self._node.get_logger().warn(response.message)
            return response

        try:
            os.remove(pid_file)
        except FileNotFoundError:
            pass

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
            response.message = f"Launch '{cfg['label']}' started."
            if self._node:
                self._node.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = f"Failed to start launch '{cfg['label']}': {exc}"
            if self._node:
                self._node.get_logger().error(response.message)
        return response

    def _stop_launch_callback(
        self,
        request: "EmptyWithSuccess.Request",
        response: "EmptyWithSuccess.Response",
        cfg: dict,
    ) -> "EmptyWithSuccess.Response":
        """ROS 2 service: stop the given launch file by sending SIGINT to its process group."""
        pid_file = _pid_file(cfg["label"])
        pid = _read_pid(pid_file)
        if pid is None or not _pid_alive(pid):
            response.success = False
            response.message = f"Launch '{cfg['label']}' is not running."
            if self._node:
                self._node.get_logger().warn(response.message)
            try:
                os.remove(pid_file)
            except Exception:
                pass
            return response

        _kill_pid(pid)
        try:
            os.remove(pid_file)
        except Exception:
            pass

        response.success = True
        response.message = f"Launch '{cfg['label']}' stopped."
        if self._node:
            self._node.get_logger().info(response.message)
        return response

    # ---------------------------------------------------------------------- #

    def _poll(self) -> None:
        for row in self._rows:
            row.poll()
