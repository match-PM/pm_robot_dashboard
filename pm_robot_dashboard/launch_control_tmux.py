import os
import re
import shlex
import shutil
import signal
import subprocess
from pathlib import Path
from typing import Optional

import PyQt6.QtWidgets as Q
import yaml
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

try:
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from pm_msgs.srv import EmptyWithSuccess
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False


DEFAULT_ROS_SETUP = (
    "source /opt/ros/humble/setup.bash && "
    "source ~/pm_ros2_ws/install/setup.bash && "
    "source ~/Documents/ros2-for-unity/install/setup.bash"
)

DEFAULT_CONFIG = {
    "tmux_session": "pm_robot_dashboard",
    "tmux_window_prefix": "launches",
    "tmux_panes_per_window": 4,
    "tmux_grid_width": 240,
    "tmux_grid_height": 80,
    "ros_setup": DEFAULT_ROS_SETUP,
    "launch_files": [
        {
            "slug": "real_hw",
            "label": "Real HW",
            "window": "real_hw",
            "package": "pm_robot_bringup",
            "launch_file": "pm_robot_real_HW.launch.py",
            "description": "Launch real hardware setup",
        },
        {
            "slug": "sim_hw",
            "label": "Simulation HW",
            "window": "sim_hw",
            "package": "pm_robot_bringup",
            "launch_file": "pm_robot_sim_HW.launch.py",
            "description": "Launch Gazebo simulation setup",
        },
        {
            "slug": "unity_hw",
            "label": "Unity HW",
            "window": "unity_hw",
            "package": "pm_robot_bringup",
            "launch_file": "pm_robot_unity_HW.launch.py",
            "description": "Launch Unity simulation setup",
        },
    ],
}


def _local_config_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "launch_commands.yaml"


def _installed_config_path() -> Optional[Path]:
    if get_package_share_directory is None:
        return None
    try:
        return (
            Path(get_package_share_directory("pm_robot_dashboard"))
            / "config"
            / "launch_commands.yaml"
        )
    except Exception:
        return None


def _load_launch_config() -> dict:
    for path in (_local_config_path(), _installed_config_path()):
        if path is not None and path.exists():
            with path.open("r") as f:
                data = yaml.safe_load(f) or {}
            break
    else:
        data = DEFAULT_CONFIG

    config = dict(DEFAULT_CONFIG)
    config.update(data)
    config["launch_files"] = [_normalize_launch_entry(e) for e in config["launch_files"]]
    return config


def _normalize_launch_entry(entry: dict) -> dict:
    cfg = dict(entry)
    if "slug" not in cfg:
        cfg["slug"] = _safe_name(cfg.get("label", cfg.get("window", "launch")))
    else:
        cfg["slug"] = _safe_name(str(cfg["slug"]))
    if "label" not in cfg:
        cfg["label"] = cfg["slug"]
    cfg["window"] = _safe_name(str(cfg.get("window", cfg["slug"])))
    if "description" not in cfg:
        cfg["description"] = cfg.get("command", "")
    if "command" not in cfg:
        cfg["command"] = f"ros2 launch {cfg['package']} {cfg['launch_file']}"
    return cfg


def _safe_name(value: str) -> str:
    name = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip().lower()).strip("_")
    return name or "launch"


def _shell_line(value: str) -> str:
    return " ".join(str(value).split())


_CONFIG = _load_launch_config()
LAUNCH_FILES = _CONFIG["launch_files"]
TMUX_SESSION = _CONFIG["tmux_session"]
TMUX_WINDOW_PREFIX = _safe_name(str(_CONFIG.get("tmux_window_prefix", "launches")))
TMUX_PANES_PER_WINDOW = max(1, int(_CONFIG.get("tmux_panes_per_window", 4)))
TMUX_GRID_WIDTH = max(120, int(_CONFIG.get("tmux_grid_width", 240)))
TMUX_GRID_HEIGHT = max(40, int(_CONFIG.get("tmux_grid_height", 80)))
ROS_SETUP = _shell_line(_CONFIG["ros_setup"])
for _idx, _cfg in enumerate(LAUNCH_FILES):
    _cfg["tmux_window"] = f"{TMUX_WINDOW_PREFIX}_{(_idx // TMUX_PANES_PER_WINDOW) + 1}"
    _cfg["tmux_pane"] = _idx % TMUX_PANES_PER_WINDOW


def _ros_setup_with_env() -> str:
    """Build the setup string, forwarding key ROS env vars from the current process."""
    env_exports = []
    for var in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION"):
        val = os.environ.get(var)
        if val is not None:
            env_exports.append(f"export {var}={shlex.quote(val)}")
    prefix = " && ".join(env_exports)
    if prefix:
        return f"{prefix} && {ROS_SETUP}"
    return ROS_SETUP


def _pid_file(label: str) -> str:
    return f"/tmp/pm_dashboard_launch_{label.lower().replace(' ', '_')}.pid"


def _pid_alive(pid: int) -> bool:
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


def _remove_pid_file(pid_file: str) -> None:
    try:
        os.remove(pid_file)
    except Exception:
        pass


def _kill_pid(pid: int) -> None:
    try:
        pgid_str = subprocess.check_output(
            ["ps", "-o", "pgid=", "-p", str(pid)],
            stderr=subprocess.DEVNULL,
        ).strip().decode()
        os.killpg(int(pgid_str), signal.SIGINT)
        return
    except Exception:
        pass
    try:
        os.kill(pid, signal.SIGINT)
    except Exception:
        pass


def _tmux(*args: str, check: bool = False) -> subprocess.CompletedProcess:
    result = subprocess.run(
        ["tmux", *args],
        text=True,
        capture_output=True,
    )
    if check and result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        if not detail:
            detail = f"exit code {result.returncode}"
        raise RuntimeError(f"tmux {' '.join(args)} failed: {detail}")
    return result


def _tmux_available() -> bool:
    return shutil.which("tmux") is not None


def _tmux_target(cfg: dict) -> str:
    return f"{TMUX_SESSION}:{cfg['tmux_window']}.{cfg['tmux_pane']}"


def _tmux_session_exists() -> bool:
    return _tmux("has-session", "-t", TMUX_SESSION).returncode == 0


def _tmux_window_exists(window: str) -> bool:
    if not _tmux_session_exists():
        return False
    result = _tmux("list-windows", "-t", TMUX_SESSION, "-F", "#{window_name}")
    return window in result.stdout.splitlines()


def _tmux_pane_count(window: str) -> int:
    if not _tmux_window_exists(window):
        return 0
    result = _tmux("list-panes", "-t", f"{TMUX_SESSION}:{window}", "-F", "#{pane_index}")
    return len(result.stdout.splitlines())


def _tmux_grid_windows() -> list[str]:
    count = (len(LAUNCH_FILES) + TMUX_PANES_PER_WINDOW - 1) // TMUX_PANES_PER_WINDOW
    return [f"{TMUX_WINDOW_PREFIX}_{idx + 1}" for idx in range(count)]


def _configure_grid_window(window: str) -> None:
    _tmux(
        "resize-window",
        "-t",
        f"{TMUX_SESSION}:{window}",
        "-x",
        str(TMUX_GRID_WIDTH),
        "-y",
        str(TMUX_GRID_HEIGHT),
        check=True,
    )
    _tmux("select-layout", "-t", f"{TMUX_SESSION}:{window}", "tiled", check=True)
    _tmux(
        "set-window-option",
        "-t",
        f"{TMUX_SESSION}:{window}",
        "pane-border-status",
        "top",
        check=True,
    )
    _tmux(
        "set-window-option",
        "-t",
        f"{TMUX_SESSION}:{window}",
        "pane-border-format",
        " #{pane_title} ",
        check=True,
    )


def _ensure_tmux_layout() -> None:
    if not _tmux_available():
        raise RuntimeError("tmux not found.")

    if not _tmux_session_exists():
        _tmux(
            "new-session",
            "-d",
            "-s",
            TMUX_SESSION,
            "-n",
            _tmux_grid_windows()[0],
            check=True,
        )

    for window in _tmux_grid_windows():
        if not _tmux_window_exists(window):
            _tmux(
                "new-window",
                "-t",
                TMUX_SESSION,
                "-n",
                window,
                check=True,
            )
        expected_panes = len([cfg for cfg in LAUNCH_FILES if cfg["tmux_window"] == window])
        _configure_grid_window(window)
        while _tmux_pane_count(window) < expected_panes:
            _tmux("split-window", "-t", f"{TMUX_SESSION}:{window}", "-v", check=True)
        _configure_grid_window(window)

    for cfg in LAUNCH_FILES:
        _tmux("select-pane", "-t", _tmux_target(cfg), "-T", cfg["label"], check=True)


def _open_tmux_terminal() -> None:
    _ensure_tmux_layout()
    _tmux("select-window", "-t", f"{TMUX_SESSION}:{_tmux_grid_windows()[0]}", check=True)
    attach_cmd = ["tmux", "attach-session", "-t", TMUX_SESSION]
    candidates = [
        ["terminator", "-e", " ".join(shlex.quote(p) for p in attach_cmd)],
        ["gnome-terminal", "--", *attach_cmd],
        ["x-terminal-emulator", "-e", *attach_cmd],
        ["konsole", "-e", *attach_cmd],
        ["xterm", "-e", *attach_cmd],
    ]
    for cmd in candidates:
        if shutil.which(cmd[0]):
            subprocess.Popen(cmd)
            return
    raise RuntimeError("No supported terminal emulator found.")


def _launch_shell_command(cfg: dict) -> str:
    pid_file = _pid_file(cfg["label"])
    setup = _shell_line(cfg.get("ros_setup", _ros_setup_with_env()))
    command = _shell_line(cfg["command"])
    return (
        f"{setup} && "
        f"echo $$ > {shlex.quote(pid_file)} && "
        f"{command}; "
        "rc=$?; "
        f"rm -f {shlex.quote(pid_file)}; "
        f"echo; echo '[{cfg['label']}] exited with code '$rc; "
        "exec bash"
    )


def _send_literal_command(target: str, command: str) -> None:
    _tmux("send-keys", "-t", target, "-l", command, check=True)
    _tmux("send-keys", "-t", target, "Enter", check=True)


def _start_launch(cfg: dict) -> None:
    _ensure_tmux_layout()
    target = _tmux_target(cfg)
    _tmux("send-keys", "-t", target, "C-c", check=True)
    _send_literal_command(target, "clear")
    _send_literal_command(target, _launch_shell_command(cfg))


def _stop_launch(cfg: dict) -> None:
    _ensure_tmux_layout()
    _tmux("send-keys", "-t", _tmux_target(cfg), "C-c", check=True)


class _LaunchRow(Q.QFrame):
    """A single row containing launch/stop controls for one tmux window."""

    def __init__(self, cfg: dict, parent: Q.QWidget | None = None):
        super().__init__(parent=parent)
        self._cfg = cfg
        self._pid_file = _pid_file(cfg["label"])

        self.setFrameShape(Q.QFrame.Shape.StyledPanel)

        outer = Q.QHBoxLayout()
        outer.setContentsMargins(4, 4, 4, 4)

        info_col = Q.QVBoxLayout()
        lbl = Q.QLabel(cfg["label"])
        lbl.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        desc = Q.QLabel(cfg["description"])
        desc.setStyleSheet("color: grey;")
        desc.setWordWrap(True)
        info_col.addWidget(lbl)
        info_col.addWidget(desc)
        outer.addLayout(info_col, stretch=3)

        self._window_label = Q.QLabel(
            f"tmux: {TMUX_SESSION}:{cfg['tmux_window']}.{cfg['tmux_pane']}"
        )
        self._window_label.setStyleSheet("color: grey;")
        outer.addWidget(self._window_label, stretch=2)

        self._status_label = Q.QLabel("Stopped")
        self._status_label.setStyleSheet("color: grey; font-weight: bold;")
        self._status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        outer.addWidget(self._status_label, stretch=1)

        btn_col = Q.QVBoxLayout()

        self._launch_btn = Q.QPushButton("Start")
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

    def _is_running(self) -> bool:
        pid = _read_pid(self._pid_file)
        if pid is None:
            return False
        if _pid_alive(pid):
            return True
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
                f"'{self._cfg['label']}' is already running.",
            )
            return

        try:
            os.remove(self._pid_file)
        except FileNotFoundError:
            pass

        try:
            _start_launch(self._cfg)
        except Exception as exc:
            Q.QMessageBox.critical(self, "Launch failed", str(exc))
            return

        QTimer.singleShot(1500, self.refresh)

    def _on_stop(self) -> None:
        reply = Q.QMessageBox.question(
            self,
            "Stop launch",
            f"Stop '{self._cfg['label']}'?",
            Q.QMessageBox.StandardButton.Yes | Q.QMessageBox.StandardButton.No,
        )
        if reply != Q.QMessageBox.StandardButton.Yes:
            return

        try:
            _stop_launch(self._cfg)
        except Exception as exc:
            Q.QMessageBox.warning(self, "Stop failed", str(exc))

        pid = _read_pid(self._pid_file)
        if pid is not None:
            _kill_pid(pid)
        _remove_pid_file(self._pid_file)
        QTimer.singleShot(500, self.refresh)

    def _set_running(self, running: bool) -> None:
        if running:
            self._status_label.setText("Running")
            self._status_label.setStyleSheet("color: #2e7d32; font-weight: bold;")
            self._launch_btn.setEnabled(False)
            self._stop_btn.setEnabled(True)
        else:
            self._status_label.setText("Stopped")
            self._status_label.setStyleSheet("color: grey; font-weight: bold;")
            self._launch_btn.setEnabled(True)
            self._stop_btn.setEnabled(False)

    def refresh(self) -> None:
        self._set_running(self._is_running())


class TmuxLaunchControlWidget(Q.QScrollArea):
    """Widget that starts configured launch commands in dedicated tmux windows."""

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

        controls = Q.QHBoxLayout()
        self._open_terminal_btn = Q.QPushButton("Open Terminal")
        self._open_terminal_btn.clicked.connect(self._on_open_terminal)
        controls.addWidget(self._open_terminal_btn)

        self._refresh_btn = Q.QPushButton("Get Running State")
        self._refresh_btn.clicked.connect(self._refresh_all)
        controls.addWidget(self._refresh_btn)

        controls.addStretch()
        layout.addLayout(controls)

        note = Q.QLabel(
            f"Commands run in tmux session '{TMUX_SESSION}'. "
            f"Output is grouped into {TMUX_PANES_PER_WINDOW}-pane tmux grid windows. "
            "Each configured command has its own pane and can be stopped "
            "from the GUI or with Ctrl+C inside tmux."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: grey; font-style: italic;")
        note.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(note)

        self._rows: list[_LaunchRow] = []
        for cfg in LAUNCH_FILES:
            row = _LaunchRow(cfg)
            layout.addWidget(row)
            self._rows.append(row)

        layout.addStretch()
        self.setWidget(container)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_all)
        self._timer.start(2000)

        QTimer.singleShot(0, self._ensure_layout_on_startup)

        if node is not None and _ROS_AVAILABLE:
            self._cb_group = MutuallyExclusiveCallbackGroup()
            for cfg in LAUNCH_FILES:
                slug = cfg["slug"]
                node.create_service(
                    EmptyWithSuccess,
                    f"{node.get_name()}/launch_tmux/{slug}/start",
                    lambda req, res, c=cfg: self._start_launch_callback(req, res, c),
                    callback_group=self._cb_group,
                )
                node.create_service(
                    EmptyWithSuccess,
                    f"{node.get_name()}/launch_tmux/{slug}/stop",
                    lambda req, res, c=cfg: self._stop_launch_callback(req, res, c),
                    callback_group=self._cb_group,
                )

    def _ensure_layout_on_startup(self) -> None:
        try:
            _ensure_tmux_layout()
        except Exception as exc:
            Q.QMessageBox.warning(self, "tmux layout unavailable", str(exc))

    def _on_open_terminal(self) -> None:
        try:
            _open_tmux_terminal()
        except Exception as exc:
            Q.QMessageBox.critical(self, "Open terminal failed", str(exc))

    def _refresh_all(self) -> None:
        for row in self._rows:
            row.refresh()

    def _start_launch_callback(
        self,
        request: "EmptyWithSuccess.Request",
        response: "EmptyWithSuccess.Response",
        cfg: dict,
    ) -> "EmptyWithSuccess.Response":
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

        try:
            _start_launch(cfg)
            response.success = True
            response.message = f"Launch '{cfg['label']}' started in tmux."
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

        try:
            _stop_launch(cfg)
        except Exception as exc:
            if self._node:
                self._node.get_logger().warn(f"tmux Ctrl+C failed: {exc}")
        _kill_pid(pid)
        _remove_pid_file(pid_file)

        response.success = True
        response.message = f"Launch '{cfg['label']}' stopped."
        if self._node:
            self._node.get_logger().info(response.message)
        return response
