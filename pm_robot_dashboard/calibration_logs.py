import json
import os
import re
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


LAST_CALIBRATIONS_FILE = "last_calibrations.yaml"


@dataclass
class CalibrationLogEvent:
    name: str
    mode: str
    log_dir: str
    timestamp_text: str
    timestamp: Optional[datetime]
    yaml_available: bool
    json_path: Optional[str]
    json_available: bool


def resolve_calibration_logs_dir() -> str:
    try:
        package_path = get_package_share_directory("pm_robot_calibration")
        return os.path.join(package_path, "calibration_logs")
    except PackageNotFoundError:
        return os.path.abspath("calibration_logs")


def resolve_real_calibration_logs_dir(node=None) -> Optional[str]:
    if node is None or not hasattr(node, "pm_robot_utils"):
        return None

    try:
        config = node.pm_robot_utils.pm_robot_config
        real_config_path = config.get_joint_config_path(use_real_HW=True)
        return os.path.join(os.path.dirname(os.path.abspath(real_config_path)), "calibration_logs")
    except Exception:
        return None


def resolve_calibration_log_sources(node=None) -> List[Tuple[str, str]]:
    sources = [("Simulation", resolve_calibration_logs_dir())]
    real_log_dir = resolve_real_calibration_logs_dir(node)
    if real_log_dir:
        sources.append(("Real Hardware", real_log_dir))

    deduplicated = []
    seen_paths = set()
    for mode, log_dir in sources:
        normalized_path = os.path.abspath(log_dir)
        if normalized_path in seen_paths:
            continue
        seen_paths.add(normalized_path)
        deduplicated.append((mode, log_dir))

    return deduplicated


def normalize_mode_label(mode: Optional[str]) -> str:
    text = str(mode or "").strip()
    lowered = text.lower()
    if "real" in lowered:
        return "Real Hardware"
    if "sim" in lowered or "unity" in lowered or "gazebo" in lowered:
        return "Simulation"
    return text or "Unknown"


def parse_timestamp(value: Any) -> Optional[datetime]:
    if isinstance(value, datetime):
        return value

    text = str(value).strip() if value is not None else ""
    if not text:
        return None

    candidates = [text]
    if text.endswith("Z"):
        candidates.append(text[:-1] + "+00:00")
    if " " in text and "T" not in text:
        candidates.append(text.replace(" ", "T", 1))

    for candidate in candidates:
        try:
            return datetime.fromisoformat(candidate)
        except ValueError:
            pass

    for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S", "%Y%m%d_%H%M%S"):
        try:
            return datetime.strptime(text, fmt)
        except ValueError:
            pass

    return None


def format_timestamp(timestamp: Optional[datetime], fallback: Any = "") -> str:
    if timestamp is None:
        return str(fallback) if fallback is not None else "Unknown"
    return timestamp.strftime("%Y-%m-%d %H:%M:%S")


def load_last_calibrations(log_dir: Optional[str] = None) -> Dict[str, Dict[str, Optional[str]]]:
    log_dir = log_dir or resolve_calibration_logs_dir()
    file_path = os.path.join(log_dir, LAST_CALIBRATIONS_FILE)
    if not os.path.isfile(file_path):
        return {}

    with open(file_path, "r") as file:
        loaded = yaml.safe_load(file) or {}

    if isinstance(loaded, dict):
        events = {}
        for key, value in loaded.items():
            if isinstance(value, dict):
                timestamp = value.get("timestamp") or value.get("time")
                mode = value.get("mode")
            else:
                timestamp = value
                mode = None
            events[str(key)] = {
                "timestamp": str(timestamp) if timestamp is not None else "",
                "mode": normalize_mode_label(mode) if mode is not None else None,
            }
        return events

    if isinstance(loaded, list):
        events = {}
        for entry in loaded:
            if not isinstance(entry, dict):
                continue
            name = entry.get("name") or entry.get("calibration") or entry.get("event")
            timestamp = entry.get("timestamp") or entry.get("time")
            if name and timestamp:
                events[str(name)] = {
                    "timestamp": str(timestamp),
                    "mode": normalize_mode_label(entry.get("mode")) if entry.get("mode") else None,
                }
        return events

    return {}


def save_last_calibrations(events: Dict[str, Dict[str, Optional[str]]], log_dir: Optional[str] = None):
    log_dir = log_dir or resolve_calibration_logs_dir()
    os.makedirs(log_dir, exist_ok=True)
    file_path = os.path.join(log_dir, LAST_CALIBRATIONS_FILE)

    serializable_events = {
        name: event.get("timestamp", "")
        for name, event in events.items()
    }

    with open(file_path, "w") as file:
        yaml.dump(serializable_events, file, default_flow_style=False, sort_keys=False)


def load_json_file(file_path: str) -> Dict[str, Any]:
    with open(file_path, "r") as file:
        loaded = json.load(file)
    return loaded if isinstance(loaded, dict) else {"value": loaded}


def find_json_for_event(name: str, log_dir: Optional[str] = None) -> Optional[str]:
    log_dir = log_dir or resolve_calibration_logs_dir()
    if not os.path.isdir(log_dir):
        return None

    direct_path = os.path.join(log_dir, f"{name}.json")
    if os.path.isfile(direct_path):
        return direct_path

    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_")
    safe_path = os.path.join(log_dir, f"{safe_name}.json")
    if safe_name and os.path.isfile(safe_path):
        return safe_path

    return None


def load_mode_from_json(json_path: Optional[str]) -> Optional[str]:
    if not json_path:
        return None

    try:
        data = load_json_file(json_path)
    except Exception:
        return None

    mode = data.get("mode")
    if mode is None and isinstance(data.get("metadata"), dict):
        mode = data["metadata"].get("mode")

    return normalize_mode_label(mode) if mode else None


def list_calibration_events(
    log_dir: Optional[str] = None,
    mode: str = "Unknown",
) -> List[CalibrationLogEvent]:
    log_dir = log_dir or resolve_calibration_logs_dir()
    last_calibrations = load_last_calibrations(log_dir)
    events = []

    for name, index_entry in last_calibrations.items():
        timestamp_text = index_entry.get("timestamp", "")
        timestamp = parse_timestamp(timestamp_text)
        json_path = find_json_for_event(name, log_dir)
        event_mode = (
            index_entry.get("mode")
            or load_mode_from_json(json_path)
            or normalize_mode_label(mode)
        )
        events.append(
            CalibrationLogEvent(
                name=name,
                mode=event_mode,
                log_dir=log_dir,
                timestamp_text=timestamp_text,
                timestamp=timestamp,
                yaml_available=True,
                json_path=json_path,
                json_available=json_path is not None,
            )
        )

    events.sort(key=lambda event: event.timestamp or datetime.min, reverse=True)
    return events


def list_calibration_events_from_sources(node=None) -> List[CalibrationLogEvent]:
    events = []
    for mode, log_dir in resolve_calibration_log_sources(node):
        events.extend(list_calibration_events(log_dir, mode=mode))
    events.sort(key=lambda event: event.timestamp or datetime.min, reverse=True)
    return events


def compact_json_summary(data: Dict[str, Any]) -> List[str]:
    summary = []

    timestamp = data.get("timestamp")
    if timestamp is None and isinstance(data.get("metadata"), dict):
        timestamp = data["metadata"].get("timestamp")
    if timestamp is not None:
        summary.append(f"Timestamp: {format_timestamp(parse_timestamp(timestamp), timestamp)}")

    for key in (
        "calibration_reference_frame",
        "calibration_fixed_reference_frame",
        "mode",
        "source",
        "active_file",
        "archive_file",
    ):
        if key in data:
            summary.append(f"{key.replace('_', ' ').title()}: {data[key]}")

    if isinstance(data.get("metadata"), dict):
        metadata = data["metadata"]
        for key in ("filename", "calibration_reference_frame", "calibration_fixed_reference_frame"):
            if key in metadata:
                summary.append(f"{key.replace('_', ' ').title()}: {metadata[key]}")

    for key in ("calibration_data", "measurement_data", "calibration_history", "changes"):
        value = data.get(key)
        if isinstance(value, list):
            summary.append(f"{key.replace('_', ' ').title()}: {len(value)} entries")

    if isinstance(data.get("pivots"), dict):
        summary.append(f"Pivots: {len(data['pivots'])}")

    return summary


def append_manual_calibration_log(
    changes: List[Dict[str, Any]],
    mode: str,
    active_file: str,
    archive_file: Optional[str],
    log_dir: Optional[str] = None,
) -> str:
    mode_label = normalize_mode_label(mode)
    if log_dir is None and mode_label == "Real Hardware":
        log_dir = os.path.join(os.path.dirname(os.path.abspath(active_file)), "calibration_logs")
    log_dir = log_dir or resolve_calibration_logs_dir()
    timestamp = datetime.now()
    timestamp_text = timestamp.isoformat()
    mode_slug = "real" if mode_label == "Real Hardware" else "sim"
    event_name = f"manual_joint_calibration_{mode_slug}_{timestamp.strftime('%Y%m%d_%H%M%S_%f')}"
    json_path = os.path.join(log_dir, f"{event_name}.json")

    os.makedirs(log_dir, exist_ok=True)
    payload = {
        "timestamp": timestamp_text,
        "source": "joint_calibration_panel",
        "mode": mode_label,
        "active_file": active_file,
        "archive_file": archive_file,
        "changes": changes,
    }

    with open(json_path, "w") as file:
        json.dump(payload, file, indent=2, sort_keys=False)

    events = load_last_calibrations(log_dir)
    events[event_name] = {
        "timestamp": timestamp_text,
        "mode": mode_label,
    }
    save_last_calibrations(events, log_dir)

    return json_path
