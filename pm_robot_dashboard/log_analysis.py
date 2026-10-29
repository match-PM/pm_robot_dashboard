"""Parse RSAP logs and calculate duration statistics without GUI or ROS dependencies."""

from dataclasses import dataclass
import math
import re
from statistics import fmean, pstdev


def parse_duration(value):
    """Return seconds from a timedelta string or a numeric duration."""
    if isinstance(value, bool):
        raise ValueError('Boolean duration')
    if isinstance(value, (int, float)):
        seconds = float(value)
    elif isinstance(value, str):
        match = re.fullmatch(
            r'\s*(?:(\d+) days?,\s*)?(\d+):(\d{2}):(\d{2}(?:\.\d+)?)\s*', value,
        )
        if not match:
            raise ValueError(f'Invalid duration: {value!r}')
        days, hours, minutes, seconds_part = match.groups()
        if int(minutes) >= 60 or float(seconds_part) >= 60:
            raise ValueError(f'Invalid duration: {value!r}')
        seconds = (int(days or 0) * 86400 + int(hours) * 3600
                   + int(minutes) * 60 + float(seconds_part))
    else:
        raise ValueError('Missing execution_time')
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError('Duration must be finite and nonnegative')
    return seconds


@dataclass
class LogEntry:
    key: tuple
    source: str
    index: int
    name: str
    uuid: str
    success: object
    duration: object
    log: dict
    issue: str = ''


def make_entry(source, identity, index, name, log, uuid=None, success=None):
    """Keep invalid durations visible so the UI can explain excluded entries."""
    try:
        duration = parse_duration(log.get('execution_time'))
        issue = ''
    except ValueError as exc:
        duration, issue = None, str(exc)
    result = log.get('success') if success is None else success
    result = result if isinstance(result, bool) else None
    return LogEntry(
        (source, identity), source, index, name, str(uuid or log.get('uuid', '')),
        result, duration, log, issue,
    )


def parse_file_data(data, source):
    """Read exported sequence logs or the earlier JSON-list topic representation."""
    records = data.get('action_log') if isinstance(data, dict) else data
    if not isinstance(records, list):
        raise ValueError('Expected an action_log list or a JSON list of action logs')
    entries, warnings = [], []
    for position, record in enumerate(records):
        if not isinstance(record, dict) or len(record) != 1:
            warnings.append(f'Entry {position}: expected one action name and log')
            continue
        raw_name, log = next(iter(record.items()))
        if not isinstance(log, dict):
            warnings.append(f'{raw_name}: expected a log object')
            continue
        match = re.match(r'^(\d+)_(.*)$', raw_name)
        index = int(match[1]) if match else position
        name = match[2].strip() if match else raw_name.strip()
        entries.append(make_entry(source, position, index, name, log))
    return entries, warnings


def statistics(values):
    """Use population standard deviation, matching the original analysis script."""
    return dict(count=len(values), mean=fmean(values), std=pstdev(values),
                min=min(values), max=max(values), total=sum(values))


def assess(entries, selected, group_by_index=False):
    """Aggregate only selected entries with valid durations; all values are seconds."""
    groups, totals = {}, {}
    for entry in entries:
        if entry.key not in selected or entry.duration is None:
            continue
        label = f'{entry.index}_{entry.name}' if group_by_index else entry.name
        groups.setdefault(label, []).append(entry.duration)
        totals[entry.source] = totals.get(entry.source, 0.0) + entry.duration
    return groups, {name: statistics(values) for name, values in groups.items()}, totals
