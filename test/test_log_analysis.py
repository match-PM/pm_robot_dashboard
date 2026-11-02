"""Duration parsing and aggregation by action identity."""

import pytest

from pm_robot_dashboard.log_analysis import assess, parse_duration, parse_file_data


@pytest.mark.parametrize('value, expected', [
    ('0:00:03.125', 3.125), ('1:02:03', 3723),
    ('2 days, 1:00:00', 176400), (0, 0), (2.5, 2.5),
])
def test_parse_duration(value, expected):
    assert parse_duration(value) == expected


@pytest.mark.parametrize('value', [None, True, -1, float('nan'), float('inf'),
                                        '-1:00:00', '1:60:00', '0:00:61', 'bad'])
def test_invalid_duration(value):
    with pytest.raises(ValueError):
        parse_duration(value)


def test_grouping_repeated_actions_and_selected_totals():
    entries, warnings = parse_file_data({'action_log': [
        {'0_Move': {'uuid': 'move-a', 'execution_time': '0:00:02', 'success': True}},
        {'2_Move': {'uuid': 'move-b', 'execution_time': '0:00:04', 'success': False}},
        {'3_Wait': {'execution_time': '0:00:10'}},
    ]}, 'run1')
    second_run, _ = parse_file_data([
        {'0_Move': {'uuid': 'move-a', 'execution_time': '0:00:06'}},
    ], 'run2')
    entries += second_run
    assert warnings == []
    selection = {entry.key for entry in entries if entry.name == 'Move'}
    _, stats, totals = assess(entries, selection)
    assert stats['Move [move-a]'] == dict(count=2, mean=4, std=2,
                                          min=2, max=6, total=8)
    assert stats['Move [move-b]'] == dict(count=1, mean=4, std=0,
                                          min=4, max=4, total=4)
    assert totals == {'run1': 6, 'run2': 6}
    _, indexed, _ = assess(entries, selection, group_by_index=True)
    assert set(indexed) == {'0_Move [move-a]', '2_Move [move-b]'}


def test_bad_entries_are_reported_and_bad_durations_remain_inspectable():
    entries, warnings = parse_file_data({'action_log': [
        {}, {'0_Bad': []}, {'1_Missing': {'success': False}},
    ]}, 'run')
    assert len(warnings) == 2
    assert len(entries) == 1
    assert entries[0].duration is None
    assert entries[0].issue
    assert assess(entries, {entries[0].key}) == ({}, {}, {})


@pytest.mark.parametrize('data', [{}, {'action_log': {}}, None])
def test_reject_non_log_files(data):
    with pytest.raises(ValueError):
        parse_file_data(data, 'run')


def test_missing_action_ids_are_not_combined():
    entries, _ = parse_file_data([
        {'0_Move': {'execution_time': 2}},
        {'0_Move': {'execution_time': 6}},
    ], 'run')
    _, stats, totals = assess(entries, {entry.key for entry in entries})
    assert len(stats) == 2
    assert all(value['count'] == 1 and value['std'] == 0 for value in stats.values())
    assert totals == {'run': 8}


def test_same_uuid_combines_despite_renaming_or_different_execution_ids():
    entries, _ = parse_file_data([
        {'0_Move': {'uuid': 'move', 'execution_id': 'first', 'execution_time': 2}},
        {'1_Renamed': {'uuid': 'move', 'execution_id': 'second', 'execution_time': 6}},
    ], 'run')
    _, stats, _ = assess(entries, {entry.key for entry in entries})
    assert stats['Move [move]']['count'] == 2
    assert stats['Move [move]']['std'] == 2
