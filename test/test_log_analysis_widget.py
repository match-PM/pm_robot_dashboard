"""Exercise Qt selections, chart updates, file ingestion, and live snapshot lifecycle."""

import json
import os
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/rsap-test-matplotlib')

import pytest
from PyQt6.QtCore import Qt, QMimeData, QUrl, QPointF
from PyQt6.QtGui import QDropEvent
from PyQt6.QtWidgets import QApplication
from PyQt6.QtTest import QTest

from pm_robot_dashboard.log_analysis_widget import RsapLogAnalysisWidget


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def widget(app, tmp_path):
    instance = RsapLogAnalysisWidget()
    path = tmp_path / 'run.json'
    path.write_text(json.dumps({'action_log': [
        {'0_Move': {'execution_time': '0:00:02', 'success': True}},
        {'1_Move': {'execution_time': '0:00:04', 'success': False}},
        {'2_Missing': {'success': False}},
    ]}))
    instance.load_paths([str(path)])
    yield instance
    instance.shutdown()
    instance.close()


def test_file_deduplication_selection_and_invalid_duration(widget):
    widget.load_paths(list(widget.file_paths))
    assert len(widget.entries[0]) == 3
    assert len(widget.selected[0]) == 2
    widget.render_analysis()
    assert widget.assessment()[1]['Move']['mean'] == 3
    root = widget.tree.topLevelItem(0)
    assert root.checkState(0) == Qt.CheckState.Checked
    root.child(1).setCheckState(0, Qt.CheckState.Unchecked)
    widget.render_analysis()
    assert widget.assessment()[1]['Move']['mean'] == 2
    root.setCheckState(0, Qt.CheckState.Checked)
    assert len(widget.selected[0]) == 2
    root.setCheckState(0, Qt.CheckState.Unchecked)
    assert widget.assessment() == ({}, {}, {})


def test_filters_and_search_leave_statistics_consistent(widget):
    widget.select_entries('failed')
    assert widget.assessment()[1]['Move']['count'] == 1
    widget.select_entries('all')
    widget.search.setText('no match')
    assert widget.tree.topLevelItem(0).isHidden()
    assert widget.assessment()[1]['Move']['count'] == 2
    widget.select_entries('none')
    assert widget.assessment() == ({}, {}, {})
    widget.select_entries('invert')
    assert len(widget.selected[0]) == 2
    widget.select_entries('successful')
    assert widget.assessment()[1]['Move']['mean'] == 2


def test_checkbox_changes_refresh_statistics_automatically(widget):
    widget.render_analysis()
    assert widget.stats_table.item(0, 1).text() == '2'
    widget.tree.topLevelItem(0).child(1).setCheckState(0, Qt.CheckState.Unchecked)
    QTest.qWait(100)
    assert widget.stats_table.item(0, 1).text() == '1'
    assert widget.stats_table.item(0, 2).text() == '2.000'


def test_every_chart_and_empty_selection_render(widget, app):
    widget.resize(1300, 850)
    widget.show()
    app.processEvents()
    for index in range(4):
        widget.chart_type.setCurrentIndex(index)
        widget.render_analysis()
        widget.canvas.draw()
        assert len(widget.figure.axes) == 1
    widget.units.setCurrentIndex(1)
    widget.axis_max.setValue(10)
    widget.render_analysis()
    assert widget.figure.axes[0].get_xlim()[1] == 10
    widget.select_entries('none')
    widget.render_analysis()
    widget.canvas.draw()
    assert widget.stats_table.rowCount() == 0


def test_drop_file_and_tab_selections_are_independent(widget, tmp_path):
    path = tmp_path / 'second.json'
    path.write_text('[{"0_Inspect": {"execution_time": "0:00:01"}}]')
    mime = QMimeData()
    mime.setUrls([QUrl.fromLocalFile(str(path))])
    event = QDropEvent(QPointF(10, 10), Qt.DropAction.CopyAction, mime,
                       Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier)
    widget.dropEvent(event)
    assert event.isAccepted()
    assert len(widget.entries[0]) == 4
    selected = widget.selected[0].copy()
    widget.source_tabs.setCurrentIndex(1)
    assert widget.tree.topLevelItemCount() == 0
    widget.select_entries('none')
    widget.source_tabs.setCurrentIndex(0)
    assert widget.selected[0] == selected


def snapshot(duration='0:00:03'):
    return SimpleNamespace(
        sequence_name='Sequence', stamp=SimpleNamespace(sec=10, nanosec=0),
        logs=[SimpleNamespace(action_name='Move', action_uuid='id', action_index=0,
                              success=True, log_json=json.dumps({'execution_time': duration}))])


def test_live_snapshots_replace_entries_and_preserve_deselection(widget):
    widget._topic_source = '/rsap/rsap_logs'
    widget.source_tabs.setCurrentIndex(1)
    widget._pending = snapshot()
    widget._consume_snapshot()
    assert len(widget.entries[1]) == 1
    widget.select_entries('none')
    widget._pending = snapshot('0:00:05')
    widget._consume_snapshot()
    assert len(widget.entries[1]) == 1
    assert widget.entries[1][0].duration == 5
    assert widget.selected[1] == set()
    widget._pending = SimpleNamespace(sequence_name='', stamp=SimpleNamespace(sec=11, nanosec=0), logs=[])
    widget._consume_snapshot()
    assert widget.entries[1] == []


def test_ros_callback_is_bounded_and_ignores_disconnected_generation(widget):
    pytest.importorskip('rsap_interfaces.msg').RsapLogs
    node = Mock()
    widget.node = node
    widget.connect_topic()
    callback = node.create_subscription.call_args.args[2]
    callback(snapshot('0:00:01'))
    callback(snapshot('0:00:02'))
    assert widget.entries[1] == []  # The executor callback never touches Qt.
    widget._consume_snapshot()
    assert len(widget.entries[1]) == 1
    assert widget.entries[1][0].duration == 2
    widget.disconnect_topic()
    node.destroy_subscription.assert_called_once()
    callback(snapshot('0:00:09'))
    assert widget._pending is None


def test_export_uses_selection_and_full_precision(widget, tmp_path, monkeypatch):
    path = tmp_path / 'stats.json'
    save_dialog = Mock(return_value=(str(path), 'JSON (*.json)'))
    monkeypatch.setattr('PyQt6.QtWidgets.QFileDialog.getSaveFileName',
                        save_dialog)
    widget.select_entries('successful')
    widget.export_statistics()
    assert save_dialog.call_args.args[2] == str(tmp_path / 'rsap_duration_stats.json')
    output = json.loads(path.read_text())
    assert output['per_process_stats']['Move']['total'] == 2
    assert list(output['total_duration_per_file'].values()) == [2]
    assert output['duration_unit'] == 'seconds'
