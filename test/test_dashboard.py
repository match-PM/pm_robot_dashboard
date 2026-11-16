"""Exercise optional recorder startup and dashboard close cancellation without ROS spin."""

import builtins
import os
import runpy
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt6.QtWidgets import QApplication, QWidget

from pm_robot_dashboard import pm_robot_dashboard as dashboard


@pytest.mark.parametrize('missing', ['pm_video_recorder', 'recorder_dependency'])
def test_recorder_import_only_tolerates_missing_recorder(monkeypatch, missing):
    original_import = builtins.__import__

    def import_without_recorder(name, *args, **kwargs):
        if name == 'pm_video_recorder.widget':
            raise ModuleNotFoundError(f'No module named {missing!r}', name=missing)
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', import_without_recorder)
    if missing == 'pm_video_recorder':
        namespace = runpy.run_path(dashboard.__file__)
        assert namespace['VideoRecorderWidget'] is None
    else:
        with pytest.raises(ModuleNotFoundError) as error:
            runpy.run_path(dashboard.__file__)
        assert error.value.name == missing


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def make_window(app, monkeypatch):
    class HelperWidget(QWidget):
        def __init__(self, *args, **kwargs):
            super().__init__()
            self.shutdown = Mock(return_value=True)

    for name in (
        'JointsControlWidget', 'NozzleControlWidget', 'PneumaticControlWidget',
        'PmRobotConfigWidget', 'LaunchControlWidget', 'DispenserBuilderWidget',
        'GraphView', 'CalibrationLogWidget', 'JointCalibrationPanel',
        'AssemblyScenceViewerWidget', 'AssemblyModifierWidget', 'TfViewerApp',
        'RsapLogAnalysisWidget',
    ):
        monkeypatch.setattr(dashboard, name, HelperWidget, raising=False)

    windows = []

    def create(recorder_available):
        monkeypatch.setattr(
            dashboard, 'VideoRecorderWidget',
            HelperWidget if recorder_available else None,
        )
        node = Mock()
        window = dashboard.PmJogToolUi(node)
        windows.append(window)
        return window, node

    yield create
    for window in windows:
        if window.video_recorder is not None:
            window.video_recorder.shutdown.return_value = True
        window.close()
        window.deleteLater()
    app.processEvents()


def test_dashboard_without_recorder_starts_and_closes(make_window):
    window, node = make_window(recorder_available=False)
    assert window.video_recorder is None
    node.get_logger().warning.assert_called_once()
    assert window.close()
    window.rsap_log_analysis.shutdown.assert_called_once_with()


@pytest.mark.parametrize('allow_close', [True, False])
def test_recorder_can_accept_or_cancel_dashboard_close(make_window, allow_close):
    window, node = make_window(recorder_available=True)
    window.video_recorder.shutdown.return_value = allow_close
    assert window.close() == allow_close
    window.video_recorder.shutdown.assert_called_once_with(prompt_to_save=True)
    node.get_logger().warning.assert_not_called()
    if allow_close:
        window.rsap_log_analysis.shutdown.assert_called_once_with()
    else:
        window.rsap_log_analysis.shutdown.assert_not_called()
