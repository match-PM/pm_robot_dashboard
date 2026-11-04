"""Check service requests against the bringup tool catalog without launching ROS."""

import copy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from pm_msgs.srv import ChangeToolAndRestart
from pm_robot_modules.submodules.pm_robot_config import PmRobotConfig, PmRobotToolsConfig

from pm_robot_dashboard import pm_robot_tool_changer as tool_changer


BRINGUP_CONFIG = (
    Path(get_package_share_directory('pm_robot_bringup'))
    / 'config/pm_robot_bringup_config.yaml'
)
DESCRIPTION_CONFIGS = (
    Path(get_package_share_directory('pm_robot_description')) / 'urdf/urdf_configs'
)
GRIPPERS = [
    ('pm_robot_vacuum_tools', '_gripper_vacuum', 'schunk_vacuum_tools.yaml', 'use_tip'),
    ('pm_robot_tool_parallel_gripper_1_jaw', '_gripper_1_jaw',
     'parallel_gripper_1_jaw.yaml', 'use_jaw_type'),
    ('pm_robot_tool_parallel_gripper_2_jaws', '_gripper_2_jaw',
     'parallel_gripper_2_jaw.yaml', 'use_jaw_type'),
]


def load_config():
    return yaml.safe_load(BRINGUP_CONFIG.read_text())


def configured_pairs():
    tools = PmRobotToolsConfig(load_config())
    return [
        (key, attr, definitions, field, name, attachment)
        for key, attr, definitions, field in GRIPPERS
        for name, attachments in getattr(tools, attr).get_available_tools()
        for attachment in attachments
    ]


@pytest.fixture
def service(monkeypatch):
    config_data = load_config()
    config = SimpleNamespace(
        tool=PmRobotToolsConfig(config_data), save_config=Mock(),
        set_real_HW=Mock(),
        get_active_bringup_config_path=Mock(return_value="/tmp/sim_config.yaml"),
    )
    instance = tool_changer.ToolChangerService.__new__(tool_changer.ToolChangerService)
    instance._node = Mock()
    instance._pm_robot_utils = SimpleNamespace(
        pm_robot_config=config, get_mode=Mock(return_value=2),
        REAL_MODE=0, UNITY_MODE=1, GAZEBO_MODE=2,
    )
    instance._trigger_unity_configure = Mock()
    monkeypatch.setattr(tool_changer.time, 'sleep', Mock())
    monkeypatch.setattr(tool_changer.subprocess, 'Popen', Mock())
    monkeypatch.setattr(tool_changer, '_read_pid', Mock(return_value=None))
    monkeypatch.setattr(tool_changer, '_kill_pid', Mock())
    return instance, config, config_data


def call(service, tool, attachment, slug='sim_hw'):
    request = ChangeToolAndRestart.Request(
        tool_name=tool, attachment_name=attachment,
        launch_slug=slug, restart_delay=0.1,
    )
    return service._change_tool_and_restart_callback(
        request, ChangeToolAndRestart.Response(),
    )


@pytest.mark.parametrize(
    'key,attr,definitions,field,tool,attachment', configured_pairs(),
)
def test_bringup_pairs_are_supported_and_saved(
    service, key, attr, definitions, field, tool, attachment,
):
    # These are the dictionaries indexed by bringup's tool/jaw/tip xacros.
    description = yaml.safe_load((DESCRIPTION_CONFIGS / definitions).read_text())
    assert isinstance(description[tool][attachment], dict)
    instance, config, config_data = service
    response = call(instance, f' {tool} ', f' {attachment} ')
    assert response.success, response.message
    selected = getattr(config.tool, attr)
    assert selected.get_current_tool() == tool
    assert selected.get_current_tool_attachment() == attachment
    assert config_data['pm_robot_tools'][key]['use_tool'] == tool
    assert config_data['pm_robot_tools'][key][field] == attachment
    assert sum(
        getattr(config.tool, gripper_attr).get_activate_status()
        for _, gripper_attr, _, _ in GRIPPERS
    ) == 1
    assert selected.get_activate_status()
    config.save_config.assert_called_once()
    instance._trigger_unity_configure.assert_called_once()
    tool_changer.subprocess.Popen.assert_called_once()


@pytest.mark.parametrize('tool,attachment,slug', [
    ('Siemens_Vacuum_Array_short', '', 'sim_hw'),
    ('Siemens_Vacuum_Array_short', '   ', 'sim_hw'),
    ('Siemens_Vacuum_Array_short', 'Sensing_Jaw', 'sim_hw'),
    ('Schunk_MPG_10_plus', 'Jaw_ASG_Gripper', 'sim_hw'),
    ('SmarAct_Piezo_Gripper', 'Sensing_Jaw', 'sim_hw'),
    ('nonexistent_tool', 'Tip_Siemens_Vacuum_Array', 'sim_hw'),
    ('', 'Tip_Siemens_Vacuum_Array', 'sim_hw'),
    ('Siemens_Vacuum_Array_short', 'Tip_Siemens_Vacuum_Array', 'invalid_slug'),
    ('Siemens_Vacuum_Array_short', 'Tip_Siemens_Vacuum_Array', ''),
])
def test_invalid_request_has_no_side_effects(service, tool, attachment, slug):
    instance, config, config_data = service
    original = copy.deepcopy(config_data)
    response = call(instance, tool, attachment, slug)
    assert not response.success
    assert response.message
    assert config_data == original
    config.save_config.assert_not_called()
    instance._trigger_unity_configure.assert_not_called()
    tool_changer._kill_pid.assert_not_called()
    tool_changer.time.sleep.assert_not_called()
    tool_changer.subprocess.Popen.assert_not_called()


@pytest.mark.parametrize('mode,slug', [(0, 'real_hw'), (1, 'unity_hw'), (2, 'sim_hw')])
def test_request_reloads_and_saves_only_active_config(service, tmp_path, mode, slug):
    instance, _, _ = service
    sim_path = tmp_path / 'sim.yaml'
    real_path = tmp_path / 'real.yaml'
    original = BRINGUP_CONFIG.read_text()
    sim_path.write_text(original)
    real_path.write_text(original)
    config = PmRobotConfig.__new__(PmRobotConfig)
    config.sim_bringup_config_path = str(sim_path)
    config._real_bringup_config_path = str(real_path)
    config._real_config_available = True
    # Start with the wrong config selected, as can happen through the GUI.
    config._active_path = str(sim_path if mode == 0 else real_path)
    config.reload_config()
    utils = instance._pm_robot_utils
    utils.pm_robot_config = config
    utils.get_mode.return_value = mode

    response = call(instance, 'Siemens_Vacuum_Array_angled',
                    'Tip_Siemens_Vacuum_Array', slug)

    assert response.success, response.message
    active_path = real_path if mode == 0 else sim_path
    inactive_path = sim_path if mode == 0 else real_path
    saved = yaml.safe_load(active_path.read_text())
    vacuum = saved['pm_robot_tools']['pm_robot_vacuum_tools']
    assert vacuum['use_tool'] == 'Siemens_Vacuum_Array_angled'
    assert vacuum['use_tip'] == 'Tip_Siemens_Vacuum_Array'
    assert inactive_path.read_text() == original
    assert str(active_path) in response.message


def test_launch_must_match_active_mode(service):
    instance, config, _ = service
    instance._pm_robot_utils.get_mode.return_value = 0
    response = call(instance, 'Siemens_Vacuum_Array_angled',
                    'Tip_Siemens_Vacuum_Array', 'sim_hw')
    assert not response.success
    assert 'active robot mode' in response.message
    config.set_real_HW.assert_not_called()
    config.save_config.assert_not_called()
    tool_changer.subprocess.Popen.assert_not_called()


def test_mode_is_checked_again_for_each_request(service):
    instance, config, _ = service
    for mode, slug in [(0, 'real_hw'), (1, 'unity_hw'), (2, 'sim_hw')]:
        instance._pm_robot_utils.get_mode.return_value = mode
        response = call(instance, 'Siemens_Vacuum_Array_angled',
                        'Tip_Siemens_Vacuum_Array', slug)
        assert response.success, response.message
        config.set_real_HW.assert_called_with(mode == 0)


@pytest.mark.parametrize('mode,slug', [(0, 'real_hw'), (1, 'unity_hw'), (2, 'sim_hw')])
def test_auto_detect_restarts_launch_matching_active_mode(service, monkeypatch, mode, slug):
    instance, config, _ = service
    instance._pm_robot_utils.get_mode.return_value = mode
    # Even if multiple PID files exist, pick the launch for the current mode.
    monkeypatch.setattr(tool_changer, '_read_pid', Mock(return_value=123))
    monkeypatch.setattr(tool_changer, '_pid_alive', Mock(return_value=True))
    monkeypatch.setattr(tool_changer.os, 'remove', Mock())
    response = call(instance, 'Siemens_Vacuum_Array_angled',
                    'Tip_Siemens_Vacuum_Array', '')
    assert response.success, response.message
    config.set_real_HW.assert_called_once_with(mode == 0)
    command = tool_changer.subprocess.Popen.call_args.args[0][-1]
    assert tool_changer._LAUNCH_CONFIGS[slug]['launch_file'] in command
