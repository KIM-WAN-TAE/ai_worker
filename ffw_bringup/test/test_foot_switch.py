from pathlib import Path
from unittest.mock import Mock

import yaml

from ffw_bringup.foot_switch import FootSwitchReader, SAVE_POSE_ID


def make_reader_without_ros_node():
    reader = object.__new__(FootSwitchReader)
    reader.middle_pedal_pressed = False
    reader.middle_pedal_pub = Mock()
    reader.left_enable_pub = Mock()
    reader.right_enable_pub = Mock()
    return reader


def test_middle_pedal_publishes_edges_and_heartbeat_state():
    reader = make_reader_without_ros_node()

    reader.handle_middle(1)
    assert reader.middle_pedal_pressed is True
    assert reader.middle_pedal_pub.publish.call_count == 1
    assert reader.middle_pedal_pub.publish.call_args.args[0].data is True

    # A repeated key-down event does not create another edge publication.
    reader.handle_middle(1)
    assert reader.middle_pedal_pub.publish.call_count == 1

    # The timer callback republishes the current state as a heartbeat.
    reader.publish_middle_pedal_state()
    assert reader.middle_pedal_pub.publish.call_count == 2
    assert reader.middle_pedal_pub.publish.call_args.args[0].data is True

    reader.handle_middle(0)
    assert reader.middle_pedal_pressed is False
    assert reader.middle_pedal_pub.publish.call_count == 3
    assert reader.middle_pedal_pub.publish.call_args.args[0].data is False


def test_left_and_right_pedals_publish_save_pose_id():
    reader = make_reader_without_ros_node()

    reader.handle_left(1)
    reader.handle_right(1)

    assert reader.left_enable_pub.publish.call_args.args[0].data == SAVE_POSE_ID
    assert reader.right_enable_pub.publish.call_args.args[0].data == SAVE_POSE_ID


def test_key_release_does_not_retrigger_save_pose():
    reader = make_reader_without_ros_node()

    reader.handle_left(0)
    reader.handle_right(0)

    reader.left_enable_pub.publish.assert_not_called()
    reader.right_enable_pub.publish.assert_not_called()


def test_a3_disables_navigation_and_auxiliary_joint_jog():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "ffw_a3"
        / "ffw_a3_ai_hardware_controller.yaml"
    )
    config = yaml.safe_load(config_path.read_text())
    params = config["/**"]["joystick_controller"]["ros__parameters"]

    assert params["enable_joystick_update"] is True
    assert params["enable_joint_jog_update"] is False
    assert params["enable_cmd_vel_update"] is False
