from pathlib import Path

from drone_arm_sim.cartesian_velocity_keyboard import (
    COMMAND_TOPIC,
    command_for_key,
    run_key_loop,
    wait_for_subscriber,
)


ROOT = Path(__file__).resolve().parents[3]


def test_keyboard_mapping_is_body_flu_and_case_insensitive():
    assert command_for_key("i") == "forward"
    assert command_for_key("K") == "back"
    assert command_for_key("j") == "left"
    assert command_for_key("L") == "right"
    assert command_for_key("u") == "up"
    assert command_for_key("O") == "down"
    assert command_for_key(" ") == "stop"
    assert command_for_key("x") == "disable"
    assert command_for_key("?") is None
    assert COMMAND_TOPIC == "/my_drone/arm_cartesian_velocity_command"


def test_one_key_loop_reuses_sender_and_x_disables_immediately(capsys):
    keys = iter("i?KjLuO xignored")
    published = []

    run_key_loop(lambda: next(keys, ""), published.append, lambda: True)

    assert published == [
        "forward",
        "back",
        "left",
        "right",
        "up",
        "down",
        "stop",
        "disable",
    ]
    assert "ignored" not in capsys.readouterr().out


def test_publisher_waits_for_subscription_before_keyboard_input():
    class Publisher:
        checks = 0

        def get_subscription_count(self):
            self.checks += 1
            return int(self.checks >= 3)

    class RosApi:
        spins = 0

        @staticmethod
        def ok():
            return True

        @classmethod
        def spin_once(cls, node, timeout_sec):
            assert node is sentinel
            assert timeout_sec == 0.1
            cls.spins += 1

    sentinel = object()
    publisher = Publisher()
    wait_for_subscriber(sentinel, publisher, RosApi, timeout_s=1.0)
    assert RosApi.spins == 2


def test_shell_delegates_velocity_mode_to_persistent_console_node():
    shell = (ROOT / "scripts/run_ros2_arm_keyboard.sh").read_text(
        encoding="utf-8"
    )
    velocity_mode = shell[
        shell.index("run_cartesian_velocity_mode()") : shell.index(
            'echo "SO101 keyboard controller"'
        )
    ]
    assert "ros2 run drone_arm_sim cartesian_arm_velocity_keyboard" in velocity_mode
    assert "ros2 topic pub --once" not in velocity_mode
    assert "send_cartesian_velocity_command" not in shell


def test_console_script_is_registered():
    setup = (ROOT / "src/drone_arm_sim/setup.py").read_text(encoding="utf-8")
    assert (
        "cartesian_arm_velocity_keyboard = "
        "drone_arm_sim.cartesian_velocity_keyboard:main"
    ) in setup
