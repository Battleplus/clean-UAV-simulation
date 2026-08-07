from px4_ros2_control.frames import enu_to_ned, flu_to_frd, frd_to_flu, ned_to_enu


def test_ned_enu_known_axes_and_round_trip():
    assert ned_to_enu((1.0, 2.0, 3.0)) == (2.0, 1.0, -3.0)
    assert enu_to_ned(ned_to_enu((1.5, -2.5, 3.5))) == (1.5, -2.5, 3.5)


def test_frd_flu_known_axes_and_round_trip():
    assert frd_to_flu((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)
    assert flu_to_frd(frd_to_flu((1.5, -2.5, 3.5))) == (1.5, -2.5, 3.5)
