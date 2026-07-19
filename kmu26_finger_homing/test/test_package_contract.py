from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_standalone_package_has_no_hydrophone_repos_dependency():
    text = (ROOT / "package.xml").read_text()
    assert "hydrophone.repos" not in text
    assert "kmu26_auv_hydrophone" not in text


def test_alt_hold_and_private_rc_defaults():
    text = (ROOT / "src" / "finger_homing_controller.cpp").read_text()
    assert 'declare_parameter<std::string>("mode", "ALT_HOLD")' in text
    assert '"/control/finger_homing/rc_override"' in text
    assert 'msg.channels[2] = 1500' in text


def test_frequency_selection_contract():
    selector = (ROOT / "src" / "finger_frequency_selector.cpp").read_text()
    assert "monitor_s_" in selector
    assert "values.size() > 5U" in selector
    assert "frequency candidates" in selector


def test_xy_only_controller():
    text = (ROOT / "src" / "finger_homing_controller.cpp").read_text()
    assert "direction_world_->y()" in text
    assert "direction.vector.z = 0.0" in text
    assert "depth" not in text.lower()
