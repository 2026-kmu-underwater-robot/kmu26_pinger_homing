#!/usr/bin/env python3
"""Keep the physical launch on the canonical C++ Phase path.

The simulator success path and the physical launch must not silently diverge
back to the archived Python controller.  This is a source-level contract test:
hardware is intentionally not touched.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_real_launch_uses_canonical_cpp_no_odom_phase_controller() -> None:
    launch = (ROOT / "launch" / "pinger_homing_real.launch.py").read_text()
    assert 'executable="pinger_homing_controller"' in launch
    assert 'executable="single_hydrophone_homing_controller.py"' not in launch
    assert '"navigation_mode": "no_odom_phase"' in launch
    assert '"acoustic_estimator_mode": "phase"' in launch
    assert '"no_odom_horizontal_only": True' in launch
    assert '"no_odom_vertical_control_enabled": False' in launch


def test_real_launch_preserves_external_hydrophone_estimator_boundary() -> None:
    launch = (ROOT / "launch" / "pinger_homing_real.launch.py").read_text()
    assert 'package="audio_capture"' in launch
    assert 'executable="audio_phase_estimator"' in launch
    assert '"enable_frequency_acquisition": False' in launch
    assert '"use_sim_time"' in launch
    assert '"audio_input_latency_s"' in launch


def test_xy_alt_hold_does_not_require_depth_for_a_heave_free_probe() -> None:
    controller = (ROOT / "src" / "pinger_homing" / "pinger_homing_controller.cpp").read_text()
    assert "passive_vertical_alt_hold" in controller
    assert "no_odom_horizontal_only_ && !no_odom_vertical_control_enabled_" in controller


def test_interactive_launch_scans_then_injects_selected_startup_frequency() -> None:
    launch = (ROOT / "launch" / "pinger_homing_real_interactive.launch.py").read_text()
    assert 'executable="pinger_frequency_selector"' in launch
    assert "OpaqueFunction(function=_start_real_homing_after_selection)" in launch
    assert '"selected_frequency_topic"' in launch
    assert 'launch_arguments["reference_frequency_hz"]' in launch
    assert 'launch_arguments["use_audio_capture"] = "false"' in launch
    assert 'gui_rc_handoff' in launch
    assert 'OnShutdown' in launch
    assert not (ROOT / "scripts" / "start_pinger_homing_real.sh").exists()
