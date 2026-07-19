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
    assert '"motion_response_enabled"' in launch
    assert '"motion_response_velocity_topic"' in launch
    assert '"motion_response_min_speed_mps"' in launch


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
    assert '"scan_fft_size", default_value="16384"' in launch
    assert '"scan_fft_hop_size", default_value="8192"' in launch
    assert "96000/16384 = 5.86 Hz bins" in launch
    assert 'gui_rc_handoff_service' in launch
    assert 'gui_rc_restore_service' in launch
    assert 'default_value="/uuv_web_control_gui/suspend_rc_override"' in launch
    assert 'default_value="/uuv_web_control_gui/restore_rc_override"' in launch
    restore_body = launch.split("def _restore_gui_rc_on_shutdown", 1)[1].split(
        "def _start_real_homing_after_selection", 1
    )[0]
    assert 'LaunchConfiguration("gui_rc_restore_service")' in restore_body
    assert 'LaunchConfiguration("gui_rc_handoff_service")' not in restore_body
    assert 'OnShutdown' in launch
    assert '"motion_response_enabled"' in launch
    assert not (ROOT / "scripts" / "start_pinger_homing_real.sh").exists()


def test_motion_response_adapts_timing_without_becoming_phase_bearing_input() -> None:
    controller = (ROOT / "src" / "pinger_homing" / "pinger_homing_controller.cpp").read_text()
    assert "handle_no_odom_motion_response" in controller
    assert "motion_response_velocity_topic" in controller


def test_adaptive_phase_reestimate_uses_innovation_not_fixed_cadence() -> None:
    root = Path(__file__).resolve().parents[1]
    controller = (root / "src" / "pinger_homing" / "pinger_homing_controller.cpp").read_text()
    real_launch = (root / "launch" / "pinger_homing_real.launch.py").read_text()
    tank_launch = (root / "launch" / "pinger_homing_test_tank.launch.py").read_text()
    assert '"no_odom_reestimate_policy", "adaptive"' in controller
    assert "handle_no_odom_phase_innovation" in controller
    assert "handle_no_odom_approach_watchdog" in controller
    assert "no_odom_innovation_limit" in controller
    assert 'DeclareLaunchArgument("reestimate_policy", default_value="adaptive")' in real_launch
    assert 'DeclareLaunchArgument("approach_max_s", default_value="25.0")' in real_launch
    assert 'DeclareLaunchArgument("reestimate_policy", default_value="adaptive")' in tank_launch
    assert "no_odom_probe_leg_extensions_s_" in controller
    assert "uses_for_bearing\\\":false" in controller
    assert "never enter the Phase ABBA regression" in controller


if __name__ == "__main__":
    test_real_launch_uses_canonical_cpp_no_odom_phase_controller()
    test_real_launch_preserves_external_hydrophone_estimator_boundary()
    test_xy_alt_hold_does_not_require_depth_for_a_heave_free_probe()
    test_interactive_launch_scans_then_injects_selected_startup_frequency()
    test_motion_response_adapts_timing_without_becoming_phase_bearing_input()
    test_adaptive_phase_reestimate_uses_innovation_not_fixed_cadence()
