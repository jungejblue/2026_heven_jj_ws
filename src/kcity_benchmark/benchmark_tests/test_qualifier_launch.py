"""Exercise real launch process-exit gates without starting ROS/CARLA nodes."""
import sys

import pytest
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess

from kcity_benchmark.qualifier_launch import after, validate_mode


@pytest.mark.parametrize("exit_code,expected", [(0, True), (1, False)])
def test_gate_starts_next_process_only_on_success(tmp_path, exit_code, expected):
    marker = tmp_path / "started"
    gate = ExecuteProcess(cmd=[sys.executable, "-c", f"raise SystemExit({exit_code})"])
    child = ExecuteProcess(cmd=[sys.executable, "-c",
        "from pathlib import Path; import sys; Path(sys.argv[1]).touch()", str(marker)])
    service = LaunchService()
    service.include_launch_description(LaunchDescription([after(gate, [child], "test"), gate]))
    service.run()
    assert marker.exists() == expected


def test_unavailable_mode_fails_before_any_controller_is_created():
    validate_mode("route_test")
    with pytest.raises(RuntimeError, match="heven_autonomy.launch.py"):
        validate_mode("jj_autonomy")
    with pytest.raises(RuntimeError, match="Unsupported"):
        validate_mode("manual")
