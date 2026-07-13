"""CLI contract tests for the deterministic offline backend runner wrapper."""

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_offline_determinism_check.sh'


def test_param_override_requires_ros_assignment_syntax():
    result = subprocess.run(
        ['bash', str(SCRIPT), '--param', 'refine_window_size=32'],
        cwd=ROOT, capture_output=True, text=True, check=False)

    assert result.returncode == 2
    assert '--param expects name:=value' in result.stderr


def test_param_override_is_forwarded_after_params_file():
    source = SCRIPT.read_text()

    assert 'PARAM_OVERRIDES+=("$2")' in source
    assert 'RUNNER_CMD+=(-p "${override}")' in source
    assert source.index('RUNNER_CMD+=(-p "${override}")') > source.index(
        '--params-file "${PARAMS}"')


def test_runs_are_dds_isolated_and_resume_requires_completion_marker():
    source = SCRIPT.read_text()

    assert '--disable-rosout-logs' in source
    assert 'ROS_DOMAIN_ID=' in source
    assert 'ROS_LOCALHOST_ONLY=1' in source
    assert '--ros-domain-base' in source
    assert '--resume' in source
    assert '${run_dir}/.complete' in source
