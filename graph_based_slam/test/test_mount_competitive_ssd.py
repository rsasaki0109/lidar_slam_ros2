#!/usr/bin/env python3
"""Mock-PATH tests for the profile-bound competitive SSD mount helper."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess

import pytest


REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / 'scripts' / 'mount_competitive_ssd.sh'
TARGET = '/media/sasaki/aiueo1'
UUID = '3b5dc9b7-c4de-4cf2-a892-00b2c063f34e'
UUID_PATH = '/dev/disk/by-uuid/' + UUID
DEVICE = '/dev/sda1'
TARGET_RECORD = '{} {} ext4 rw,nosuid,nodev,relatime {}\n'.format(
    TARGET, DEVICE, UUID
)


MOCK_COMMAND = r'''#!/usr/bin/env python3
from pathlib import Path
import os
import sys


TARGET = "/media/sasaki/aiueo1"
UUID_PATH = "/dev/disk/by-uuid/3b5dc9b7-c4de-4cf2-a892-00b2c063f34e"
DEVICE = "/dev/sda1"
UUID = "3b5dc9b7-c4de-4cf2-a892-00b2c063f34e"
state_path = Path(os.environ["MOCK_STATE"])
log_path = Path(os.environ["MOCK_LOG"])
name = Path(sys.argv[0]).name


def state():
    return state_path.read_text(encoding="ascii").strip()


def log():
    with log_path.open("a", encoding="utf-8") as stream:
        stream.write(name + " " + " ".join(sys.argv[1:]) + "\n")


if name == "stat":
    path = sys.argv[-1]
    if path == UUID_PATH:
        print("symbolic link")
    elif path == DEVICE:
        print("block special file")
    elif path == TARGET and os.environ.get("MOCK_UNSAFE_TARGET") == "1":
        print("symbolic link")
    else:
        print("directory")
elif name == "readlink":
    path = sys.argv[-1]
    if path == UUID_PATH:
        print(DEVICE)
    elif path == DEVICE:
        print(DEVICE)
    else:
        print(path)
elif name == "lsblk":
    uuid = "0" * 8 + "-0000-0000-0000-000000000000"
    if os.environ.get("MOCK_WRONG_UUID") != "1":
        uuid = UUID
    print(f"{DEVICE} part ext4 {uuid}")
elif name == "find":
    if os.environ.get("MOCK_UNSAFE_TARGET") == "1":
        print(TARGET + "/unsafe-file")
elif name == "findmnt":
    args = sys.argv[1:]
    mounted = state()
    if "-M" in args:
        if mounted == "target":
            print(f"{TARGET} {DEVICE} ext4 rw,nosuid,nodev,relatime {UUID}")
        else:
            sys.exit(1)
    elif "-S" in args:
        if mounted == "target":
            print(f"{TARGET} {DEVICE} ext4 rw,nosuid,nodev,relatime {UUID}")
        elif mounted == "other":
            print(f"/media/sasaki/aiueo2 {DEVICE} ext4 rw,nosuid,nodev,relatime {UUID}")
        elif mounted == "other_desktop_options":
            print(f"/media/sasaki/aiueo_ssd {DEVICE} ext4 rw,relatime {UUID}")
        else:
            sys.exit(1)
    else:
        sys.exit(2)
elif name == "udisksctl":
    log()
    if os.environ.get("MOCK_UNMOUNT_FAIL") == "1":
        sys.exit(1)
    state_path.write_text("none\n", encoding="ascii")
elif name == "sudo":
    log()
    if os.environ.get("MOCK_MOUNT_FAIL") == "1":
        sys.exit(1)
    state_path.write_text("target\n", encoding="ascii")
else:
    sys.exit(127)
'''


@pytest.fixture()
def mock_env(tmp_path):
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    for command in ('stat', 'readlink', 'lsblk', 'find', 'findmnt', 'udisksctl', 'sudo'):
        path = bindir / command
        path.write_text(MOCK_COMMAND, encoding='utf-8')
        path.chmod(0o755)
    state = tmp_path / 'state'
    state.write_text('none\n', encoding='ascii')
    log = tmp_path / 'commands.log'
    log.write_text('', encoding='ascii')
    env = os.environ.copy()
    real_path = env.get('PATH', '/usr/bin:/bin')
    env.update({
        'PATH': str(bindir) + os.pathsep + real_path,
        'MOCK_STATE': str(state),
        'MOCK_LOG': str(log),
        'PYTHONDONTWRITEBYTECODE': '1',
    })
    return env, state, log


def _run(env, *args):
    return subprocess.run(
        ['bash', str(SCRIPT), *args],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def test_already_correct_mount_is_idempotent_and_check_is_read_only(mock_env):
    env, state, log = mock_env
    state.write_text('target\n', encoding='ascii')

    applied = _run(env)
    checked = _run(env, '--check')

    assert applied.returncode == 0, applied.stderr
    assert checked.returncode == 0, checked.stderr
    assert applied.stdout == TARGET_RECORD
    assert checked.stdout == TARGET_RECORD
    assert log.read_text(encoding='utf-8') == ''


def test_check_rejects_other_mount_without_changing_state(mock_env):
    env, state, log = mock_env
    state.write_text('other\n', encoding='ascii')

    result = _run(env, '--check')

    assert result.returncode != 0
    assert 'read-only' in result.stderr
    assert state.read_text(encoding='ascii').strip() == 'other'
    assert log.read_text(encoding='utf-8') == ''


def test_apply_moves_other_mount_then_verifies_exact_options(mock_env):
    env, state, log = mock_env
    state.write_text('other\n', encoding='ascii')

    result = _run(env)

    assert result.returncode == 0, result.stderr
    assert result.stdout == TARGET_RECORD
    assert state.read_text(encoding='ascii').strip() == 'target'
    lines = log.read_text(encoding='utf-8').splitlines()
    assert lines[0].startswith('udisksctl unmount --block-device ')
    assert lines[1] == 'sudo mount -o rw,nosuid,nodev ' + UUID_PATH + ' ' + TARGET


def test_apply_moves_desktop_automount_with_unhardened_options(mock_env):
    env, state, log = mock_env
    state.write_text('other_desktop_options\n', encoding='ascii')

    result = _run(env)

    assert result.returncode == 0, result.stderr
    assert result.stdout == TARGET_RECORD
    assert state.read_text(encoding='ascii').strip() == 'target'
    lines = log.read_text(encoding='utf-8').splitlines()
    assert lines[0].startswith('udisksctl unmount --block-device ')
    assert lines[1] == 'sudo mount -o rw,nosuid,nodev ' + UUID_PATH + ' ' + TARGET


def test_unmount_failure_does_not_attempt_mount(mock_env):
    env, state, log = mock_env
    state.write_text('other\n', encoding='ascii')
    env['MOCK_UNMOUNT_FAIL'] = '1'

    result = _run(env)

    assert result.returncode != 0
    assert 'unmount' in result.stderr
    assert state.read_text(encoding='ascii').strip() == 'other'
    assert all(not line.startswith('sudo ') for line in log.read_text().splitlines())


def test_mount_failure_is_reported_without_fake_success(mock_env):
    env, state, log = mock_env
    env['MOCK_MOUNT_FAIL'] = '1'

    result = _run(env)

    assert result.returncode != 0
    assert 'mount failed' in result.stderr
    assert state.read_text(encoding='ascii').strip() == 'none'
    assert log.read_text(encoding='utf-8').startswith('sudo mount ')


def test_wrong_uuid_fails_before_mount_operations(mock_env):
    env, _state, log = mock_env
    env['MOCK_WRONG_UUID'] = '1'

    result = _run(env)

    assert result.returncode != 0
    assert 'UUID/device filesystem metadata' in result.stderr
    assert log.read_text(encoding='utf-8') == ''


def test_unsafe_target_fails_before_unmount_or_mount(mock_env):
    env, state, log = mock_env
    state.write_text('other\n', encoding='ascii')
    env['MOCK_UNSAFE_TARGET'] = '1'

    result = _run(env)

    assert result.returncode != 0
    assert 'real directory' in result.stderr
    assert log.read_text(encoding='utf-8') == ''


def test_help_does_not_probe_or_change_mount_state(mock_env):
    env, _state, log = mock_env

    result = _run(env, '--help')

    assert result.returncode == 0
    assert 'Usage: mount_competitive_ssd.sh' in result.stdout
    assert log.read_text(encoding='utf-8') == ''
