"""The world launch refuses to start a second simulation."""

import importlib.util
from pathlib import Path
import subprocess

import pytest

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'arm_world.launch.py'


def load():
    spec = importlib.util.spec_from_file_location('arm_world_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fake_pgrep(output):
    def run(cmd, **kwargs):
        assert cmd[:2] == ['pgrep', '-f']
        return subprocess.CompletedProcess(cmd, 0 if output else 1, stdout=output, stderr='')
    return run


def test_refuses_when_gazebo_is_running(monkeypatch):
    module = load()
    monkeypatch.setattr(module.subprocess, 'run', fake_pgrep('4067\n4733\n'))
    with pytest.raises(RuntimeError, match='already running.*pid 4067'):
        module.refuse_second_simulation()


def test_starts_when_nothing_is_running(monkeypatch):
    module = load()
    monkeypatch.setattr(module.subprocess, 'run', fake_pgrep(''))
    module.refuse_second_simulation()
