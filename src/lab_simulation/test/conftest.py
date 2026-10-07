"""Make the in-source lab_simulation package importable, and share fixtures."""

from pathlib import Path
import sys

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))


@pytest.fixture(scope='session')
def lab_description():
    """Return the lab robot description (the SO-101 on the table) as URDF XML."""
    from lab_simulation.description import robot_description
    return robot_description(PKG / 'urdf' / 'lab_so101.urdf.xacro')


@pytest.fixture(scope='session')
def lab_kinematics(lab_description):
    """Return the kinematics of the lab arm, in the world frame."""
    from lab_simulation.so101_kinematics import Kinematics
    return Kinematics(lab_description)
