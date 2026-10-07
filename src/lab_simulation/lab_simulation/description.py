"""Build the lab robot description (the SO-101 on the table) from its xacro."""

from pathlib import Path
from typing import Optional, Union

XACRO = 'lab_so101.urdf.xacro'


def robot_description(path: Optional[Union[str, Path]] = None, controllers: str = '') -> str:
    """
    Return the URDF XML of the lab arm.

    path defaults to the installed urdf/lab_so101.urdf.xacro. controllers is
    the ros2_control parameter file given to the Gazebo plugin; it is only
    needed when the description is spawned into Gazebo.
    """
    import xacro
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('lab_simulation')) / 'urdf' / XACRO
    return xacro.process_file(str(path), mappings={'controllers': controllers}).toxml()
