"""
The overhead RGB-D camera: its Gazebo model and the perception node's settings.

One definition feeds both, so the camera the simulation renders and the
camera model perception projects through cannot drift apart. The pose was
chosen in the M3a spike: the whole workspace is in view and the arm at home
hides none of it.
"""

from typing import Dict

from . import layout

POSITION = (0.55, -0.30, 1.30)
YAW = 2.481             # looks towards (0.10, 0.05) on the table
PITCH = 0.767
HFOV = 0.9              # rad
WIDTH, HEIGHT = 640, 480
RATE_HZ = 2.0           # frames per simulated second; perception looks before each pick
TOPIC = 'lab/camera'    # Gazebo publishes TOPIC/image, TOPIC/depth_image, TOPIC/camera_info
NAME = 'overhead_camera'
MODEL_ID = 'google/siglip2-base-patch16-224'
PROMPT = 'a photo of a {}'
# Naming thresholds on SigLIP 2 cosine similarity. Calibrated on 30 live
# scenes with random lighting and piece angles (8 October 2026): all 90
# pieces named correctly; lowest best score 0.120, smallest lead over the
# second label 0.030. The thresholds sit well below both.
MIN_SCORE = 0.08
MIN_MARGIN = 0.015
BIN_MASK_RADIUS = 0.07  # ignore anything inside a bin's footprint


def sdf() -> str:
    """Return the camera model as an SDF string, for ros_gz_sim create -string."""
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{NAME}">
    <static>true</static>
    <link name="link">
      <sensor name="rgbd" type="rgbd_camera">
        <always_on>1</always_on>
        <update_rate>{RATE_HZ}</update_rate>
        <topic>{TOPIC}</topic>
        <camera>
          <horizontal_fov>{HFOV}</horizontal_fov>
          <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>3.0</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""


def spawn_arguments() -> list:
    """Return the ros_gz_sim create arguments that place the camera."""
    x, y, z = POSITION
    return ['-string', sdf(), '-name', NAME, '-x', str(x), '-y', str(y), '-z', str(z),
            '-R', '0', '-P', str(PITCH), '-Y', str(YAW)]


def perception_parameters(embedder: str = 'siglip') -> Dict:
    """Return the perception node's parameters for this world."""
    bins = list(layout.BINS.values())
    return {
        'camera_position': list(POSITION),
        'camera_yaw': YAW,
        'camera_pitch': PITCH,
        'table_top': layout.TABLE_TOP,
        'axis_xy': list(layout.PAN_AXIS_XY),
        'radius': [layout.MIN_RADIUS - 0.04, layout.MAX_RADIUS + 0.08],
        'height': [0.005, 0.07],
        'exclude_xy': [v for b in bins for v in b.xy],
        'exclude_radius': BIN_MASK_RADIUS,
        'labels': sorted(layout.PIECES),
        'prompts': [PROMPT.format(layout.PIECES[n].description) for n in sorted(layout.PIECES)],
        'embedder': embedder,
        'model_id': MODEL_ID,
        'min_score': MIN_SCORE,
        'min_margin': MIN_MARGIN,
    }
