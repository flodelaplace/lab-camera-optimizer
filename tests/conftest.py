"""
Shared fixtures for the lab-camera-optimizer test suite.
"""

import os
import sys
import pytest

# Ensure project root is on sys.path so 'core' is importable as a package
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


# ─────────────────────────────────────────────────────────────────────────────
# Minimal YAML configuration for a 4×3 m rectangular room
# ─────────────────────────────────────────────────────────────────────────────

SIMPLE_CONFIG_YAML = """\
room:
  corners: [[0,0],[4,0],[4,3],[0,3]]
  height: 3.0

obstacles: []

subject:
  height: 1.8
  foot_z: 0.0

camera_sets:
  - id: "cam_A"
    mounting: "wall"
    fov_h_landscape: 110.0
    fov_v_landscape:  70.0
    fov_h_portrait:   70.0
    fov_v_portrait:  110.0
    max_range: 8.0
    min_range: 0.5
    height_options: [2.0]
    max_count: 4
    min_spacing: 1.0
    score_weight: 1.0
    color: "#1f77b4"

capture_zones:
  - id: "corridor"
    type: "corridor"
    priority: 0.5
    length: 3.0
    placement:
      x_start_options: [0.5]
      y_options: [1.5]

  - id: "zone"
    type: "sub_zone"
    priority: 1.0
    length: 2.0
    contained_in: "corridor"
    offset_options: [0.5]

  - id: "sts"
    type: "point"
    priority: 2.0
    radius: 0.3

optimization:
  target_coverage: 2
  bilateral_weight: 0.8
  vertical_coverage_threshold: 0.9
  restarts_per_combo: 3
  wall_step: 0.5
  angle_steps: 12
  tripod_grid_step: 0.8
  distance_quality_factor: 0.01
  algo: "greedy"
  graph_mode: "records_only"
"""


@pytest.fixture
def simple_config_path(tmp_path):
    """Write minimal YAML config to a temp file and return its path."""
    p = tmp_path / "test_config.yaml"
    p.write_text(SIMPLE_CONFIG_YAML)
    return str(p)


@pytest.fixture
def cfg(simple_config_path):
    """Load and return a minimal Config namespace."""
    from core.config_loader import load_config
    return load_config(simple_config_path)


@pytest.fixture
def state(cfg):
    """Build a state dict matching the minimal config."""
    from core.room import build_wall_segments
    wall_segs = build_wall_segments(cfg.ROOM_CORNERS, cfg.obstacles, cfg.ROOM_HEIGHT)
    return {
        "walk_y":           1.5,
        "walk_x_start":     0.5,
        "walk_x_end":       3.5,
        "analysis_x_start": 1.0,
        "analysis_x_end":   3.0,
        "sts_x":            2.0,
        "sts_y":            1.5,
        "wall_segments":    wall_segs,
    }
