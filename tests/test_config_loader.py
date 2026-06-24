"""
Tests for core/config_loader.py — YAML loading and validation.
"""

import os
import pytest
import warnings
from core.config_loader import load_config, validate_config


# ─────────────────────────────────────────────────────────────────────────────
# Successful loading
# ─────────────────────────────────────────────────────────────────────────────

class TestLoadConfig:

    def test_loads_simple_config(self, simple_config_path):
        cfg = load_config(simple_config_path)
        assert cfg.ROOM_CORNERS == [(0, 0), (4, 0), (4, 3), (0, 3)]
        assert cfg.ROOM_HEIGHT == 3.0

    def test_room_attributes(self, cfg):
        assert cfg.room.height == 3.0
        assert len(cfg.room.corners) == 4

    def test_subject_attributes(self, cfg):
        assert cfg.subject.height == 1.8
        assert cfg.subject.foot_z == 0.0
        assert cfg.subject.head_z == 1.8

    def test_camera_sets_loaded(self, cfg):
        assert len(cfg.camera_sets) >= 1
        cam_A = cfg.camera_sets[0]
        assert cam_A.id == "cam_A"
        assert cam_A.mounting == "wall"
        assert cam_A.fov_h_L == 110.0
        assert cam_A.max_count == 4

    def test_capture_zones_loaded(self, cfg):
        assert len(cfg.capture_zones) == 3
        ids = [z.id for z in cfg.capture_zones]
        assert "corridor" in ids
        assert "zone" in ids
        assert "sts" in ids

    def test_optimization_params(self, cfg):
        assert cfg.opt.target_coverage == 2
        assert cfg.opt.bilateral_weight == 0.8
        assert cfg.opt.algo == "greedy"

    def test_backward_compat_shortcuts(self, cfg):
        assert cfg.HUMAN_HEIGHT == 1.8
        assert cfg.TARGET_COVERAGE == 2
        assert cfg.BILATERAL_WEIGHT == 0.8

    def test_file_not_found(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            load_config(str(tmp_path / "nonexistent.yaml"))


# ─────────────────────────────────────────────────────────────────────────────
# Validation — fatal errors
# ─────────────────────────────────────────────────────────────────────────────

class TestValidationErrors:

    def _write_and_load(self, tmp_path, yaml_text):
        p = tmp_path / "bad.yaml"
        p.write_text(yaml_text)
        return load_config(str(p))

    def test_negative_room_height(self, tmp_path):
        yaml = """\
room:
  corners: [[0,0],[4,0],[4,3],[0,3]]
  height: -1.0
obstacles: []
subject: {height: 1.8, foot_z: 0.0}
camera_sets:
  - id: cam_A
    mounting: wall
    fov_h_landscape: 110
    fov_v_landscape: 70
    fov_h_portrait: 70
    fov_v_portrait: 110
    max_range: 8
    min_range: 0.5
    height_options: [2.0]
    max_count: 4
    min_spacing: 1.0
    score_weight: 1.0
    color: "#1f77b4"
capture_zones: []
optimization:
  target_coverage: 2
  bilateral_weight: 0.5
  vertical_coverage_threshold: 0.9
  restarts_per_combo: 3
"""
        with pytest.raises(ValueError, match="Room height must be positive"):
            self._write_and_load(tmp_path, yaml)

    def test_invalid_bilateral_weight(self, tmp_path):
        yaml = """\
room:
  corners: [[0,0],[4,0],[4,3],[0,3]]
  height: 3.0
obstacles: []
subject: {height: 1.8, foot_z: 0.0}
camera_sets:
  - id: cam_A
    mounting: wall
    fov_h_landscape: 110
    fov_v_landscape: 70
    fov_h_portrait: 70
    fov_v_portrait: 110
    max_range: 8
    min_range: 0.5
    height_options: [2.0]
    max_count: 4
    min_spacing: 1.0
    score_weight: 1.0
    color: "#1f77b4"
capture_zones: []
optimization:
  target_coverage: 2
  bilateral_weight: 1.5
  vertical_coverage_threshold: 0.9
  restarts_per_combo: 3
"""
        with pytest.raises(ValueError, match="bilateral_weight"):
            self._write_and_load(tmp_path, yaml)

    def test_max_range_less_than_min_range(self, tmp_path):
        yaml = """\
room:
  corners: [[0,0],[4,0],[4,3],[0,3]]
  height: 3.0
obstacles: []
subject: {height: 1.8, foot_z: 0.0}
camera_sets:
  - id: cam_A
    mounting: wall
    fov_h_landscape: 110
    fov_v_landscape: 70
    fov_h_portrait: 70
    fov_v_portrait: 110
    max_range: 0.3
    min_range: 0.5
    height_options: [2.0]
    max_count: 4
    min_spacing: 1.0
    score_weight: 1.0
    color: "#1f77b4"
capture_zones: []
optimization:
  target_coverage: 2
  bilateral_weight: 0.5
  vertical_coverage_threshold: 0.9
  restarts_per_combo: 3
"""
        with pytest.raises(ValueError, match="max_range"):
            self._write_and_load(tmp_path, yaml)


# ─────────────────────────────────────────────────────────────────────────────
# Validation — warnings (non-fatal)
# ─────────────────────────────────────────────────────────────────────────────

class TestValidationWarnings:

    def test_camera_height_above_room(self, tmp_path):
        yaml = """\
room:
  corners: [[0,0],[4,0],[4,3],[0,3]]
  height: 3.0
obstacles: []
subject: {height: 1.8, foot_z: 0.0}
camera_sets:
  - id: cam_A
    mounting: wall
    fov_h_landscape: 110
    fov_v_landscape: 70
    fov_h_portrait: 70
    fov_v_portrait: 110
    max_range: 8
    min_range: 0.5
    height_options: [3.5]
    max_count: 4
    min_spacing: 1.0
    score_weight: 1.0
    color: "#1f77b4"
capture_zones: []
optimization:
  target_coverage: 2
  bilateral_weight: 0.5
  vertical_coverage_threshold: 0.9
  restarts_per_combo: 3
"""
        p = tmp_path / "warn.yaml"
        p.write_text(yaml)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            cfg = load_config(str(p))
        warn_msgs = [str(x.message) for x in w]
        assert any("height 3.5m >= room height" in m for m in warn_msgs)

    def test_corridor_outside_room(self, tmp_path):
        yaml = """\
room:
  corners: [[0,0],[4,0],[4,3],[0,3]]
  height: 3.0
obstacles: []
subject: {height: 1.8, foot_z: 0.0}
camera_sets:
  - id: cam_A
    mounting: wall
    fov_h_landscape: 110
    fov_v_landscape: 70
    fov_h_portrait: 70
    fov_v_portrait: 110
    max_range: 8
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
    length: 10.0
    placement:
      x_start_options: [0.0]
      y_options: [1.5]
optimization:
  target_coverage: 2
  bilateral_weight: 0.5
  vertical_coverage_threshold: 0.9
  restarts_per_combo: 3
"""
        p = tmp_path / "warn.yaml"
        p.write_text(yaml)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            cfg = load_config(str(p))
        warn_msgs = [str(x.message) for x in w]
        assert any("corridor end" in m and "outside the room" in m
                    for m in warn_msgs)

    def test_obstacle_outside_room(self, tmp_path):
        yaml = """\
room:
  corners: [[0,0],[4,0],[4,3],[0,3]]
  height: 3.0
obstacles:
  - type: polygon
    vertices: [[5,5],[6,5],[6,6],[5,6]]
    height: 3.0
    label: "Outside"
subject: {height: 1.8, foot_z: 0.0}
camera_sets:
  - id: cam_A
    mounting: wall
    fov_h_landscape: 110
    fov_v_landscape: 70
    fov_h_portrait: 70
    fov_v_portrait: 110
    max_range: 8
    min_range: 0.5
    height_options: [2.0]
    max_count: 4
    min_spacing: 1.0
    score_weight: 1.0
    color: "#1f77b4"
capture_zones: []
optimization:
  target_coverage: 2
  bilateral_weight: 0.5
  vertical_coverage_threshold: 0.9
  restarts_per_combo: 3
"""
        p = tmp_path / "warn.yaml"
        p.write_text(yaml)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            cfg = load_config(str(p))
        warn_msgs = [str(x.message) for x in w]
        assert any("Outside" in m and "outside the room" in m
                    for m in warn_msgs)
