"""
Tests for core/scoring.py — score configuration, FOV, distance quality.
"""

import math
import random
import pytest
from core.scoring import (get_fov, dist_quality, score_configuration,
                          score_at_point, precompute_coverage, score_indexed)
from core.config_loader import load_config
from core.room import build_wall_segments


# ─────────────────────────────────────────────────────────────────────────────
# get_fov
# ─────────────────────────────────────────────────────────────────────────────

class TestGetFov:

    def test_landscape(self, cfg):
        cam_A = next(c for c in cfg.camera_sets if c.mounting == "wall")
        fov_h, fov_v = get_fov(cam_A, "L")
        assert fov_h == 110.0
        assert fov_v == 70.0

    def test_portrait(self, cfg):
        cam_A = next(c for c in cfg.camera_sets if c.mounting == "wall")
        fov_h, fov_v = get_fov(cam_A, "P")
        assert fov_h == 70.0
        assert fov_v == 110.0


# ─────────────────────────────────────────────────────────────────────────────
# dist_quality
# ─────────────────────────────────────────────────────────────────────────────

class TestDistQuality:

    def test_zero_distance(self):
        assert dist_quality(0.0, 0.01) == 1.0

    def test_decreases_with_distance(self):
        q1 = dist_quality(1.0, 0.01)
        q5 = dist_quality(5.0, 0.01)
        assert q1 > q5

    def test_always_positive(self):
        assert dist_quality(100.0, 0.01) > 0.0


# ─────────────────────────────────────────────────────────────────────────────
# score_configuration
# ─────────────────────────────────────────────────────────────────────────────

class TestScoreConfiguration:

    def test_no_cameras_scores_zero(self, cfg, state):
        sample_pts = [(2.0, 1.5, 1.0), (2.5, 1.5, 1.0)]
        score, cov, s, n = score_configuration([], [], sample_pts, cfg, state)
        assert score == 0.0
        assert all(c == 0 for c in cov)

    def test_some_cameras_positive_score(self, cfg, state):
        """A well-placed camera should produce a positive score."""
        # Camera on south wall facing north, at centre of analysis zone
        cam_A_list = [(2.0, 0.0, 90, "L", 2.0)]
        sample_pts = [(2.0, 1.5, 1.0)]
        score, cov, s, n = score_configuration(
            cam_A_list, [], sample_pts, cfg, state)
        assert score > 0.0

    def test_more_cameras_higher_score(self, cfg, state):
        """Adding a camera should not decrease the score."""
        sample_pts = [(2.0, 1.5, 1.0), (2.5, 1.5, 1.0)]
        cam1 = [(2.0, 0.0, 90, "L", 2.0)]
        cam2 = [(2.0, 0.0, 90, "L", 2.0), (2.0, 3.0, 270, "L", 2.0)]
        s1, *_ = score_configuration(cam1, [], sample_pts, cfg, state)
        s2, *_ = score_configuration(cam2, [], sample_pts, cfg, state)
        assert s2 >= s1

    def test_bilateral_cameras_outscore_unilateral(self, cfg, state):
        """Cameras on both sides should score higher than all on one side."""
        sample_pts = [(2.0, 1.5, 1.0)]
        # Two cameras on south wall
        unilateral = [(1.5, 0.0, 90, "L", 2.0), (2.5, 0.0, 90, "L", 2.0)]
        # One on each side
        bilateral  = [(2.0, 0.0, 90, "L", 2.0), (2.0, 3.0, 270, "L", 2.0)]
        s_uni, *_ = score_configuration(unilateral, [], sample_pts, cfg, state)
        s_bil, *_ = score_configuration(bilateral, [], sample_pts, cfg, state)
        assert s_bil > s_uni


# ─────────────────────────────────────────────────────────────────────────────
# score_at_point
# ─────────────────────────────────────────────────────────────────────────────

class TestScoreAtPoint:

    def test_returns_zero_with_no_cameras(self, cfg, state):
        s = score_at_point(2.0, 1.5, [], [], cfg, state)
        assert s == 0.0

    def test_returns_positive_with_coverage(self, cfg, state):
        cam = [(2.0, 0.0, 90, "L", 2.0)]
        s = score_at_point(2.0, 1.5, cam, [], cfg, state)
        assert s > 0.0


# ─────────────────────────────────────────────────────────────────────────────
# precompute_coverage / score_indexed  —  must EXACTLY match score_configuration
# ─────────────────────────────────────────────────────────────────────────────

_TWO_SET_YAML = """\
room:
  corners: [[0,0],[8,0],[8,5],[0,5]]
  height: 3.0
obstacles: []
subject: {height: 1.8, foot_z: 0.0}
camera_sets:
  - id: wall
    mounting: wall
    fov_h_landscape: 110.0
    fov_v_landscape: 70.0
    fov_h_portrait: 70.0
    fov_v_portrait: 110.0
    max_range: 10.0
    min_range: 0.5
    height_options: [2.0]
    count_max: 6
    min_spacing: 0.5
    score_weight: 1.0
  - id: tripod
    mounting: tripod
    fov_h_landscape: 70.0
    fov_v_landscape: 50.0
    fov_h_portrait: 50.0
    fov_v_portrait: 70.0
    max_range: 9.0
    min_range: 0.8
    height_options: [1.5]
    count_max: 6
    min_spacing: 0.5
    walk_axis_margin: 0.5
    score_weight: 1.0
capture_zones:
  - id: zone
    type: polygon
    priority: 1.0
    grid_step: 0.5
    vertices: [[0,0],[4,0],[4,2],[0,2]]
    placement: {x_offsets: [2.0], y_offsets: [1.5]}
optimization:
  target_coverage: 3
  bilateral_weight: 0.8
  vertical_coverage_threshold: 0.8
  distance_quality_factor: 0.001
"""


class TestScoreIndexedEquivalence:
    """score_indexed (precomputed table) must equal score_configuration (geometry)."""

    def _setup(self, tmp_path):
        p = tmp_path / "two_set.yaml"
        p.write_text(_TWO_SET_YAML)
        cfg = load_config(str(p))
        verts = [(vx + 2.0, vy + 1.5) for (vx, vy) in cfg.capture_zones[0].vertices]
        cfg.capture_zones[0]._translated_vertices = verts
        sx = sum(v[0] for v in verts) / len(verts)
        sy = sum(v[1] for v in verts) / len(verts)
        state = {
            "walk_y": sy, "walk_x_start": 2.0, "walk_x_end": 6.0,
            "analysis_x_start": 2.0, "analysis_x_end": 6.0,
            "sts_x": sx, "sts_y": sy,
            "wall_segments": build_wall_segments(cfg.ROOM_CORNERS, cfg.obstacles, cfg.ROOM_HEIGHT),
        }
        from core.candidates import build_sample_points
        sp = build_sample_points(cfg, state)
        # Hand-built candidate pools (wall on the two long walls, tripods inside)
        cand_A = [(x, 0.0, 90, "L", 2.0) for x in (2.0, 3.5, 5.0)] + \
                 [(x, 5.0, 270, "L", 2.0) for x in (2.0, 3.5, 5.0)]
        cand_B = [(x, y, a, "L", 1.5)
                  for x in (2.5, 4.0, 5.5) for y in (1.0, 4.0) for a in (90, 270)]
        return cfg, state, sp, cand_A, cand_B

    def test_matches_on_random_subsets(self, tmp_path):
        cfg, state, sp, cand_A, cand_B = self._setup(tmp_path)
        covA, covB, meta = precompute_coverage(cand_A, cand_B, sp, cfg, state)
        rng = random.Random(0)
        for _ in range(40):
            selA = rng.sample(range(len(cand_A)), rng.randint(0, len(cand_A)))
            selB = rng.sample(range(len(cand_B)), rng.randint(0, len(cand_B)))
            listA = [cand_A[i] for i in selA]
            listB = [cand_B[i] for i in selB]
            s_geo, _, south_geo, north_geo = score_configuration(listA, listB, sp, cfg, state)
            s_idx, south_idx, north_idx = score_indexed(selA, selB, covA, covB, meta, cfg)
            assert s_idx == pytest.approx(s_geo, rel=1e-9, abs=1e-9)
            assert south_idx == pytest.approx(south_geo, rel=1e-9, abs=1e-9)
            assert north_idx == pytest.approx(north_geo, rel=1e-9, abs=1e-9)
