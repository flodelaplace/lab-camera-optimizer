"""
Tests for core/candidates.py — sample points and candidate generation.
"""

import pytest
from core.candidates import build_sample_points, generate_candidates
from core.room import point_in_room


# ─────────────────────────────────────────────────────────────────────────────
# build_sample_points
# ─────────────────────────────────────────────────────────────────────────────

class TestBuildSamplePoints:

    def test_returns_non_empty(self, cfg, state):
        pts = build_sample_points(cfg, state)
        assert len(pts) > 0

    def test_all_points_inside_room(self, cfg, state):
        pts = build_sample_points(cfg, state)
        for px, py, w in pts:
            assert point_in_room(px, py, cfg.ROOM_CORNERS,
                                 cfg.obstacles, cfg.ROOM_HEIGHT), \
                f"Point ({px:.2f}, {py:.2f}) outside room"

    def test_weights_positive(self, cfg, state):
        pts = build_sample_points(cfg, state)
        assert all(w > 0 for _, _, w in pts)

    def test_points_have_three_components(self, cfg, state):
        pts = build_sample_points(cfg, state)
        for pt in pts:
            assert len(pt) == 3


# ─────────────────────────────────────────────────────────────────────────────
# generate_candidates
# ─────────────────────────────────────────────────────────────────────────────

class TestGenerateCandidates:

    def test_returns_two_lists(self, cfg, state):
        cands_A, cands_B = generate_candidates(cfg, state, wall_step=0.5)
        assert isinstance(cands_A, list)
        assert isinstance(cands_B, list)

    def test_wall_candidates_non_empty(self, cfg, state):
        sample_pts = build_sample_points(cfg, state)
        cands_A, _ = generate_candidates(cfg, state, wall_step=0.5,
                                         sample_points=sample_pts)
        assert len(cands_A) > 0

    def test_candidate_tuple_format(self, cfg, state):
        cands_A, _ = generate_candidates(cfg, state, wall_step=0.5)
        for cand in cands_A:
            assert len(cand) == 5  # (x, y, angle, orient, height)
            x, y, angle, orient, zh = cand
            assert isinstance(x, float) or isinstance(x, int)
            assert orient in ("L", "P")
            assert zh > 0

    def test_no_tripod_when_disabled(self, cfg, state):
        """With max_count=0 for tripod set, cam_B should be empty."""
        # The simple config has no tripod set enabled
        _, cands_B = generate_candidates(cfg, state, wall_step=0.5)
        assert len(cands_B) == 0
