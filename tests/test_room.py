"""
Tests for core/room.py — geometry, line-of-sight, coverage.
"""

import math
import pytest
from core.room import (
    point_in_polygon,
    point_in_room,
    point_in_obstacle,
    build_wall_segments,
    has_line_of_sight,
    vertical_body_coverage,
    point_in_wedge,
    cam_side,
    cam_fixed_tilt,
    obstacle_segments,
    obs_centroid,
)


# ─────────────────────────────────────────────────────────────────────────────
# point_in_polygon
# ─────────────────────────────────────────────────────────────────────────────

class TestPointInPolygon:

    SQUARE = [(0, 0), (4, 0), (4, 3), (0, 3)]

    def test_inside(self):
        assert point_in_polygon(2, 1.5, self.SQUARE) is True

    def test_outside_right(self):
        assert point_in_polygon(5, 1.5, self.SQUARE) is False

    def test_outside_negative(self):
        assert point_in_polygon(-1, 1.5, self.SQUARE) is False

    def test_outside_above(self):
        assert point_in_polygon(2, 4, self.SQUARE) is False

    def test_l_shape_inside_arm(self):
        l_shape = [(0, 0), (4, 0), (4, 2), (2, 2), (2, 3), (0, 3)]
        assert point_in_polygon(1, 2.5, l_shape) is True

    def test_l_shape_outside_notch(self):
        l_shape = [(0, 0), (4, 0), (4, 2), (2, 2), (2, 3), (0, 3)]
        assert point_in_polygon(3, 2.5, l_shape) is False


# ─────────────────────────────────────────────────────────────────────────────
# point_in_room  /  point_in_obstacle
# ─────────────────────────────────────────────────────────────────────────────

class TestPointInRoom:

    CORNERS = [(0, 0), (10, 0), (10, 6), (0, 6)]

    def test_inside_empty_room(self):
        assert point_in_room(5, 3, self.CORNERS, [], 3.0) is True

    def test_outside_room(self):
        assert point_in_room(11, 3, self.CORNERS, [], 3.0) is False

    def test_blocked_by_full_height_obstacle(self):
        obs = [{"vertices": [(4, 2), (6, 2), (6, 4), (4, 4)], "height": 3.0}]
        assert point_in_room(5, 3, self.CORNERS, obs, 3.0) is False

    def test_partial_height_obstacle_does_not_block(self):
        obs = [{"vertices": [(4, 2), (6, 2), (6, 4), (4, 4)], "height": 1.0}]
        assert point_in_room(5, 3, self.CORNERS, obs, 3.0) is True

    def test_point_in_obstacle(self):
        obs = [{"vertices": [(4, 2), (6, 2), (6, 4), (4, 4)], "height": 3.0}]
        assert point_in_obstacle(5, 3, obs, 3.0) is True

    def test_point_not_in_obstacle(self):
        obs = [{"vertices": [(4, 2), (6, 2), (6, 4), (4, 4)], "height": 3.0}]
        assert point_in_obstacle(1, 1, obs, 3.0) is False


# ─────────────────────────────────────────────────────────────────────────────
# build_wall_segments
# ─────────────────────────────────────────────────────────────────────────────

class TestBuildWallSegments:

    def test_rectangle_has_four_segments(self):
        corners = [(0, 0), (4, 0), (4, 3), (0, 3)]
        segs = build_wall_segments(corners, [], 3.0)
        assert len(segs) == 4

    def test_full_height_obstacle_adds_segments(self):
        corners = [(0, 0), (4, 0), (4, 3), (0, 3)]
        obs = [{"vertices": [(1, 1), (2, 1), (2, 2), (1, 2)], "height": 3.0}]
        segs = build_wall_segments(corners, obs, 3.0)
        assert len(segs) == 8  # 4 room + 4 obstacle

    def test_partial_height_obstacle_not_included(self):
        corners = [(0, 0), (4, 0), (4, 3), (0, 3)]
        obs = [{"vertices": [(1, 1), (2, 1), (2, 2), (1, 2)], "height": 1.0}]
        segs = build_wall_segments(corners, obs, 3.0)
        assert len(segs) == 4  # only room walls


# ─────────────────────────────────────────────────────────────────────────────
# has_line_of_sight
# ─────────────────────────────────────────────────────────────────────────────

class TestHasLineOfSight:

    CORNERS = [(0, 0), (10, 0), (10, 6), (0, 6)]

    def test_clear_path(self):
        segs = build_wall_segments(self.CORNERS, [], 3.0)
        assert has_line_of_sight(1, 3, 5, 3, segs, [], 3.0,
                                 cam_z=2.0, target_z=0.9) is True

    def test_blocked_by_full_height_wall(self):
        obs = [{"vertices": [(3, 0), (3.2, 0), (3.2, 6), (3, 6)], "height": 3.0}]
        segs = build_wall_segments(self.CORNERS, obs, 3.0)
        assert has_line_of_sight(1, 3, 5, 3, segs, obs, 3.0,
                                 cam_z=2.0, target_z=0.9) is False

    def test_can_see_over_low_obstacle(self):
        obs = [{"vertices": [(4, 2), (4.2, 2), (4.2, 4), (4, 4)], "height": 1.0}]
        segs = build_wall_segments(self.CORNERS, obs, 3.0)
        # cam at 2.5m, target at 1.5m → line goes above 1.0m obstacle
        assert has_line_of_sight(1, 3, 8, 3, segs, obs, 3.0,
                                 cam_z=2.5, target_z=1.5) is True

    def test_blocked_by_low_obstacle_at_ground(self):
        obs = [{"vertices": [(4, 2), (4.2, 2), (4.2, 4), (4, 4)], "height": 1.0}]
        segs = build_wall_segments(self.CORNERS, obs, 3.0)
        # cam at 0.5m, target at 0.1m → line is below 1.0m obstacle
        assert has_line_of_sight(1, 3, 8, 3, segs, obs, 3.0,
                                 cam_z=0.5, target_z=0.1) is False

    def test_coincident_points(self):
        segs = build_wall_segments(self.CORNERS, [], 3.0)
        assert has_line_of_sight(3, 3, 3, 3, segs, [], 3.0,
                                 cam_z=2.0, target_z=0.9) is True

    def test_blocked_when_ray_dips_below_obstacle_at_far_edge(self):
        """High cam → low target over a 1.5m box: the ray is above the box at
        mid-span (~1.6m) but descends below it before exiting the footprint
        (~1.4m at the far edge) → must be blocked. The old midpoint-only check
        wrongly returned visible here."""
        obs = [{"vertices": [(4, 2), (5, 2), (5, 4), (4, 4)], "height": 1.5}]
        segs = build_wall_segments(self.CORNERS, obs, 3.0)
        assert has_line_of_sight(1, 3, 8, 3, segs, obs, 3.0,
                                 cam_z=3.0, target_z=0.2) is False

    def test_clears_when_above_obstacle_at_both_edges(self):
        """Ray stays above the 1.5m box across the whole footprint → visible."""
        obs = [{"vertices": [(4, 2), (5, 2), (5, 4), (4, 4)], "height": 1.5}]
        segs = build_wall_segments(self.CORNERS, obs, 3.0)
        assert has_line_of_sight(1, 3, 8, 3, segs, obs, 3.0,
                                 cam_z=3.0, target_z=2.0) is True


# ─────────────────────────────────────────────────────────────────────────────
# vertical_body_coverage
# ─────────────────────────────────────────────────────────────────────────────

class TestVerticalBodyCoverage:

    def test_full_coverage_far_away(self):
        v = vertical_body_coverage(0, 0, 2.0, 5, 0, 70.0,
                                   1.8, 0.0, 1.8, v_thresh=0.9)
        assert v > 0.9

    def test_zero_coverage_too_close(self):
        """Point closer than the 0.01m epsilon → zero coverage."""
        v = vertical_body_coverage(0, 0, 2.0, 0.005, 0, 70.0,
                                   1.8, 0.0, 1.8, v_thresh=0.9)
        assert v == 0.0

    def test_zero_same_position(self):
        v = vertical_body_coverage(1, 1, 2.0, 1, 1, 70.0,
                                   1.8, 0.0, 1.8, v_thresh=0.9)
        assert v == 0.0

    def test_coverage_decreases_when_closer(self):
        """Closer → narrower vertical angle → less body coverage at some point."""
        v_far  = vertical_body_coverage(0, 0, 2.0, 6, 0, 70.0, 1.8, 0.0, 1.8)
        v_near = vertical_body_coverage(0, 0, 2.0, 1.5, 0, 70.0, 1.8, 0.0, 1.8)
        # Far camera should have good coverage; very near may have less
        assert v_far >= v_near or v_near == 0.0

    def test_returns_at_most_one(self):
        v = vertical_body_coverage(0, 0, 2.0, 3, 0, 110.0,
                                   1.8, 0.0, 1.8, v_thresh=0.0)
        assert v <= 1.0


# ─────────────────────────────────────────────────────────────────────────────
# point_in_wedge
# ─────────────────────────────────────────────────────────────────────────────

class TestPointInWedge:

    CORNERS = [(0, 0), (10, 0), (10, 6), (0, 6)]

    def _wall_segs(self):
        return build_wall_segments(self.CORNERS, [], 3.0)

    def test_point_inside_cone(self):
        in_w, dist = point_in_wedge(5, 3, 0, 3, 0, 110, 12, 0.5,
                                    self._wall_segs(), [], 3.0, 1.8, cam_z=2.0)
        assert in_w is True
        assert abs(dist - 5.0) < 0.01

    def test_point_outside_angle(self):
        # Camera at (5,3) facing right (0°), target at (5,0) → 90° below
        in_w, _ = point_in_wedge(5, 0, 5, 3, 0, 60, 12, 0.5,
                                 self._wall_segs(), [], 3.0, 1.8, cam_z=2.0)
        assert in_w is False

    def test_too_far(self):
        in_w, _ = point_in_wedge(9, 3, 0, 3, 0, 110, 5.0, 0.5,
                                 self._wall_segs(), [], 3.0, 1.8, cam_z=2.0)
        assert in_w is False

    def test_too_close(self):
        in_w, _ = point_in_wedge(0.2, 3, 0, 3, 0, 110, 12, 0.5,
                                 self._wall_segs(), [], 3.0, 1.8, cam_z=2.0)
        assert in_w is False


# ─────────────────────────────────────────────────────────────────────────────
# cam_side  /  cam_fixed_tilt
# ─────────────────────────────────────────────────────────────────────────────

class TestCamSide:
    def test_south(self):
        assert cam_side(1.0, 2.0) == 'S'

    def test_north(self):
        assert cam_side(3.0, 2.0) == 'N'

    def test_on_axis_is_north(self):
        assert cam_side(2.0, 2.0) == 'N'


class TestCamFixedTilt:
    def test_tilt_is_negative_when_camera_above_midpoint(self):
        """Camera at 2.5m looking at human midpoint at 0.9m → tilt is negative (downward)."""
        tilt = cam_fixed_tilt(0, 0, 2.5, 90, 3.0, 1.8)
        assert tilt < 0  # looking down

    def test_tilt_magnitude_increases_when_closer(self):
        tilt_far  = cam_fixed_tilt(0, 0, 2.5, 90, 5.0, 1.8)
        tilt_close = cam_fixed_tilt(0, 2.5, 2.5, 90, 3.0, 1.8)
        assert abs(tilt_close) >= abs(tilt_far)


# ─────────────────────────────────────────────────────────────────────────────
# obstacle helpers
# ─────────────────────────────────────────────────────────────────────────────

class TestObstacleHelpers:
    OBS = {"vertices": [(1, 1), (3, 1), (3, 2), (1, 2)], "height": 3.0}

    def test_obstacle_segments(self):
        segs = obstacle_segments(self.OBS)
        assert len(segs) == 4

    def test_obs_centroid(self):
        cx, cy = obs_centroid(self.OBS)
        assert abs(cx - 2.0) < 0.01
        assert abs(cy - 1.5) < 0.01
