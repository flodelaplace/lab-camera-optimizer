"""
scoring.py
Score configuration: coverage, bilateral constraint, angular diversity.
All functions receive explicit context (cfg, state) — no global state.
"""

import math
import numpy as np
from .room import (vertical_body_coverage, point_in_wedge,
                   cam_fixed_tilt, cam_side, point_in_room)


def get_fov(cam_set, orientation):
    """Returns (fov_h, fov_v) for a camera set and orientation ('L' or 'P')."""
    if orientation == "P":
        return cam_set.fov_h_P, cam_set.fov_v_P
    return cam_set.fov_h_L, cam_set.fov_v_L


def dist_quality(d, k):
    """Distance quality multiplier: 1 / (1 + k*d²)."""
    return 1.0 / (1.0 + k * d * d)


def score_configuration(cam_A_list, cam_B_list,
                        sample_points, cfg, state):
    """
    Computes the global score for a camera configuration.

    Improvements over original:
    1. sum_v with diminishing returns instead of avg_v
       → rewards more cameras covering a point, not just quality of best one
    2. bilateral uses sum-per-side with saturation (not max)
       → rewards having SEVERAL cameras on each side, not just one good one
    3. angular proximity penalty for cameras that are BOTH positionally close
       AND pointing in the same direction (close + opposite angle = OK,
       far + same angle = OK, close + same angle = penalised)

    Returns (total_score, coverage_list, south_total, north_total)
    """
    cam_A     = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B_set = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)

    walk_y           = state["walk_y"]
    analysis_x_start = state["analysis_x_start"]
    analysis_x_end   = state["analysis_x_end"]
    v_thresh         = cfg.opt.vertical_coverage_threshold
    target_coverage  = cfg.TARGET_COVERAGE
    bilateral_weight = cfg.BILATERAL_WEIGHT
    k_dist           = cfg.DIST_QUALITY_K

    wall_segs = state["wall_segments"]
    obstacles = cfg.obstacles
    room_h    = cfg.ROOM_HEIGHT
    human_h   = cfg.HUMAN_HEIGHT
    human_fz  = cfg.HUMAN_FOOT_Z
    human_hz  = cfg.HUMAN_HEAD_Z

    total_score   = 0.0
    coverage_list = []
    south_total   = 0.0
    north_total   = 0.0

    for (px, py, weight) in sample_points:
        # Per-camera contributions at this point
        # Each entry: (angle, score_v, side)
        cam_contribs = []   # (angle, score_v, side, cx, cy)

        # ── Wall cameras (cam_A) ─────────────────────────────────────────
        for (cx, cy, angle, orient, zh) in cam_A_list:
            fov_h, fov_v = get_fov(cam_A, orient)
            in_cone, dist = point_in_wedge(
                px, py, cx, cy, angle, fov_h, cam_A.max_range, cam_A.min_range,
                wall_segs, obstacles, room_h, human_h, cam_z=zh)
            if not in_cone:
                continue
            fixed_tilt = cam_fixed_tilt(cx, cy, zh, angle, walk_y, human_h)
            v = vertical_body_coverage(cx, cy, zh, px, py, fov_v,
                                       human_h, human_fz, human_hz,
                                       v_thresh=v_thresh,
                                       fixed_tilt_rad=fixed_tilt)
            if v >= v_thresh:
                q = dist_quality(dist, k_dist)
                score_v = v * v * q * cam_A.score_weight * cam_A.score_factor
                side = cam_side(cy, walk_y)
                cam_contribs.append((angle, score_v, side, cx, cy))

        # ── Tripod cameras (cam_B) ───────────────────────────────────────
        if cam_B_set and cam_B_list:
            tripod_factor = cam_B_set.score_factor
            for (cx, cy, angle, orient, ih) in cam_B_list:
                fov_h, fov_v = get_fov(cam_B_set, orient)
                in_cone, dist = point_in_wedge(
                    px, py, cx, cy, angle, fov_h,
                    cam_B_set.max_range, cam_B_set.min_range,
                    wall_segs, obstacles, room_h, human_h, cam_z=ih)
                if not in_cone:
                    continue
                v = vertical_body_coverage(cx, cy, ih, px, py, fov_v,
                                           human_h, human_fz, human_hz,
                                           v_thresh=v_thresh,
                                           fixed_tilt_rad=None)
                if v >= v_thresh:
                    q = dist_quality(dist, k_dist)
                    score_v = v * v * q * cam_B_set.score_weight * tripod_factor

                    # Bilateral: weight cross-side contribution by distance
                    # to the walk axis.  A tripod on the axis contributes
                    # equally to both sides; one 2 m+ away counts only for
                    # its own side.
                    dist_to_axis = abs(cy - walk_y)
                    cross_factor = max(0.0, 1.0 - dist_to_axis / 2.0)
                    own_side   = cam_side(cy, walk_y)
                    other_side = 'N' if own_side == 'S' else 'S'
                    cam_contribs.append((angle, score_v, own_side, cx, cy))
                    if cross_factor > 0.01:
                        cam_contribs.append((angle, score_v * cross_factor,
                                             other_side, cx, cy))

        ps, south_v, north_v, n_c = _score_point(
            cam_contribs, weight, target_coverage, bilateral_weight)
        total_score += ps
        coverage_list.append(n_c)
        if analysis_x_start <= px <= analysis_x_end:
            south_total += south_v
            north_total += north_v

    return total_score, coverage_list, south_total, north_total


def _score_point(cam_contribs, weight, target_coverage, bilateral_weight):
    """
    Aggregate a single point's camera contributions into a point score.

    cam_contribs : list of (angle, score_v, side, cx, cy)
    Returns (point_score, south_v, north_v, n_contribs).

    Shared by score_configuration (geometry path) and score_indexed (precomputed
    coverage-table path) so the two ALWAYS produce identical scores.
    """
    if not cam_contribs:
        return 0.0, 0.0, 0.0, 0

    # ── 1. Effective camera count with angular+positional diversity ──
    sa = sorted(cam_contribs, key=lambda c: c[0])   # sort by angle
    effective_n = 1.0
    last_angle  = sa[0][0]
    last_x, last_y = sa[0][3], sa[0][4]
    for i in range(1, len(sa)):
        angle_i    = sa[i][0]
        cx_i, cy_i = sa[i][3], sa[i][4]
        ang_diff = abs((angle_i - last_angle + 180) % 360 - 180)
        pos_dist = math.sqrt((cx_i - last_x)**2 + (cy_i - last_y)**2)
        if ang_diff < 20:
            effective_n += 0.3
        else:
            if pos_dist < 2.0 and ang_diff < 60:
                proximity_factor = (pos_dist / 2.0)
                angle_factor     = (ang_diff / 60.0)
                effective_n += 0.3 + 0.7 * max(proximity_factor, angle_factor)
            else:
                effective_n += 1.0
        last_angle = angle_i
        last_x, last_y = cx_i, cy_i

    # ── 2. sum_v with diminishing returns ─────────────────────────────
    sorted_scores = sorted([c[1] for c in cam_contribs], reverse=True)
    sum_v = sum(s / math.sqrt(i + 1) for i, s in enumerate(sorted_scores))
    sum_v_norm = sum_v / math.sqrt(target_coverage)
    base = min(effective_n, target_coverage) / target_coverage * sum_v_norm

    # ── 3. Angular bonus ──────────────────────────────────────────────
    angles_only = [c[0] for c in sa]
    angular_bonus = 0.0
    if len(angles_only) >= 2:
        all_gaps = []
        for i in range(len(angles_only)):
            nxt = (i + 1) % len(angles_only)
            all_gaps.append((angles_only[nxt] - angles_only[i] + 360) % 360)
        if max(all_gaps) < 120:
            angular_bonus += 0.15
        if sum(1 for g in all_gaps if g > 60) >= 2:
            angular_bonus += 0.15

    # ── 4. Bilateral factor — sum per side with saturation ────────────
    south_scores = sorted([c[1] for c in cam_contribs if c[2] == 'S'], reverse=True)
    north_scores = sorted([c[1] for c in cam_contribs if c[2] == 'N'], reverse=True)
    south_v = sum(s / math.sqrt(i + 1) for i, s in enumerate(south_scores)) if south_scores else 0.0
    north_v = sum(s / math.sqrt(i + 1) for i, s in enumerate(north_scores)) if north_scores else 0.0
    denom        = max(south_v, north_v, 0.01)
    bilateral_pt = math.sqrt(south_v * north_v) / denom if bilateral_weight > 0 else 1.0

    point_score = weight * (
        (1.0 - bilateral_weight) * (base + angular_bonus) +
        bilateral_weight         * (base + angular_bonus) * bilateral_pt
    )
    return point_score, south_v, north_v, len(cam_contribs)


def count_cameras_3d(px, py, cam_A_list, cam_B_list,
                     cfg, state, v_thresh=None):
    """
    Counts cameras truly seeing (px, py) in 3D (FOV_H + FOV_V ≥ threshold).
    Tripod cameras count as 0.5.
    """
    if v_thresh is None:
        v_thresh = cfg.opt.vertical_coverage_threshold

    cam_A    = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B    = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)
    walk_y   = state["walk_y"]
    wall_segs = state["wall_segments"]
    obstacles = cfg.obstacles
    room_h   = cfg.ROOM_HEIGHT
    human_h  = cfg.HUMAN_HEIGHT
    human_fz = cfg.HUMAN_FOOT_Z
    human_hz = cfg.HUMAN_HEAD_Z
    k_dist   = cfg.DIST_QUALITY_K

    count = 0.0
    for (cx, cy, angle, orient, zh) in cam_A_list:
        fov_h, fov_v = get_fov(cam_A, orient)
        in_c, _ = point_in_wedge(px, py, cx, cy, angle, fov_h,
                                  cam_A.max_range, cam_A.min_range,
                                  wall_segs, obstacles, room_h, human_h, cam_z=zh)
        if in_c:
            ft = cam_fixed_tilt(cx, cy, zh, angle, walk_y, human_h)
            if vertical_body_coverage(cx, cy, zh, px, py, fov_v,
                                      human_h, human_fz, human_hz,
                                      v_thresh=v_thresh,
                                      fixed_tilt_rad=ft) >= v_thresh:
                count += 1.0

    if cam_B and cam_B_list:
        for (cx, cy, angle, orient, ih) in cam_B_list:
            fov_h, fov_v = get_fov(cam_B, orient)
            in_c, _ = point_in_wedge(px, py, cx, cy, angle, fov_h,
                                      cam_B.max_range, cam_B.min_range,
                                      wall_segs, obstacles, room_h, human_h, cam_z=ih)
            if in_c:
                if vertical_body_coverage(cx, cy, ih, px, py, fov_v,
                                          human_h, human_fz, human_hz,
                                          v_thresh=v_thresh,
                                          fixed_tilt_rad=None) >= v_thresh:
                    count += 0.5
    return count


def score_at_point(px, py, cam_A_list, cam_B_list, cfg, state):
    """
    Evaluates the exact quality score (including bilateral penalties)
    at a single point (px, py).
    """
    sample_points = [(px, py, 1.0)]
    score, _, _, _ = score_configuration(cam_A_list, cam_B_list, sample_points, cfg, state)
    return score


def find_best_sts_position(cam_A_list, cam_B_list, cfg, state,
                           step=0.20, wall_margin=0.7):
    """
    For a 'point' (STS) zone with auto_optimize, find the (x, y) spot inside the
    analysis-zone X-range that is best covered by the GIVEN camera configuration.

    Post-placement search on the full (wall + tripod) layout — answers
    "given these cameras, where is the ideal spot for the chair?". Returns (x, y),
    falling back to the configured STS position if nothing scores.
    """
    ax0 = state["analysis_x_start"]
    ax1 = state["analysis_x_end"]
    corners   = cfg.ROOM_CORNERS
    obstacles = cfg.obstacles
    room_h    = cfg.ROOM_HEIGHT
    wall_segs = state["wall_segments"]

    ys = [c[1] for c in corners]
    y0, y1 = min(ys), max(ys)

    best_pos   = (state["sts_x"], state["sts_y"])
    best_score = -1.0

    sx = ax0
    while sx <= ax1 + 1e-9:
        sy = y0 + wall_margin
        while sy <= y1 - wall_margin + 1e-9:
            if point_in_room(sx, sy, corners, obstacles, room_h):
                # keep clear of walls
                too_close = False
                for (wx1, wy1), (wx2, wy2) in wall_segs:
                    dx, dy = wx2 - wx1, wy2 - wy1
                    l2 = dx*dx + dy*dy
                    if l2 < 1e-9:
                        continue
                    t = max(0.0, min(1.0, ((sx-wx1)*dx + (sy-wy1)*dy) / l2))
                    if math.hypot(sx-(wx1+t*dx), sy-(wy1+t*dy)) < wall_margin:
                        too_close = True
                        break
                if not too_close:
                    s = score_at_point(sx, sy, cam_A_list, cam_B_list, cfg, state)
                    if s > best_score:
                        best_score = s
                        best_pos = (sx, sy)
            sy += step
        sx += step

    return best_pos


# =============================================================
# PRECOMPUTED COVERAGE  (performance — see ALGORITHM/#7)
# =============================================================

def precompute_coverage(cand_A, cand_B, sample_points, cfg, state):
    """
    Precompute, for each candidate, the per-sample-point contribution it makes
    where it usefully covers the point (in-cone + line-of-sight + v >= threshold).

    Within a combo the candidate set and the sample points are FIXED, so this
    geometry is identical every time a candidate is (re)scored during the search.
    Computing it ONCE here turns the thousands of later score evaluations into
    cheap table aggregations (score_indexed), instead of recomputing cones and
    line-of-sight millions of times.

    Returns (covA, covB, meta):
      covA[c] : dict {point_index: (angle, score_v, side, cx, cy)}        (wall)
      covB[c] : dict {point_index: [contrib, ...]}  (1 or 2 entries)      (tripod)
      meta    : {"weights": [...], "in_analysis": [...], "n_points": int}
    Indices in covA/covB match the order of cand_A/cand_B respectively.
    """
    cam_A     = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B_set = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)

    walk_y           = state["walk_y"]
    analysis_x_start = state["analysis_x_start"]
    analysis_x_end   = state["analysis_x_end"]
    v_thresh         = cfg.opt.vertical_coverage_threshold
    k_dist           = cfg.DIST_QUALITY_K
    wall_segs = state["wall_segments"]
    obstacles = cfg.obstacles
    room_h    = cfg.ROOM_HEIGHT
    human_h   = cfg.HUMAN_HEIGHT
    human_fz  = cfg.HUMAN_FOOT_Z
    human_hz  = cfg.HUMAN_HEAD_Z

    weights     = [w for (_, _, w) in sample_points]
    in_analysis = [analysis_x_start <= px <= analysis_x_end
                   for (px, py, _) in sample_points]

    covA = []
    for (cx, cy, angle, orient, zh) in cand_A:
        fov_h, fov_v = get_fov(cam_A, orient)
        d = {}
        for pi, (px, py, w) in enumerate(sample_points):
            in_cone, dist = point_in_wedge(
                px, py, cx, cy, angle, fov_h, cam_A.max_range, cam_A.min_range,
                wall_segs, obstacles, room_h, human_h, cam_z=zh)
            if not in_cone:
                continue
            fixed_tilt = cam_fixed_tilt(cx, cy, zh, angle, walk_y, human_h)
            v = vertical_body_coverage(cx, cy, zh, px, py, fov_v,
                                       human_h, human_fz, human_hz,
                                       v_thresh=v_thresh, fixed_tilt_rad=fixed_tilt)
            if v >= v_thresh:
                score_v = v * v * dist_quality(dist, k_dist) \
                          * cam_A.score_weight * cam_A.score_factor
                d[pi] = (angle, score_v, cam_side(cy, walk_y), cx, cy)
        covA.append(d)

    covB = []
    if cam_B_set:
        for (cx, cy, angle, orient, ih) in cand_B:
            fov_h, fov_v = get_fov(cam_B_set, orient)
            d = {}
            for pi, (px, py, w) in enumerate(sample_points):
                in_cone, dist = point_in_wedge(
                    px, py, cx, cy, angle, fov_h,
                    cam_B_set.max_range, cam_B_set.min_range,
                    wall_segs, obstacles, room_h, human_h, cam_z=ih)
                if not in_cone:
                    continue
                v = vertical_body_coverage(cx, cy, ih, px, py, fov_v,
                                           human_h, human_fz, human_hz,
                                           v_thresh=v_thresh, fixed_tilt_rad=None)
                if v >= v_thresh:
                    score_v = v * v * dist_quality(dist, k_dist) \
                              * cam_B_set.score_weight * cam_B_set.score_factor
                    own   = cam_side(cy, walk_y)
                    other = 'N' if own == 'S' else 'S'
                    cross_factor = max(0.0, 1.0 - abs(cy - walk_y) / 2.0)
                    entries = [(angle, score_v, own, cx, cy)]
                    if cross_factor > 0.01:
                        entries.append((angle, score_v * cross_factor, other, cx, cy))
                    d[pi] = entries
            covB.append(d)

    return covA, covB, {"weights": weights, "in_analysis": in_analysis,
                        "n_points": len(sample_points)}


def score_indexed(selA, selB, covA, covB, meta, cfg):
    """
    Exact configuration score from candidate INDICES, using the precomputed
    coverage tables. Produces the same result as score_configuration on the
    same points (both aggregate via _score_point).

    selA / selB : iterables of candidate indices into covA / covB.
    Returns (total_score, south_total, north_total).
    """
    from collections import defaultdict

    target_coverage  = cfg.TARGET_COVERAGE
    bilateral_weight = cfg.BILATERAL_WEIGHT
    weights      = meta["weights"]
    in_analysis  = meta["in_analysis"]

    pts = defaultdict(list)
    for c in selA:
        for pi, entry in covA[c].items():
            pts[pi].append(entry)
    for c in selB:
        for pi, entries in covB[c].items():
            pts[pi].extend(entries)

    total = south_total = north_total = 0.0
    for pi, contribs in pts.items():
        ps, sv, nv, _ = _score_point(contribs, weights[pi],
                                     target_coverage, bilateral_weight)
        total += ps
        if in_analysis[pi]:
            south_total += sv
            north_total += nv
    return total, south_total, north_total
