"""
scoring.py
Score configuration: coverage, bilateral constraint, angular diversity.
All functions receive explicit context (cfg, state) — no global state.
"""

import math
from .room import (vertical_body_coverage, point_in_wedge,
                   cam_fixed_tilt, cam_side)


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
                score_v = v * v * q * cam_A.score_weight
                side = cam_side(cy, walk_y)
                cam_contribs.append((angle, score_v, side, cx, cy))

        # ── Tripod cameras (cam_B) ───────────────────────────────────────
        if cam_B_set and cam_B_list:
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
                    score_v = v * v * q * cam_B_set.score_weight * 0.5
                    cam_contribs.append((angle, score_v, 'S', cx, cy))
                    cam_contribs.append((angle, score_v, 'N', cx, cy))

        if not cam_contribs:
            coverage_list.append(0)
            continue

        # ── 1. Effective camera count with angular+positional diversity ──
        # Sort by angle, then apply two diversity rules:
        #   a) Angular rule: cameras <20° apart count as 0.3 (unchanged)
        #   b) Proximity+angle rule: if two cameras are close in XY AND
        #      similar in angle, the second one is heavily discounted.
        #      But close + opposite angle = full credit (different view).
        sa = sorted(cam_contribs, key=lambda c: c[0])   # sort by angle

        effective_n = 0.0
        last_angle  = sa[0][0]
        last_x, last_y = sa[0][3], sa[0][4]
        effective_n = 1.0

        for i in range(1, len(sa)):
            angle_i  = sa[i][0]
            cx_i, cy_i = sa[i][3], sa[i][4]

            ang_diff = abs((angle_i - last_angle + 180) % 360 - 180)
            pos_dist = math.sqrt((cx_i - last_x)**2 + (cy_i - last_y)**2)

            if ang_diff < 20:
                # Angularly near-identical → weak contribution
                effective_n += 0.3
            else:
                # Check position+angle redundancy:
                # Close in position AND similar angle → penalise
                # Close in position but opposite angle → OK (good diversity)
                # Far in position → full credit regardless of angle
                if pos_dist < 2.0 and ang_diff < 60:
                    # Close AND same general direction → partially redundant
                    proximity_factor = (pos_dist / 2.0)         # 0=touching → 1=far
                    angle_factor     = (ang_diff / 60.0)         # 0=same → 1=different
                    effective_n += 0.3 + 0.7 * max(proximity_factor, angle_factor)
                else:
                    effective_n += 1.0
                last_angle  = angle_i
                last_x, last_y = cx_i, cy_i

        # ── 2. sum_v with diminishing returns (replaces avg_v) ───────────
        # Each additional camera at this point adds less and less.
        # Uses sqrt-like saturation: sum_v = Σ score_v_i / sqrt(i)
        # This rewards 3 cameras over 1, but avoids 10 cameras being 10× better.
        sorted_scores = sorted([c[1] for c in cam_contribs], reverse=True)
        sum_v = sum(s / math.sqrt(i + 1) for i, s in enumerate(sorted_scores))
        # Normalise so a single perfect camera (score_v=1.0) gives base=1.0
        sum_v_norm = sum_v / math.sqrt(target_coverage)

        base = min(effective_n, target_coverage) / target_coverage * sum_v_norm

        # ── 3. Angular bonus (unchanged) ─────────────────────────────────
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

        # ── 4. Bilateral factor — sum per side, not max ───────────────────
        # Accumulate contributions per side with sqrt saturation.
        # → one good camera per side gives bilateral ≈ same as before
        # → two good cameras per side gives a meaningfully better bilateral
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
        total_score   += point_score
        coverage_list.append(len(cam_contribs))

        if analysis_x_start <= px <= analysis_x_end:
            south_total += south_v
            north_total += north_v

    return total_score, coverage_list, south_total, north_total


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
            dp = max(abs(cy - walk_y), 0.1)
            ft = math.atan2(human_h / 2.0 - zh, dp)
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

