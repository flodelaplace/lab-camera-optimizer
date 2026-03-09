"""
greedy.py
Greedy optimisation with random restarts for camera placement.
Two algorithms available (set via cfg.opt.algo):
  - "greedy"      : pure greedy (original behaviour)
  - "greedy_1opt" : greedy init + 1-opt local search (better quality)
"""

import math
import random
import numpy as np
from tqdm import tqdm

from .room import (point_in_room, point_in_wedge, vertical_body_coverage,
                   wall_normal_at)
from .scoring import score_configuration, get_fov, dist_quality


# ─────────────────────────────────────────────────────────────────────────────
# 1-OPT LOCAL SEARCH
# ─────────────────────────────────────────────────────────────────────────────

def _coverage_ratio(cam, sample_points, cam_set, wall_segs, obstacles, room_h, human_h,
                    subsample=6):
    """
    Returns the fraction of sample_points (subsampled) that this camera sees.
    Used to penalise candidates whose footprint covers very little of the zone,
    regardless of how close they are (counteracts distance_quality bias).
    """
    cx, cy, angle, orient, zh = cam
    fov_h, _ = get_fov(cam_set, orient)
    pts = sample_points[::subsample]
    if not pts:
        return 0.0
    seen = sum(
        1 for (px, py, w) in pts
        if point_in_wedge(px, py, cx, cy, angle, fov_h,
                          cam_set.max_range, cam_set.min_range,
                          wall_segs, obstacles, room_h, human_h,
                          cam_z=zh)[0]
    )
    return seen / len(pts)


def _1opt(cur_A, cur_B, pool_A, sample_points, cfg, state,
          current_score, min_spacing_A, max_passes=3, pbar=None):
    """
    1-opt local search on cur_A.

    Each pass: iterate over cameras in RANDOM order (so different restarts
    explore different swap sequences), find the best replacement for each
    camera from pool_A. Accept if score improves.
    Repeat until no pass improves or max_passes reached.

    Returns (improved_cam_A, improved_score).
    """
    cam_A_set = next(c for c in cfg.camera_sets if c.mounting == "wall")
    wall_segs = state["wall_segments"]
    obstacles = cfg.obstacles
    room_h    = cfg.ROOM_HEIGHT
    human_h   = cfg.HUMAN_HEIGHT

    best_A     = list(cur_A)
    best_score = current_score

    for _pass in range(max_passes):
        improved = False
        # Randomise camera order each pass → different restarts explore differently
        indices = list(range(len(best_A)))
        random.shuffle(indices)

        for i in indices:
            others = [c for j, c in enumerate(best_A) if j != i]

            best_cand       = None
            best_cand_score = best_score   # only strict improvements

            for cand in tqdm(pool_A,
                             desc=f"    1-opt pass {_pass+1} cam {i+1}/{len(best_A)}",
                             leave=False,
                             ncols=80,
                             unit="cand"):
                cx, cy, angle, orient, zh = cand

                if any(math.sqrt((cx-c[0])**2 + (cy-c[1])**2) < min_spacing_A
                       for c in others):
                    continue

                fov_h, _ = get_fov(cam_A_set, orient)
                if not any(point_in_wedge(px, py, cx, cy, angle, fov_h,
                                          cam_A_set.max_range, cam_A_set.min_range,
                                          wall_segs, obstacles, room_h, human_h,
                                          cam_z=zh)[0]
                           for (px, py, w) in sample_points[::8]):
                    continue

                s, *_ = score_configuration(
                    others + [cand], cur_B, sample_points, cfg, state)

                # Penalise candidates with tiny footprint on the zone
                cov = _coverage_ratio(cand, sample_points, cam_A_set,
                                      wall_segs, obstacles, room_h, human_h)
                s *= cov  # zero if sees nothing, proportional otherwise

                if s > best_cand_score:
                    best_cand_score = s
                    best_cand       = cand

            if best_cand is not None:
                best_A     = others + [best_cand]
                best_score = best_cand_score
                improved   = True
                if pbar:
                    pbar.set_postfix_str(f"score={best_score:.1f}", refresh=True)

        if not improved:
            break

    return best_A, best_score


# ─────────────────────────────────────────────────────────────────────────────
# DIVERSE INITIALISATION  (farthest-point sampling)
# ─────────────────────────────────────────────────────────────────────────────

def _init_diverse(n_A, pool_A, sample_points, cfg, state, min_spacing_A):
    """
    Place n_A cameras using farthest-point sampling:
      - 1st camera: random valid candidate
      - Each next camera: the candidate that maximises the minimum
        distance to all already-placed cameras
        (and still passes the visibility pre-filter)

    This guarantees cameras are spread spatially around the room
    before any score-based refinement.

    Returns list of (x, y, angle, orient, height).
    """
    cam_A_set = next(c for c in cfg.camera_sets if c.mounting == "wall")
    wall_segs = state["wall_segments"]
    obstacles = cfg.obstacles
    room_h    = cfg.ROOM_HEIGHT
    human_h   = cfg.HUMAN_HEIGHT

    # Pre-filter: keep only candidates that see at least one sample point
    visible = [
        cam for cam in pool_A
        if any(point_in_wedge(px, py, cam[0], cam[1], cam[2],
                              get_fov(cam_A_set, cam[3])[0],
                              cam_A_set.max_range, cam_A_set.min_range,
                              wall_segs, obstacles, room_h, human_h,
                              cam_z=cam[4])[0]
               for (px, py, w) in sample_points[::6])
    ]
    if not visible:
        visible = pool_A

    placed = []

    # 1st camera: random pick from visible candidates
    placed.append(random.choice(visible))

    # Subsequent cameras: farthest from all placed so far
    for _ in range(n_A - 1):
        best_cand   = None
        best_min_d  = -1.0

        for cand in visible:
            cx, cy = cand[0], cand[1]

            # Spacing constraint
            if any(math.sqrt((cx-c[0])**2 + (cy-c[1])**2) < min_spacing_A
                   for c in placed):
                continue

            # Minimum distance to any already-placed camera
            min_d = min(math.sqrt((cx-c[0])**2 + (cy-c[1])**2) for c in placed)

            if min_d > best_min_d:
                best_min_d = min_d
                best_cand  = cand

        if best_cand is None:
            # All remaining candidates violate spacing — relax and take farthest
            for cand in visible:
                if cand in placed:
                    continue
                min_d = min(math.sqrt((cand[0]-c[0])**2 + (cand[1]-c[1])**2)
                            for c in placed)
                if min_d > best_min_d:
                    best_min_d = min_d
                    best_cand  = cand

        if best_cand:
            placed.append(best_cand)

    return placed



def greedy_place_cameras(cam_A_cands, cam_B_cands,
                         sample_points, cfg, state,
                         n_restarts=20,
                         combo_label="combo",
                         log=None,
                         on_record=None):
    """
    Greedy optimisation with random restarts.

    Algorithm selected via cfg.opt.algo:
      "greedy"      — pure greedy (fast)
      "greedy_1opt" — greedy init + 1-opt local search (better quality)

    Returns (best_cam_A, best_cam_B, best_score, best_sts_pos)
    """
    def _log(msg="", end="\n"):
        if log:   log(msg, end=end)
        else:     print(msg, end=end)

    algo = getattr(cfg.opt, "algo", "greedy_1opt")

    cam_A_set  = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B_set  = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)
    n_A        = cam_A_set.max_count
    n_B        = cam_B_set.max_count if cam_B_set else 0

    walk_y           = state["walk_y"]
    analysis_x_start = state["analysis_x_start"]
    analysis_x_end   = state["analysis_x_end"]
    sts_x_init       = state["sts_x"]
    sts_y_init       = state["sts_y"]
    wall_segs        = state["wall_segments"]
    obstacles        = cfg.obstacles
    room_h           = cfg.ROOM_HEIGHT
    human_h          = cfg.HUMAN_HEIGHT
    human_fz         = cfg.HUMAN_FOOT_Z
    human_hz         = cfg.HUMAN_HEAD_Z
    k_dist           = cfg.DIST_QUALITY_K
    v_thresh         = cfg.opt.vertical_coverage_threshold
    min_spacing_A    = cam_A_set.min_spacing
    min_spacing_B    = cam_B_set.min_spacing if cam_B_set else 1.5

    _log(f"\nGreedy optimisation ({n_restarts} restarts)  —  algo={algo}  bilateral ON")
    _log(f"  Cam A (wall) candidates : {len(cam_A_cands)}")
    _log(f"  Cam B (tripod) candidates : {len(cam_B_cands)}")
    _log(f"  Eval points             : {len(sample_points)}")
    _log(f"  Graph mode              : {cfg.opt.graph_mode}")

    best_score   = -1.0
    best_cam_A   = []
    best_cam_B   = []
    best_sts_pos = (sts_x_init, sts_y_init)
    no_improvement = 0

    # ── Outer progress bar — one tick per restart ─────────────────────────
    pbar = tqdm(total=n_restarts,
                desc="  Restarts",
                unit="restart",
                ncols=90,
                colour="cyan")

    for restart in range(n_restarts):
        pbar.set_description(
            f"  Restart {restart+1:2d}/{n_restarts}  best={best_score:.1f}")

        # pool_A : subsample for greedy init (fast)
        # pool_1opt drawn fresh per restart inside phase 1b (different each time)
        pool_A = random.sample(cam_A_cands, min(250, len(cam_A_cands)))
        pool_B = random.sample(cam_B_cands, min(150, len(cam_B_cands))) if cam_B_cands else []
        cur_A  = []
        cur_B  = []

        # ── PHASE 1: initial camera placement ────────────────────────────
        if algo == "greedy_1opt":
            cur_A = _init_diverse(n_A, pool_A, sample_points, cfg, state, min_spacing_A)
        else:
            # Pure greedy: sequential score-maximising
            for _ in range(n_A):
                best_add = -1.0; best_cam = None
                n_south  = sum(1 for c in cur_A if c[1] <  walk_y)
                n_north  = sum(1 for c in cur_A if c[1] >= walk_y)
                n_placed = n_south + n_north
                sub = random.sample(pool_A, min(100, len(pool_A)))
                for cam in sub:
                    cx, cy, angle, orient, zh = cam
                    if any(math.sqrt((cx-c[0])**2+(cy-c[1])**2) < min_spacing_A
                           for c in cur_A):
                        continue
                    fov_h, _ = get_fov(cam_A_set, orient)
                    if not any(point_in_wedge(px, py, cx, cy, angle, fov_h,
                                              cam_A_set.max_range, cam_A_set.min_range,
                                              wall_segs, obstacles, room_h, human_h,
                                              cam_z=zh)[0]
                               for (px, py, w) in sample_points[::6]):
                        continue
                    s, *_ = score_configuration(cur_A + [cam], [], sample_points, cfg, state)
                    # Penalise candidates with tiny footprint on the zone
                    cov = _coverage_ratio(cam, sample_points, cam_A_set,
                                          wall_segs, obstacles, room_h, human_h)
                    s *= cov
                    if n_placed >= 2 and cfg.BILATERAL_WEIGHT > 0:
                        cam_side_this = 'S' if cy < walk_y else 'N'
                        n_this  = n_south if cam_side_this == 'S' else n_north
                        n_other = n_north if cam_side_this == 'S' else n_south
                        if n_this > n_other:
                            ratio = (n_this + 1) / max(n_other + 1, 1)
                            s *= (1.0/math.sqrt(ratio))*cfg.BILATERAL_WEIGHT \
                                 + (1.0-cfg.BILATERAL_WEIGHT)
                    if s > best_add:
                        best_add = s; best_cam = cam
                if best_cam:
                    cur_A.append(best_cam)

        # ── PHASE 1b: 1-opt ───────────────────────────────────────────────
        if algo == "greedy_1opt" and cur_A:
            init_score, *_ = score_configuration(cur_A, [], sample_points, cfg, state)
            pool_1opt = random.sample(cam_A_cands, min(80, len(cam_A_cands)))
            cur_A, _ = _1opt(cur_A, cur_B, pool_1opt,
                             sample_points, cfg, state,
                             init_score, min_spacing_A, max_passes=2,
                             pbar=pbar)

        # ── Pre-STS score (cam_A only) — used only to decide record ──────
        pre_score, _, _, _ = score_configuration(
            cur_A, [], sample_points, cfg, state)
        is_record_pre = pre_score > best_score

        if is_record_pre:
            # ── FIND BEST STS POINT (only on new records — expensive grid search) ──
            WALL_MARGIN_STS = 0.7
            best_sts_score  = -1.0
            sts_x_opt, sts_y_opt = sts_x_init, sts_y_init
            room_corners = cfg.ROOM_CORNERS

            for sx in np.arange(analysis_x_start, analysis_x_end + 0.01, 0.25):
                for sy in np.arange(WALL_MARGIN_STS,
                                    room_h + WALL_MARGIN_STS + 0.01, 0.25):
                    if not point_in_room(sx, sy, room_corners, obstacles, room_h):
                        continue
                    too_close = False
                    for (wx1, wy1), (wx2, wy2) in wall_segs:
                        sdx, sdy = wx2-wx1, wy2-wy1
                        sl2 = sdx**2 + sdy**2
                        if sl2 < 1e-9: continue
                        t = max(0, min(1, ((sx-wx1)*sdx+(sy-wy1)*sdy)/sl2))
                        if math.sqrt((sx-(wx1+t*sdx))**2+(sy-(wy1+t*sdy))**2) < WALL_MARGIN_STS:
                            too_close = True; break
                    if too_close:
                        continue
                    sts_score_pt = 0.0
                    for (cx_z, cy_z, ang_z, ori_z, zh) in cur_A:
                        fov_h_z, fov_v_z = get_fov(cam_A_set, ori_z)
                        in_h, dist_z = point_in_wedge(
                            sx, sy, cx_z, cy_z, ang_z, fov_h_z,
                            cam_A_set.max_range, cam_A_set.min_range,
                            wall_segs, obstacles, room_h, human_h, cam_z=zh)
                        if not in_h: continue
                        dp = max(abs(cy_z - walk_y), 0.1)
                        ft = math.atan2(human_h/2.0 - zh, dp)
                        v = vertical_body_coverage(cx_z, cy_z, zh, sx, sy, fov_v_z,
                                                   human_h, human_fz, human_hz,
                                                   v_thresh=v_thresh,
                                                   fixed_tilt_rad=ft)
                        if v >= v_thresh:
                            sts_score_pt += v * v * dist_quality(dist_z, k_dist)
                    if sts_score_pt > best_sts_score:
                        best_sts_score = sts_score_pt
                        sts_x_opt, sts_y_opt = sx, sy

            best_sts_pos = (sts_x_opt, sts_y_opt)
        else:
            # Reuse best STS position found so far
            sts_x_opt, sts_y_opt = best_sts_pos

        # ── PHASE 2: cam_B placement — always run (uses best known STS) ──
        cur_B = []
        if cam_B_set and pool_B:
            for _ in range(n_B):
                best_add = -1.0; best_cam = None
                sub = random.sample(pool_B, min(150, len(pool_B)))
                for cam in sub:
                    cx, cy, angle, orient, ih = cam
                    if any(math.sqrt((cx-c[0])**2+(cy-c[1])**2) < min_spacing_B
                           for c in cur_B):
                        continue
                    fov_h, fov_v = get_fov(cam_B_set, orient)
                    sees_sts, _ = point_in_wedge(
                        sts_x_opt, sts_y_opt, cx, cy, angle, fov_h,
                        cam_B_set.max_range, cam_B_set.min_range,
                        wall_segs, obstacles, room_h, human_h, cam_z=ih)
                    if not sees_sts: continue
                    v_sts = vertical_body_coverage(
                        cx, cy, ih, sts_x_opt, sts_y_opt, fov_v,
                        human_h, human_fz, human_hz, v_thresh=v_thresh)
                    if v_sts < 0.8: continue
                    tri_penalty = 1.0
                    if cur_B:
                        ang_to_sts = math.degrees(math.atan2(sts_y_opt-cy, sts_x_opt-cx))
                        for ec in cur_B:
                            ea = math.degrees(math.atan2(sts_y_opt-ec[1], sts_x_opt-ec[0]))
                            diff = abs(ang_to_sts - ea) % 360
                            if diff > 180: diff = 360 - diff
                            if diff < 30 or diff > 150:
                                tri_penalty = 0.1; break
                    s_b, *_ = score_configuration(
                        [], cur_B + [cam], sample_points, cfg, state)
                    if s_b * tri_penalty > best_add:
                        best_add = s_b * tri_penalty; best_cam = cam
                if best_cam:
                    cur_B.append(best_cam)

        # ── Recompute final score including cam_B ─────────────────────────
        final_score, _, s_tot, n_tot = score_configuration(
            cur_A, cur_B, sample_points, cfg, state)
        bilat = min(s_tot, n_tot) / max(s_tot, n_tot, 1e-6) * 100

        if final_score > best_score:
            best_score     = final_score
            best_cam_A     = list(cur_A)
            best_cam_B     = list(cur_B)
            record_tag     = "  --> NEW RECORD"
            is_new_record  = True
            no_improvement = 0
        else:
            record_tag    = ""
            is_new_record = False

        # Update outer bar
        pbar.set_postfix({
            "score": f"{final_score:.1f}",
            "best":  f"{best_score:.1f}",
            "bal":   f"{bilat:.0f}%",
            "rec":   "★" if is_new_record else "",
        })
        pbar.update(1)

        # ── Early stop ────────────────────────────────────────────────────
        early_stop = getattr(cfg.opt, "early_stop", max(3, n_restarts // 3))
        if early_stop > 0 and no_improvement >= early_stop:
            _log(f"    Early stop after {no_improvement} restarts without improvement.")
            break

        # ── Graph callback ────────────────────────────────────────────────
        graph_mode = cfg.opt.graph_mode
        save_graph = (graph_mode == "all" or
                      (graph_mode in ("records_only", "best_per_combo") and is_new_record))
        if save_graph and on_record:
            on_record(restart + 1, cur_A, cur_B, final_score,
                      (sts_x_opt, sts_y_opt), is_new_record)

        _log(f"    - Attempt {restart+1:2d}: Score={final_score:6.3f} "
             f"(South={s_tot:.2f} | North={n_tot:.2f} | Bal={bilat:.0f}%)"
             f"  STS=({sts_x_opt:.1f},{sts_y_opt:.1f})"
             f"{record_tag}")
        _log(f"       Cam A ({len(cur_A)} cameras):")
        for zi, (zx, zy, za, zo, zh) in enumerate(cur_A):
            side = 'S' if zy < walk_y else 'N'
            d_perp = max(abs(zy - walk_y), 0.3)
            tilt_deg = math.degrees(math.atan2(human_h/2.0 - zh, d_perp))
            wn = wall_normal_at(zx, zy, wall_segs,
                                cfg.ROOM_CORNERS, cfg.ROOM_HEIGHT, obstacles)
            pan = (za - wn + 180) % 360 - 180
            pan_str = f"{abs(pan):.0f}deg {'RIGHT' if pan>1 else 'LEFT' if pan<-1 else 'STRAIGHT'}"
            _log(f"         Z{zi+1:2d} [{zo}][{side}]  "
                 f"({zx:.2f}m,{zy:.2f}m) h={zh:.1f}m  "
                 f"angle={za:.0f}deg  pan={pan_str}  tilt={abs(tilt_deg):.1f}deg down")

    pbar.close()
    return best_cam_A, best_cam_B, best_score, best_sts_pos

