"""
greedy.py
Unified camera-placement engine with random restarts.

Every enabled camera set (wall or tripod) goes through the SAME placement
routine — diverse farthest-point init → 1-opt local search with a coverage
penalty → angle reorientation. No camera set is special-cased.

Performance: within a combo the candidate set and the sample points are fixed,
so each candidate's per-point contribution is computed ONCE up front
(precompute_coverage) and the whole search runs on candidate INDICES, scored by
table aggregation (score_indexed) instead of recomputing cones/line-of-sight.
The reorientation pass and the decisive per-restart score still use the exact
geometry path (score_configuration) on off-grid angles / the full point set.

Two budget modes (driven by cfg):
  FIXED mode  (no optimization.total_cameras)
      Each set places exactly its count_max cameras. Block-coordinate: walls
      first, then tripods given the walls.
  FREE mode   (optimization.total_cameras = N)
      The optimiser allocates N cameras across the sets within their
      [count_min, count_max] bounds via marginal greedy over the UNION of
      candidate pools, then a combined 1-opt whose swaps may change a camera's
      set/type.

Algorithm per set selected via cfg.opt.algo: "greedy" | "greedy_1opt".
"""

import math
import random
import numpy as np
from tqdm import tqdm

from .room import (point_in_room, point_in_wedge, vertical_body_coverage,
                   wall_normal_at, wall_angular_limits, cam_fixed_tilt)
from .scoring import (score_configuration, get_fov,
                      precompute_coverage, score_indexed)


# ─────────────────────────────────────────────────────────────────────────────
# GEOMETRY-PATH SCORING HELPERS  (used by reorient / final score only)
# ─────────────────────────────────────────────────────────────────────────────

def _score_full(cam_A, cam_B, sample_points, cfg, state):
    s, *_ = score_configuration(cam_A, cam_B, sample_points, cfg, state)
    return s


def _score_role(role, lst, fixed_A, fixed_B, sample_points, cfg, state):
    if role == 'A':
        s, *_ = score_configuration(lst, fixed_B, sample_points, cfg, state)
    else:
        s, *_ = score_configuration(fixed_A, lst, sample_points, cfg, state)
    return s


# ─────────────────────────────────────────────────────────────────────────────
# INDEX-BASED SCORING (via precomputed coverage tables)
# ─────────────────────────────────────────────────────────────────────────────

def _score_idx(role, sel, fixedA_sel, fixedB_sel, covA, covB, meta, cfg):
    """Score this role's selection (by index) with the other role held fixed."""
    if role == 'A':
        s, _, _ = score_indexed(sel, fixedB_sel, covA, covB, meta, cfg)
    else:
        s, _, _ = score_indexed(fixedA_sel, sel, covA, covB, meta, cfg)
    return s


def _cov_ratio_idx(c, cov, n_pts):
    """Fraction of sample points this candidate usefully covers (table lookup)."""
    return (len(cov[c]) / n_pts) if n_pts else 0.0


# ─────────────────────────────────────────────────────────────────────────────
# DIVERSE INITIALISATION  (farthest-point sampling) — index based
# ─────────────────────────────────────────────────────────────────────────────

def _init_diverse_idx(n, pool_idx, cands, cov, min_spacing):
    """Place n candidates by farthest-point sampling (indices into `cands`)."""
    if n <= 0 or not pool_idx:
        return []
    visible = [c for c in pool_idx if cov[c]]
    if not visible:
        visible = list(pool_idx)

    placed = [random.choice(visible)]
    for _ in range(n - 1):
        best, best_d = None, -1.0
        for c in visible:
            cx, cy = cands[c][0], cands[c][1]
            if any(math.hypot(cx-cands[p][0], cy-cands[p][1]) < min_spacing
                   for p in placed):
                continue
            md = min(math.hypot(cx-cands[p][0], cy-cands[p][1]) for p in placed)
            if md > best_d:
                best_d, best = md, c
        if best is None:
            for c in visible:
                if c in placed:
                    continue
                md = min(math.hypot(cands[c][0]-cands[p][0],
                                    cands[c][1]-cands[p][1]) for p in placed)
                if md > best_d:
                    best_d, best = md, c
        if best is not None:
            placed.append(best)
    return placed


# ─────────────────────────────────────────────────────────────────────────────
# SEQUENTIAL GREEDY FILL (algo == "greedy") — index based
# ─────────────────────────────────────────────────────────────────────────────

def _greedy_fill_idx(role, n, pool_idx, cands, cov, covA, covB, meta, cfg,
                     fixedA_sel, fixedB_sel, min_spacing):
    n_pts = meta["n_points"]
    cur = []
    for _ in range(n):
        best_add, best = -1.0, None
        sub = random.sample(pool_idx, min(120, len(pool_idx)))
        for c in sub:
            cx, cy = cands[c][0], cands[c][1]
            if any(math.hypot(cx-cands[o][0], cy-cands[o][1]) < min_spacing
                   for o in cur):
                continue
            if not cov[c]:
                continue
            if role == 'A':
                s = _score_idx('A', cur + [c], None, fixedB_sel, covA, covB, meta, cfg)
            else:
                s = _score_idx('B', cur + [c], fixedA_sel, None, covA, covB, meta, cfg)
            s *= _cov_ratio_idx(c, cov, n_pts)
            if s > best_add:
                best_add, best = s, c
        if best is not None:
            cur.append(best)
    return cur


# ─────────────────────────────────────────────────────────────────────────────
# 1-OPT LOCAL SEARCH — index based
# ─────────────────────────────────────────────────────────────────────────────

def _1opt_idx(role, cur_sel, cands, cov, covA, covB, meta, cfg,
              fixedA_sel, fixedB_sel, pool_idx, current_score, min_spacing,
              max_passes=2, pbar=None, quiet=False):
    n_pts = meta["n_points"]
    best, best_score = list(cur_sel), current_score

    for _pass in range(max_passes):
        improved = False
        order = list(range(len(best)))
        random.shuffle(order)
        for i in order:
            others = [c for j, c in enumerate(best) if j != i]
            best_cand, best_cand_score = None, best_score
            for cand in tqdm(pool_idx,
                             desc=f"    1-opt[{role}] pass {_pass+1} cam {i+1}/{len(best)}",
                             leave=False, ncols=80, unit="cand", disable=quiet):
                cx, cy = cands[cand][0], cands[cand][1]
                if any(math.hypot(cx-cands[o][0], cy-cands[o][1]) < min_spacing
                       for o in others):
                    continue
                if not cov[cand]:
                    continue
                sel_try = others + [cand]
                if role == 'A':
                    s = _score_idx('A', sel_try, None, fixedB_sel, covA, covB, meta, cfg)
                else:
                    s = _score_idx('B', sel_try, fixedA_sel, None, covA, covB, meta, cfg)
                s *= _cov_ratio_idx(cand, cov, n_pts)
                if s > best_cand_score:
                    best_cand_score, best_cand = s, cand
            if best_cand is not None:
                best = others + [best_cand]
                best_score = best_cand_score
                improved = True
                if pbar:
                    pbar.set_postfix_str(f"score={best_score:.1f}", refresh=True)
        if not improved:
            break
    return best, best_score


def _place_set_idx(role, n, all_idx, cands, cov, covA, covB, meta, cfg,
                   fixedA_sel, fixedB_sel, algo, min_spacing,
                   pbar=None, quiet=False):
    """Place n cameras of one set (index based), other set held fixed."""
    if n <= 0 or not all_idx:
        return []
    pool_init = random.sample(all_idx, min(250, len(all_idx)))
    if algo == "greedy_1opt":
        cur = _init_diverse_idx(n, pool_init, cands, cov, min_spacing)
        if cur:
            init = _score_idx(role, cur, fixedA_sel, fixedB_sel, covA, covB, meta, cfg)
            pool_1opt = random.sample(all_idx, min(80, len(all_idx)))
            cur, _ = _1opt_idx(role, cur, cands, cov, covA, covB, meta, cfg,
                               fixedA_sel, fixedB_sel, pool_1opt, init, min_spacing,
                               max_passes=2, pbar=pbar, quiet=quiet)
        return cur
    return _greedy_fill_idx(role, n, pool_init, cands, cov, covA, covB, meta, cfg,
                            fixedA_sel, fixedB_sel, min_spacing)


# ─────────────────────────────────────────────────────────────────────────────
# FREE-ALLOCATION MODE — index based
# ─────────────────────────────────────────────────────────────────────────────

def _free_bounds(wall_set, tripod_set):
    cmin = {'A': wall_set.count_min if wall_set else 0,
            'B': tripod_set.count_min if tripod_set else 0}
    cmax = {'A': wall_set.count_max if wall_set else 0,
            'B': tripod_set.count_max if tripod_set else 0}
    spc  = {'A': wall_set.min_spacing if wall_set else 1.0,
            'B': tripod_set.min_spacing if tripod_set else 1.0}
    return cmin, cmax, spc


def _marginal_greedy_free_idx(roles, candsR, covR, covA, covB, meta, cfg,
                              cmin, cmax, spc, total):
    """Allocate `total` cameras across roles by marginal gain. Returns (selA, selB)."""
    n_pts = meta["n_points"]
    sel = {'A': [], 'B': []}
    cnt = {'A': 0, 'B': 0}
    allidx = {r: list(range(len(candsR[r]))) for r in roles}

    n_placed = 0
    while n_placed < total:
        remaining = total - n_placed
        forced = sum(max(0, cmin[r] - cnt[r]) for r in roles)
        below  = [r for r in roles if cnt[r] < cmin[r]]
        eligible = below if (below and remaining <= forced) \
                   else [r for r in roles if cnt[r] < cmax[r]]
        if not eligible:
            break

        best_metric, best = -1.0, None
        for r in eligible:
            pool = random.sample(allidx[r], min(150, len(allidx[r])))
            cands, cov = candsR[r], covR[r]
            for c in pool:
                cx, cy = cands[c][0], cands[c][1]
                if any(math.hypot(cx-cands[o][0], cy-cands[o][1]) < spc[r]
                       for o in sel[r]):
                    continue
                if not cov[c]:
                    continue
                if r == 'A':
                    s, _, _ = score_indexed(sel['A'] + [c], sel['B'],
                                            covA, covB, meta, cfg)
                else:
                    s, _, _ = score_indexed(sel['A'], sel['B'] + [c],
                                            covA, covB, meta, cfg)
                metric = s * _cov_ratio_idx(c, cov, n_pts)
                if metric > best_metric:
                    best_metric, best = metric, (r, c)
        if best is None:
            break
        r, c = best
        sel[r].append(c)
        cnt[r] += 1
        n_placed += 1

    return sel['A'], sel['B']


def _combined_1opt_free_idx(roles, candsR, covR, covA, covB, meta, cfg,
                            cmin, cmax, spc, selA, selB, max_passes=2):
    """Combined 1-opt with type-changing swaps (index based). Returns (selA, selB)."""
    sel = {'A': list(selA), 'B': list(selB)}

    def full():
        s, _, _ = score_indexed(sel['A'], sel['B'], covA, covB, meta, cfg)
        return s

    best_score = full()
    for _pass in range(max_passes):
        improved = False
        slots = [(r, k) for r in roles for k in range(len(sel[r]))]
        random.shuffle(slots)
        for (r, k) in slots:
            if k >= len(sel[r]):
                continue
            src_wo = [c for kk, c in enumerate(sel[r]) if kk != k]
            best_repl, best_repl_score = None, best_score
            for t in roles:
                if t != r:
                    if len(sel[r]) - 1 < cmin[r]:
                        continue
                    if len(sel[t]) + 1 > cmax[t]:
                        continue
                tgt_list = src_wo if t == r else sel[t]
                cands, cov = candsR[t], covR[t]
                pool = random.sample(range(len(cands)), min(80, len(cands)))
                for c in pool:
                    cx, cy = cands[c][0], cands[c][1]
                    if any(math.hypot(cx-cands[o][0], cy-cands[o][1]) < spc[t]
                           for o in tgt_list):
                        continue
                    if not cov[c]:
                        continue
                    trialA = list(sel['A'])
                    trialB = list(sel['B'])
                    if r == 'A':
                        trialA = [x for kk, x in enumerate(sel['A']) if kk != k]
                    else:
                        trialB = [x for kk, x in enumerate(sel['B']) if kk != k]
                    if t == 'A':
                        trialA = trialA + [c]
                    else:
                        trialB = trialB + [c]
                    s, _, _ = score_indexed(trialA, trialB, covA, covB, meta, cfg)
                    if s > best_repl_score:
                        best_repl_score, best_repl = s, (t, c)
            if best_repl is not None:
                t, c = best_repl
                sel[r] = [x for kk, x in enumerate(sel[r]) if kk != k]
                sel[t] = sel[t] + [c]
                best_score = best_repl_score
                improved = True
        if not improved:
            break
    return sel['A'], sel['B']


# ─────────────────────────────────────────────────────────────────────────────
# ANGLE REORIENTATION — geometry path (wall = aperture-limited, tripod = free)
# ─────────────────────────────────────────────────────────────────────────────

def _reorient(role, cur, cam_set, fixed_A, fixed_B,
              sample_points, cfg, state, current_score):
    """Refine each camera's angle (geometry scoring on off-grid angles)."""
    wall_segs = state["wall_segments"]
    is_wall   = (cam_set.mounting == "wall")
    best, best_score = list(cur), current_score

    for i in range(len(best)):
        cx, cy, cur_angle, orient, zh = best[i]
        fov_h, _ = get_fov(cam_set, orient)

        if is_wall:
            wall_lim = wall_angular_limits(cx, cy, wall_segs)
            (lim_left, _), (lim_right, _) = wall_lim
            if lim_left is None:
                continue
            half = fov_h / 2.0
            sin_m = (math.sin(math.radians(lim_left))
                     + math.sin(math.radians(lim_right))) / 2.0
            cos_m = (math.cos(math.radians(lim_left))
                     + math.cos(math.radians(lim_right))) / 2.0
            mid = math.degrees(math.atan2(sin_m, cos_m))

            def _rel(a, ref=mid):
                return (a - ref + 180) % 360 - 180

            lo = min(_rel(lim_left), _rel(lim_right))
            hi = max(_rel(lim_left), _rel(lim_right))
            if not (lo <= 0 <= hi):
                continue
            angle_lo, angle_hi = lo + half, hi - half
            if angle_lo > angle_hi:
                offsets = [lo + half, hi - half]
            else:
                offsets = list(np.arange(angle_lo, angle_hi + 0.1, 3.0))
            test_angles = [mid + off for off in offsets]
        else:
            test_angles = [cur_angle + d for d in range(-40, 41, 5)]

        best_angle = cur_angle
        for ta in test_angles:
            test = list(best)
            test[i] = (cx, cy, ta, orient, zh)
            s = _score_role(role, test, fixed_A, fixed_B, sample_points, cfg, state)
            if s > best_score:
                best_score, best_angle = s, ta
        best[i] = (cx, cy, best_angle, orient, zh)

    return best, best_score


# ─────────────────────────────────────────────────────────────────────────────
# MAIN ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def greedy_place_cameras(cam_A_cands, cam_B_cands,
                         sample_points, cfg, state,
                         n_restarts=20,
                         combo_label="combo",
                         log=None,
                         on_record=None,
                         quiet=False):
    """
    Unified placement with random restarts.

    Returns (best_cam_A, best_cam_B, best_score, sts_pos).
    `sts_pos` is the configured STS location — kept for reporting only.
    """
    def _log(msg="", end="\n"):
        if quiet:
            return
        if log:  log(msg, end=end)
        else:    print(msg, end=end)

    algo      = getattr(cfg.opt, "algo", "greedy_1opt")
    free_mode = bool(getattr(cfg, "FREE_MODE", False))
    walk_y    = state["walk_y"]
    sts_pos   = (state["sts_x"], state["sts_y"])

    wall_set   = next((c for c in cfg.camera_sets if c.mounting == "wall"),   None)
    tripod_set = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)

    # Search scores on a subsample of points (≤ ~150) for speed; the decisive
    # per-restart score is recomputed on the FULL set, so the kept score is exact.
    _SEARCH_PTS_TARGET = 150
    if len(sample_points) > _SEARCH_PTS_TARGET:
        _stride = math.ceil(len(sample_points) / _SEARCH_PTS_TARGET)
        search_points = sample_points[::_stride]
    else:
        search_points = sample_points

    # ── Precompute coverage ONCE for this combo (on the search points) ────
    covA, covB, meta = precompute_coverage(cam_A_cands, cam_B_cands,
                                           search_points, cfg, state)

    have_A = bool(wall_set and wall_set.count_max > 0 and cam_A_cands)
    have_B = bool(tripod_set and tripod_set.count_max > 0 and cam_B_cands)
    roles  = [r for r, ok in (('A', have_A), ('B', have_B)) if ok]
    candsR = {'A': cam_A_cands, 'B': cam_B_cands}
    covR   = {'A': covA,        'B': covB}

    mode_str = (f"FREE (total={cfg.TOTAL_CAMERAS})" if free_mode else "FIXED")
    _log(f"\nUnified placement ({n_restarts} restarts)  —  algo={algo}  mode={mode_str}")
    _log(f"  Wall (cam_A) candidates   : {len(cam_A_cands)}")
    _log(f"  Tripod (cam_B) candidates : {len(cam_B_cands)}")
    _log(f"  Eval points               : {len(sample_points)} "
         f"(search on {len(search_points)})")
    _log(f"  Graph mode                : {cfg.opt.graph_mode}")

    best_score, best_cam_A, best_cam_B = -1.0, [], []
    no_improvement = 0
    restart_results = []   # every restart's final config (for consensus analysis)

    pbar = tqdm(total=n_restarts, desc="  Restarts", unit="restart",
                ncols=90, colour="cyan", disable=quiet)

    for restart in range(n_restarts):
        pbar.set_description(
            f"  Restart {restart+1:2d}/{n_restarts}  best={best_score:.1f}")

        if free_mode:
            cmin, cmax, spc = _free_bounds(wall_set, tripod_set)
            selA, selB = _marginal_greedy_free_idx(
                roles, candsR, covR, covA, covB, meta, cfg,
                cmin, cmax, spc, cfg.TOTAL_CAMERAS)
            if algo == "greedy_1opt":
                selA, selB = _combined_1opt_free_idx(
                    roles, candsR, covR, covA, covB, meta, cfg,
                    cmin, cmax, spc, selA, selB, max_passes=2)
        else:
            selA, selB = [], []
            if have_A:
                selA = _place_set_idx('A', wall_set.count_max,
                                      list(range(len(cam_A_cands))), cam_A_cands,
                                      covA, covA, covB, meta, cfg, [], selB, algo,
                                      wall_set.min_spacing, pbar=pbar, quiet=quiet)
            if have_B:
                selB = _place_set_idx('B', tripod_set.count_max,
                                      list(range(len(cam_B_cands))), cam_B_cands,
                                      covB, covA, covB, meta, cfg, selA, [], algo,
                                      tripod_set.min_spacing, pbar=pbar, quiet=quiet)

        cur_A = [cam_A_cands[i] for i in selA]
        cur_B = [cam_B_cands[i] for i in selB]

        # ── Reorient (geometry path, off-grid angles) ────────────────────
        if cur_A and wall_set:
            sc = _score_full(cur_A, cur_B, search_points, cfg, state)
            cur_A, _ = _reorient('A', cur_A, wall_set, [], cur_B,
                                 search_points, cfg, state, sc)
        if cur_B and tripod_set:
            sc = _score_full(cur_A, cur_B, search_points, cfg, state)
            cur_B, _ = _reorient('B', cur_B, tripod_set, cur_A, [],
                                 search_points, cfg, state, sc)

        # ── Decisive score on the FULL point set (exact geometry) ────────
        final_score, _, s_tot, n_tot = score_configuration(
            cur_A, cur_B, sample_points, cfg, state)
        bilat = min(s_tot, n_tot) / max(s_tot, n_tot, 1e-6) * 100

        restart_results.append({"score": final_score,
                                "cam_A": list(cur_A), "cam_B": list(cur_B)})

        if final_score > best_score:
            best_score, best_cam_A, best_cam_B = final_score, list(cur_A), list(cur_B)
            is_new_record, no_improvement = True, 0
        else:
            is_new_record = False
            no_improvement += 1

        pbar.set_postfix({"score": f"{final_score:.1f}", "best": f"{best_score:.1f}",
                          "bal": f"{bilat:.0f}%", "rec": "★" if is_new_record else ""})
        pbar.update(1)

        graph_mode = cfg.opt.graph_mode
        save_graph = (graph_mode == "all" or
                      (graph_mode in ("records_only", "best_per_combo") and is_new_record))
        if save_graph and on_record:
            on_record(restart + 1, cur_A, cur_B, final_score, sts_pos, is_new_record)

        _log(f"    - Attempt {restart+1:2d}: Score={final_score:6.3f} "
             f"(South={s_tot:.2f} | North={n_tot:.2f} | Bal={bilat:.0f}%)"
             f"  cams: {len(cur_A)} wall + {len(cur_B)} tripod"
             f"{'  --> NEW RECORD' if is_new_record else ''}")
        if cur_A and wall_set:
            obstacles = cfg.obstacles
            for zi, (zx, zy, za, zo, zh) in enumerate(cur_A):
                side   = 'S' if zy < walk_y else 'N'
                tilt   = math.degrees(cam_fixed_tilt(zx, zy, zh, za, walk_y, cfg.HUMAN_HEIGHT))
                wn     = wall_normal_at(zx, zy, state["wall_segments"],
                                        cfg.ROOM_CORNERS, cfg.ROOM_HEIGHT, obstacles)
                pan    = (za - wn + 180) % 360 - 180
                pan_s  = f"{abs(pan):.0f}deg {'RIGHT' if pan>1 else 'LEFT' if pan<-1 else 'STRAIGHT'}"
                _log(f"         A{zi+1:2d} [{zo}][{side}]  ({zx:.2f}m,{zy:.2f}m) "
                     f"h={zh:.1f}m  angle={za:.0f}deg  pan={pan_s}  "
                     f"tilt={abs(tilt):.1f}deg down")
        if cur_B and tripod_set:
            for zi, (zx, zy, za, zo, zh) in enumerate(cur_B):
                side = 'S' if zy < walk_y else 'N'
                _log(f"         B{zi+1:2d} [{zo}][{side}]  ({zx:.2f}m,{zy:.2f}m) "
                     f"h={zh:.1f}m  angle={za:.0f}deg")

        early_stop = getattr(cfg.opt, "early_stop", 0) or 0
        if early_stop > 0 and no_improvement >= early_stop:
            _log(f"    Early stop after {no_improvement} restarts without improvement.")
            break

    pbar.close()
    return best_cam_A, best_cam_B, best_score, sts_pos, restart_results
