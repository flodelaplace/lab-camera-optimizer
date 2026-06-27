"""
combo_worker.py
Worker function for parallel combo processing.

This module is designed to be safely importable by multiprocessing worker
processes on Windows (no module-level side effects, no argparse, no GUI).

Each worker loads the YAML config independently (avoids pickle issues
with the _Ns class loaded via importlib in the main process).
"""

import copy
import os
import math
import warnings


# ─────────────────────────────────────────────────────────────────────────────
# Process-level shared state (set once per worker via init_worker)
# ─────────────────────────────────────────────────────────────────────────────
_worker_cfg = None


def init_worker(config_path):
    """
    Initialiser called once per worker process.
    Loads the config from YAML (avoids pickling _Ns across processes).
    """
    global _worker_cfg
    # Force non-interactive matplotlib backend in workers
    import matplotlib
    matplotlib.use("Agg")
    # Load config in each worker independently
    from core.config_loader import load_config
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        _worker_cfg = load_config(config_path)


# ─────────────────────────────────────────────────────────────────────────────
# Per-combo processing
# ─────────────────────────────────────────────────────────────────────────────

def process_combo(params):
    """
    Process a single zone combination.

    Parameters
    ----------
    params : dict
        combo_idx, combo_label, combo_label_short,
        corridor_x_start, corridor_x_end, zone_start, zone_end,
        zone_length, walk_y, poly_combo, has_corridor, room_cx, room_cy

    Returns
    -------
    dict with keys:
        combo_idx, combo_label, combo_label_short,
        best_cam_A, best_cam_B, best_score, best_sts,
        state, n_cam_A_cands, n_cam_B_cands, n_sample_points,
        skipped (bool)
    """
    cfg = copy.deepcopy(_worker_cfg)

    combo_idx        = params["combo_idx"]
    combo_label      = params["combo_label"]
    combo_label_short = params["combo_label_short"]
    has_corridor     = params["has_corridor"]
    walk_y           = params["walk_y"]
    corridor_x_start = params["corridor_x_start"]
    corridor_x_end   = params["corridor_x_end"]
    zone_start       = params["zone_start"]
    zone_end         = params["zone_end"]
    poly_combo       = params["poly_combo"]
    room_cx          = params["room_cx"]
    room_cy          = params["room_cy"]

    from core.room       import build_wall_segments
    from core.candidates import build_sample_points, generate_candidates
    from core.greedy     import greedy_place_cameras

    # ── Apply polygon translations ────────────────────────────────────────
    for item in poly_combo:
        zone_id, dx, dy = item
        for z in cfg.capture_zones:
            if z.id == zone_id:
                z._translated_vertices = [
                    (vx + dx, vy + dy) for (vx, vy) in z.vertices
                ]

    # ── Polygon-only: derive walk_y and zone bounds from translated verts ─
    if not has_corridor:
        best_z = max(
            (z for z in cfg.capture_zones if z.type == "polygon"),
            key=lambda z: z.priority,
            default=None,
        )
        if best_z is not None:
            _verts = getattr(best_z, "_translated_vertices", None) or best_z.vertices
            sts_x  = sum(v[0] for v in _verts) / len(_verts)
            sts_y  = sum(v[1] for v in _verts) / len(_verts)
            walk_y = sts_y
        else:
            sts_x, sts_y = room_cx, room_cy
            walk_y = room_cy

        all_verts = []
        for z in cfg.capture_zones:
            if z.type == "polygon":
                v = getattr(z, "_translated_vertices", None) or z.vertices
                all_verts.extend(v)
        if all_verts:
            corridor_x_start = min(v[0] for v in all_verts)
            corridor_x_end   = max(v[0] for v in all_verts)
            zone_start       = corridor_x_start
            zone_end         = corridor_x_end
        else:
            corridor_x_start = room_cx - 1.0
            corridor_x_end   = room_cx + 1.0
            zone_start       = corridor_x_start
            zone_end         = corridor_x_end
    else:
        sts_x = zone_start + params.get("zone_length", zone_end - zone_start) / 2.0
        sts_y = walk_y

    # ── Build state ───────────────────────────────────────────────────────
    wall_segs = build_wall_segments(cfg.ROOM_CORNERS, cfg.obstacles, cfg.ROOM_HEIGHT)
    state = {
        "walk_y":           walk_y,
        "walk_x_start":     corridor_x_start,
        "walk_x_end":       corridor_x_end,
        "analysis_x_start": zone_start,
        "analysis_x_end":   zone_end,
        "sts_x":            sts_x,
        "sts_y":            sts_y,
        "wall_segments":    wall_segs,
    }

    # ── Generate candidates and sample points ─────────────────────────────
    sample_points = build_sample_points(cfg, state)
    cam_A_cands, cam_B_cands = generate_candidates(
        cfg, state,
        wall_step        = cfg.WALL_STEP,
        angle_steps      = cfg.ANGLE_STEPS,
        tripod_grid_step = cfg.TRIPOD_GRID_STEP,
        sample_points    = sample_points,
    )

    # ── Run greedy optimisation (quiet — no tqdm in workers) ──────────────
    best_cam_A, best_cam_B, best_score, best_sts, restart_results = greedy_place_cameras(
        cam_A_cands, cam_B_cands,
        sample_points, cfg, state,
        n_restarts  = cfg.opt.restarts_per_combo,
        combo_label = combo_label,
        quiet       = True,
    )

    return {
        "combo_idx":        combo_idx,
        "combo_label":      combo_label,
        "combo_label_short": combo_label_short,
        "best_cam_A":       best_cam_A,
        "best_cam_B":       best_cam_B,
        "best_score":       best_score,
        "best_sts":         best_sts,
        "state":            state,
        "restart_results":  restart_results,
        "n_cam_A_cands":    len(cam_A_cands),
        "n_cam_B_cands":    len(cam_B_cands),
        "n_sample_points":  len(sample_points),
        "skipped":          False,
    }
