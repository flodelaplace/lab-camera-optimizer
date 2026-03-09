"""
optimize.py
Entry point for the Lab Camera Optimizer.

Usage:
    python optimize.py --config configs/labo_CHU.yaml
    python optimize.py                          # uses labo_CHU.yaml by default
"""

import sys
import argparse
import os
import math
import datetime
import importlib.util

# ── Resolve paths ─────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_CORE = os.path.join(_HERE, "core")
sys.path.insert(0, _HERE)

# ── Load config_loader via importlib (avoids sys.path pollution) ──────────────
def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

config_loader = _load_module("config_loader", os.path.join(_CORE, "config_loader.py"))

# ── Parse CLI argument ────────────────────────────────────────────────────────
_parser = argparse.ArgumentParser(
    description="Lab Camera Optimizer — place cameras optimally in a biomechanics lab.")
_parser.add_argument(
    "--config",
    default=os.path.join(_HERE, "configs", "example_simple.yaml"),
    help="Path to YAML configuration file (default: configs/example_simple.yaml)")
_parser.add_argument(
    "--no-preview",
    action="store_true",
    help="Skip the room layout preview shown before optimisation starts.")
_args = _parser.parse_args()

CFG = config_loader.load_config(_args.config)

# ── Room preview ──────────────────────────────────────────────────────────────
# Displays a top-down view of the room, obstacles and capture zones so the
# user can verify the layout before the (potentially long) optimisation runs.
# Disable with --no-preview (useful for batch / headless runs).
if not _args.no_preview:
    from preview_room import draw_room as _draw_room
    _cfg_name = os.path.splitext(os.path.basename(_args.config))[0]
    _preview_path = os.path.join(_HERE, "outputs", "preview_room",
                                 f"{_cfg_name}.png")
    CFG._yaml_path = _args.config
    print(f"\n[Preview] Room layout saved to: {_preview_path}")
    print("[Preview] Close the window (or press Ctrl+C) to start optimisation.\n")
    _draw_room(CFG, save_path=_preview_path, show=True)

# ── Import core modules ───────────────────────────────────────────────────────
from core.room       import build_wall_segments, wall_normal_at
from core.candidates import build_sample_points, generate_candidates
from core.scoring    import score_configuration
from core.greedy     import greedy_place_cameras
from core.visualize  import visualize_solution


# =============================================================
# LOGGER
# =============================================================

class Logger:
    def __init__(self):
        self._file = None

    def init(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True) if os.path.dirname(path) else None
        self._file = open(path, 'w', encoding='utf-8')
        self.log(f"=== CAMERA OPTIMISATION LOG  —  "
                 f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===")
        self.log(f"=== Config: {_args.config} ===\n")

    def log(self, msg="", end="\n"):
        print(msg, end=end)
        if self._file:
            self._file.write(msg + end)
            self._file.flush()

    def close(self):
        if self._file:
            self._file.close()
            self._file = None

LOG = Logger()


# =============================================================
# BUILD STATE DICT  (replaces mutated globals)
# =============================================================

def build_state(cfg, walk_y, walk_x_start, walk_x_end,
                analysis_x_start, analysis_x_end,
                sts_x, sts_y):
    """
    Returns a state dict with all zone-dependent values.
    Passed explicitly to all core functions — no globals() mutation.
    """
    wall_segs = build_wall_segments(cfg.ROOM_CORNERS, cfg.obstacles, cfg.ROOM_HEIGHT)
    return {
        "walk_y":           walk_y,
        "walk_x_start":     walk_x_start,
        "walk_x_end":       walk_x_end,
        "analysis_x_start": analysis_x_start,
        "analysis_x_end":   analysis_x_end,
        "sts_x":            sts_x,
        "sts_y":            sts_y,
        "wall_segments":    wall_segs,
    }


# =============================================================
# PRINT RESULTS
# =============================================================

def print_results(cam_A_list, cam_B_list, score, cfg, state):
    _, _, south_s, north_s = score_configuration(cam_A_list, cam_B_list,
                                                  build_sample_points(cfg, state),
                                                  cfg, state)
    total_s = max(south_s + north_s, 1e-6)
    bal     = min(south_s, north_s) / max(south_s, north_s, 1e-6)

    cam_A = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)
    walk_y = state["walk_y"]
    wall_segs = state["wall_segments"]

    LOG.log("\n" + "="*65)
    LOG.log("  OPTIMAL CONFIGURATION")
    LOG.log("="*65)
    LOG.log(f"Score: {score:.2f}")
    LOG.log(f"Bilateral:  SOUTH={south_s:.1f} ({100*south_s/total_s:.0f}%)  "
            f"NORTH={north_s:.1f} ({100*north_s/total_s:.0f}%)  "
            f"balance={bal*100:.0f}% "
            f"{'OK' if bal > 0.6 else 'UNBALANCED'}\n")

    LOG.log(f"{cam_A.name} cameras:")
    for i, (x, y, angle, orient, zh) in enumerate(cam_A_list):
        side      = 'S' if y < walk_y else 'N'
        d_perp    = max(abs(y - walk_y), 0.3)
        tilt_deg  = math.degrees(math.atan2(cfg.HUMAN_HEIGHT / 2.0 - zh, d_perp))
        wall_n    = wall_normal_at(x, y, wall_segs, cfg.ROOM_CORNERS, cfg.ROOM_HEIGHT, cfg.obstacles)
        pan_deg   = (angle - wall_n + 180) % 360 - 180
        pan_lbl   = (f"{abs(pan_deg):.0f}deg to the RIGHT" if pan_deg > 1
                     else f"{abs(pan_deg):.0f}deg to the LEFT" if pan_deg < -1
                     else "0deg (straight)")
        LOG.log(f"  A{i+1:2d} [{orient}][{side}]  pos=({x:.2f}m, {y:.2f}m)  h={zh:.1f}m")
        LOG.log(f"       Pan: {pan_lbl}  |  Tilt: {abs(tilt_deg):.1f}deg downward")

    if cam_B and cam_B_list:
        LOG.log(f"\n{cam_B.name} cameras:")
        for i, (x, y, angle, orient, ih) in enumerate(cam_B_list):
            d_perp   = max(abs(y - walk_y), 0.3)
            tilt_deg = math.degrees(math.atan2(cfg.HUMAN_HEIGHT / 2.0 - ih, d_perp))
            LOG.log(f"  B{i+1} [{orient}]  pos=({x:.2f}m, {y:.2f}m)  h={ih:.1f}m  "
                    f"angle={angle:.0f}deg  tilt={abs(tilt_deg):.1f}deg")

    LOG.log("="*65)


# =============================================================
# MAIN LOOP
# =============================================================

if __name__ == "__main__":
    run_ts      = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    config_name = os.path.splitext(os.path.basename(_args.config))[0]

    # ── Unique run directory ──────────────────────────────────────────────
    # Structure:
    #   outputs/
    #     <config>_<YYYYMMDD>_<HHMMSS>/
    #       log.txt
    #       graphs_all/          ← every attempt (graph_mode=all)
    #       graphs_best_per_combo/  ← best result per combo
    #       FINAL_RESULT.png     ← global best at the end
    run_dir          = os.path.join(_HERE, "outputs", f"{config_name}_{run_ts}")
    graphs_all_dir   = os.path.join(run_dir, "graphs_all")
    graphs_combo_dir = os.path.join(run_dir, "graphs_best_per_combo")
    log_path         = os.path.join(run_dir, "log.txt")

    os.makedirs(graphs_all_dir,   exist_ok=True)
    os.makedirs(graphs_combo_dir, exist_ok=True)
    LOG.init(log_path)

    cam_A = next(c for c in CFG.camera_sets if c.mounting == "wall")
    cam_B = next((c for c in CFG.camera_sets if c.mounting == "tripod"), None)

    LOG.log("="*65)
    LOG.log("  LAB CAMERA OPTIMIZER")
    LOG.log("="*65)
    LOG.log(f"Config : {_args.config}")
    LOG.log(f"Room   : {CFG.ROOM_CORNERS}")
    LOG.log(f"{cam_A.name}: L={cam_A.fov_h_L}°×{cam_A.fov_v_L}°  P={cam_A.fov_h_P}°×{cam_A.fov_v_P}°"
            f"  heights={cam_A.height_options}m  max={cam_A.max_count}")
    if cam_B:
        LOG.log(f"{cam_B.name}: L={cam_B.fov_h_L}°×{cam_B.fov_v_L}°  P={cam_B.fov_h_P}°×{cam_B.fov_v_P}°"
                f"  heights={cam_B.height_options}m  max={cam_B.max_count}")
    LOG.log(f"Bilateral weight : {CFG.BILATERAL_WEIGHT}"
            f"  {'(disabled)' if CFG.BILATERAL_WEIGHT == 0 else ''}")
    LOG.log(f"Target coverage  : {CFG.TARGET_COVERAGE} cameras per point")
    LOG.log(f"Run directory    : {run_dir}")
    LOG.log(f"Log              : {log_path}\n")

    # ── Zone sweep parameters ─────────────────────────────────────────────
    # Corridor parameters are optional — a config may use only polygon zones.
    _has_corridor = hasattr(CFG, "CORRIDOR_LENGTH") and CFG.CORRIDOR_LENGTH > 0

    # Room centre — always defined, used as fallback for polygon-only configs
    _room_cx = sum(c[0] for c in CFG.ROOM_CORNERS) / len(CFG.ROOM_CORNERS)
    _room_cy = sum(c[1] for c in CFG.ROOM_CORNERS) / len(CFG.ROOM_CORNERS)

    if _has_corridor:
        CORRIDOR_LENGTH   = CFG.CORRIDOR_LENGTH
        ZONE_LENGTH       = CFG.ZONE_LENGTH
        CORRIDOR_X_STARTS = CFG.CORRIDOR_X_STARTS
        ZONE_OFFSETS      = CFG.ZONE_OFFSETS
        ZONE_Y_WALKS      = CFG.ZONE_Y_WALKS
    else:
        # No corridor — single neutral iteration; polygon sweep drives all combos.
        CORRIDOR_LENGTH   = 0.0
        ZONE_LENGTH       = 0.0
        CORRIDOR_X_STARTS = [_room_cx]
        ZONE_OFFSETS      = [0.0]
        ZONE_Y_WALKS      = [_room_cy]

    # ── Polygon zone offsets sweep ────────────────────────────────────────
    # Zones that share the same x_offsets+y_offsets are part of the same
    # "group" (e.g. all parts of a T-shape) and must move TOGETHER.
    # Zones with different offsets are swept independently (cartesian product).
    #
    # Grouping key = frozenset of (dx,dy) pairs — zones with identical
    # placement grids are merged into one group.
    import itertools

    poly_zones = [z for z in CFG.capture_zones if z.type == "polygon" and z.vertices]

    if poly_zones:
        # Group zones by their placement grid (x_offsets × y_offsets)
        groups = {}   # key -> list of zone_ids
        for z in poly_zones:
            key = (tuple(z.x_offsets), tuple(z.y_offsets))
            groups.setdefault(key, []).append(z)

        # For each group, build the list of (dx,dy) offset pairs (same for all zones in group)
        # Each combo entry = list of (zone_id, dx, dy) for all zones in the group
        per_group_combos = []
        for (x_offs, y_offs), grp_zones in groups.items():
            pairs = list(itertools.product(x_offs, y_offs))
            # Each pair applies to ALL zones in this group simultaneously
            group_combos = [
                [(z.id, dx, dy) for z in grp_zones]
                for (dx, dy) in pairs
            ]
            per_group_combos.append(group_combos)

        # Cartesian product across groups (groups with different grids vary independently)
        poly_combos = [
            sum(combo_per_group, [])   # flatten: list of (zone_id, dx, dy)
            for combo_per_group in itertools.product(*per_group_combos)
        ]
    else:
        poly_combos = [()]

    total_combos = (len(CORRIDOR_X_STARTS) * len(ZONE_OFFSETS)
                    * len(ZONE_Y_WALKS) * len(poly_combos))

    LOG.log(f"Grid: {len(CORRIDOR_X_STARTS)} corridor X"
            f"  ×  {len(ZONE_OFFSETS)} zone offsets"
            f"  ×  {len(ZONE_Y_WALKS)} Y positions"
            f"  ×  {len(poly_combos)} polygon placement(s)"
            f"  =  {total_combos} combinations  (restarts={CFG.opt.restarts_per_combo})\n")
    if poly_zones:
        for (x_offs, y_offs), grp_zones in groups.items():
            ids = [z.id for z in grp_zones]
            n_placements = len(x_offs) * len(y_offs)
            LOG.log(f"  Group {ids}: {len(x_offs)} X × {len(y_offs)} Y"
                    f" = {n_placements} placements (move together)")
        LOG.log("")

    global_best_score    = -1.0
    global_best_cam_A    = []
    global_best_cam_B    = []
    global_best_sts      = (CFG.STS_X, CFG.STS_Y)
    global_best_state    = None
    record_counter       = [0]
    combo_idx            = 0
    combo_best_configs   = {}

    for corridor_x_start in CORRIDOR_X_STARTS:
        corridor_x_end = corridor_x_start + CORRIDOR_LENGTH
        for zone_offset in ZONE_OFFSETS:
            zone_start = corridor_x_start + zone_offset
            zone_end   = zone_start + ZONE_LENGTH
            for walk_y in ZONE_Y_WALKS:
              for poly_combo in poly_combos:
                combo_idx += 1

                # ── Apply polygon translations for this combo ─────────────
                # Each entry in poly_combo = (zone_id, dx, dy)
                # We translate the polygon vertices by (dx, dy) for this run.
                # The original vertices (relative coords) are preserved in z.vertices.
                poly_label_parts = []
                for item in poly_combo:
                    zone_id, dx, dy = item
                    for z in CFG.capture_zones:
                        if z.id == zone_id:
                            z._translated_vertices = [
                                (vx + dx, vy + dy) for (vx, vy) in z.vertices
                            ]
                    poly_label_parts.append(f"{zone_id}_dx{dx:.1f}_dy{dy:.1f}")
                poly_suffix = ("_P" + "-".join(poly_label_parts)) if poly_label_parts else ""

                # ── Build combo labels ────────────────────────────────────
                # combo_label      : full human-readable label for logs
                # combo_label_short: compact label safe for filenames (no path length issues)
                if _has_corridor:
                    combo_label = (f"C{corridor_x_start:.0f}-{corridor_x_end:.0f}"
                                   f"_Z{zone_start:.0f}-{zone_end:.0f}"
                                   f"_Y{walk_y:.1f}{poly_suffix}")
                else:
                    combo_label = f"Y{walk_y:.1f}{poly_suffix}" if poly_suffix else "poly_only"

                # Short label: just the combo index + first offset pair (fits in any path)
                if poly_combo:
                    _first = poly_combo[0]   # (zone_id, dx, dy)
                    combo_label_short = f"combo{combo_idx:03d}_dx{_first[1]:.1f}_dy{_first[2]:.1f}"
                else:
                    combo_label_short = f"combo{combo_idx:03d}"

                # Skip geometrically invalid combos (only meaningful with corridor)
                if _has_corridor and zone_end > 8.0 and walk_y > 2.8:
                    LOG.log(f"  [{combo_idx:2d}/{total_combos}]  {combo_label}"
                            f"  — skipped (narrow section)")
                    continue

                LOG.log(f"\n{'─'*60}")
                LOG.log(f"  COMBO [{combo_idx:2d}/{total_combos}]  {combo_label}")
                if _has_corridor:
                    LOG.log(f"  Run-up={zone_offset:.0f}m  |  "
                            f"Decel={CORRIDOR_LENGTH-ZONE_LENGTH-zone_offset:.0f}m")
                else:
                    LOG.log(f"  walk_y (bilateral axis) = {walk_y:.2f}m"
                            f"  |  X range = [{zone_start:.1f} → {zone_end:.1f}]")
                LOG.log(f"{'─'*60}")

                # Build zone state (no globals mutation)
                if _has_corridor:
                    sts_x = zone_start + ZONE_LENGTH / 2.0
                    sts_y = walk_y
                else:
                    # ── Polygon-only: derive all zone params from translated vertices ──
                    #
                    # walk_y  → centroid Y of the highest-priority zone (intersection)
                    #           This is the true "axis" for bilateral S/N scoring.
                    #
                    # analysis_x_start/end → bounding box X of the full T
                    #           (used for sample point weighting and STS search range)
                    #
                    # corridor_x_start/end → same, wider bounding box

                    # Highest-priority zone centroid → walk_y and sts position
                    _best_z = max(
                        (z for z in CFG.capture_zones if z.type == "polygon"),
                        key=lambda z: z.priority,
                        default=None
                    )
                    if _best_z is not None:
                        _verts = getattr(_best_z, "_translated_vertices", None) or _best_z.vertices
                        sts_x  = sum(v[0] for v in _verts) / len(_verts)
                        sts_y  = sum(v[1] for v in _verts) / len(_verts)
                        walk_y = sts_y   # bilateral axis = centre of the priority zone
                    else:
                        sts_x  = _room_cx
                        sts_y  = _room_cy
                        walk_y = _room_cy

                    # Bounding box of ALL translated polygon vertices → analysis range
                    all_verts = []
                    for z in CFG.capture_zones:
                        if z.type == "polygon":
                            v = getattr(z, "_translated_vertices", None) or z.vertices
                            all_verts.extend(v)
                    if all_verts:
                        corridor_x_start = min(v[0] for v in all_verts)
                        corridor_x_end   = max(v[0] for v in all_verts)
                        zone_start       = corridor_x_start
                        zone_end         = corridor_x_end
                    else:
                        corridor_x_start = _room_cx - 1.0
                        corridor_x_end   = _room_cx + 1.0
                        zone_start       = corridor_x_start
                        zone_end         = corridor_x_end
                state = build_state(CFG, walk_y,
                                    corridor_x_start, corridor_x_end,
                                    zone_start, zone_end,
                                    sts_x, sts_y)

                sample_points = build_sample_points(CFG, state)

                cam_A_cands, cam_B_cands = generate_candidates(
                    CFG, state,
                    wall_step        = CFG.WALL_STEP,
                    angle_steps      = CFG.ANGLE_STEPS,
                    tripod_grid_step = CFG.TRIPOD_GRID_STEP,
                    sample_points    = sample_points,
                )
                LOG.log(f"  Candidates: {len(cam_A_cands)} cam_A  +  {len(cam_B_cands)} cam_B"
                        f"  |  Eval points: {len(sample_points)}")

                def _on_record(attempt, ca, cb, sc, sts_pos, is_rec=False):
                    record_counter[0] += (1 if is_rec else 0)
                    record_marker = f"_RECORD{record_counter[0]}" if is_rec else ""
                    graph_fname = (f"attempt{attempt:02d}_{combo_label_short}"
                                   f"_score{sc:.0f}{record_marker}.png")
                    graph_path = os.path.join(graphs_all_dir, graph_fname)
                    os.makedirs(graphs_all_dir, exist_ok=True)
                    s = dict(state)
                    s["sts_x"] = sts_pos[0]; s["sts_y"] = sts_pos[1]
                    visualize_solution(ca, cb, sc, CFG, s,
                                       show_window=False, save_path=graph_path)
                    LOG.log(f"       Graph: {graph_path}")

                best_cam_A, best_cam_B, best_score, best_sts = greedy_place_cameras(
                    cam_A_cands, cam_B_cands,
                    sample_points, CFG, state,
                    n_restarts  = CFG.opt.restarts_per_combo,
                    combo_label = combo_label,
                    log         = LOG.log,
                    on_record   = _on_record,
                )

                combo_best_configs[combo_label] = {
                    "cam_A":          list(best_cam_A),
                    "cam_B":          list(best_cam_B),
                    "score":          best_score,
                    "sts":            best_sts,
                    "state":          dict(state, sts_x=best_sts[0], sts_y=best_sts[1]),
                    "combo_idx":      combo_idx,
                }

                LOG.log(f"\n  -> Score {combo_label}: {best_score:.3f}"
                        f"  STS=({best_sts[0]:.2f},{best_sts[1]:.2f})", end="")

                if best_score > global_best_score:
                    global_best_score  = best_score
                    global_best_cam_A  = list(best_cam_A)
                    global_best_cam_B  = list(best_cam_B)
                    global_best_sts    = best_sts
                    global_best_state  = dict(state, sts_x=best_sts[0], sts_y=best_sts[1])
                    LOG.log("  BEST GLOBAL CONFIG SO FAR")
                else:
                    LOG.log("")

                # ── Best-per-combo graph — generated immediately ──────────
                combo_graph = os.path.join(
                    graphs_combo_dir,
                    f"{combo_label_short}_score{best_score:.0f}.png")
                visualize_solution(best_cam_A, best_cam_B, best_score, CFG,
                                   dict(state, sts_x=best_sts[0], sts_y=best_sts[1]),
                                   show_window=False, save_path=combo_graph)
                LOG.log(f"  Best-per-combo graph: {combo_graph}")

    # ── Final summary ─────────────────────────────────────────────────────
    LOG.log(f"\n{'='*65}")
    LOG.log(f"  FINAL OPTIMAL CONFIGURATION")
    LOG.log(f"  Score: {global_best_score:.3f}")
    LOG.log(f"  STS  : ({global_best_sts[0]:.2f}m, {global_best_sts[1]:.2f}m)")
    LOG.log(f"  Total records: {record_counter[0]}")
    LOG.log(f"{'='*65}")

    print_results(global_best_cam_A, global_best_cam_B,
                  global_best_score, CFG, global_best_state)

    # ── Final graph ───────────────────────────────────────────────────────
    final_graph = os.path.join(run_dir, f"FINAL_RESULT_score{global_best_score:.0f}.png")
    LOG.log(f"\nFinal graph: {final_graph}")
    visualize_solution(global_best_cam_A, global_best_cam_B,
                       global_best_score, CFG, global_best_state,
                       show_window=True, save_path=final_graph)

    # ── Per-combo ranked graphs ───────────────────────────────────────────
    sorted_combos = sorted(combo_best_configs.items(),
                           key=lambda x: x[1]["score"], reverse=True)
    for rank, (label, cfg_combo) in enumerate(sorted_combos, start=1):
        short = f"combo{cfg_combo['combo_idx']:03d}"
        gp = os.path.join(graphs_combo_dir,
                          f"rank{rank:02d}_{short}_score{cfg_combo['score']:.0f}.png")
        LOG.log(f"  [{rank:2d}]  {label}  score={cfg_combo['score']:.3f}  ->  {gp}")
        visualize_solution(cfg_combo["cam_A"], cfg_combo["cam_B"],
                           cfg_combo["score"], CFG, cfg_combo["state"],
                           show_window=False, save_path=gp)

    LOG.log(f"\nAll outputs saved in: {run_dir}")
    LOG.log(f"Log: {log_path}")
    LOG.close()

