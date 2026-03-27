"""
evaluate.py
Entry point for evaluating a fixed camera setup.

Usage:
    python evaluate.py --config configs/example_evaluation_CHU.yaml
"""

import sys
import argparse
import os
import datetime
import importlib.util
import yaml

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
    description="Lab Camera Evaluator — evaluate a fixed camera setup.")
_parser.add_argument(
    "--config",
    default=os.path.join(_HERE, "configs", "example_evaluation_CHU.yaml"),
    help="Path to YAML configuration file with an 'evaluation_setup' section.")
_args = _parser.parse_args()

CFG = config_loader.load_config(_args.config)

# ── Import core modules ───────────────────────────────────────────────────────
from core.room       import build_wall_segments
from core.candidates import build_sample_points
from core.scoring    import score_configuration
from core.visualize  import visualize_solution
from core.reporting  import LOG, print_results


# =============================================================
# BUILD STATE DICT (from optimize.py)
# =============================================================

def build_state(cfg, walk_y, walk_x_start, walk_x_end,
                analysis_x_start, analysis_x_end,
                sts_x, sts_y):
    """
    Returns a state dict with all zone-dependent values.
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
# MAIN
# =============================================================

def main():
    run_ts      = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    config_name = os.path.splitext(os.path.basename(_args.config))[0]

    run_dir  = os.path.join(_HERE, "outputs", "evaluations", f"{config_name}_{run_ts}")
    log_path = os.path.join(run_dir, "log.txt")
    os.makedirs(run_dir, exist_ok=True)
    LOG.init(log_path)
    LOG.log(f"=== CAMERA EVALUATION LOG — {config_name} ===")

    # ── Load fixed cameras from config ────────────────────────────────────
    with open(_args.config, 'r', encoding='utf-8') as f:
        raw_config = yaml.safe_load(f)

    eval_setup = raw_config.get("evaluation_setup", {})
    fixed_cams_data = eval_setup.get("fixed_cameras")

    if not fixed_cams_data:
        LOG.log(f"ERROR: No 'evaluation_setup' section with 'fixed_cameras' list found in {_args.config}")
        LOG.close()
        return

    LOG.log(f"Found {len(fixed_cams_data)} fixed cameras to evaluate.")

    cam_A_list = []
    cam_B_list = []
    for cam_data in fixed_cams_data:
        set_id = cam_data.get("set_id")
        cam_set = next((cs for cs in CFG.camera_sets if cs.id == set_id), None)
        if not cam_set:
            LOG.log(f"WARNING: Camera set '{set_id}' not found. Skipping camera.")
            continue

        cam_tuple = (
            cam_data["x"], cam_data["y"], cam_data["angle"],
            cam_data["orientation"], cam_data["z"]
        )
        if cam_set.mounting == "wall":
            cam_A_list.append(cam_tuple)
        elif cam_set.mounting == "tripod":
            cam_B_list.append(cam_tuple)

    # ── Build state for the FIRST zone combination ────────────────────────
    # In evaluation mode, we don't sweep through all zone positions. We just
    # evaluate the setup against the first defined capture zone placement.
    # This logic is simplified from optimize.py's main loop.
    
    # Corridor-based (if any)
    _has_corridor = hasattr(CFG, "CORRIDOR_LENGTH") and CFG.CORRIDOR_LENGTH > 0
    if _has_corridor:
        corridor_x_start = CFG.CORRIDOR_X_STARTS[0]
        zone_offset      = CFG.ZONE_OFFSETS[0]
        walk_y           = CFG.ZONE_Y_WALKS[0]
        corridor_x_end   = corridor_x_start + CFG.CORRIDOR_LENGTH
        zone_start       = corridor_x_start + zone_offset
        zone_end         = zone_start + CFG.ZONE_LENGTH
        sts_x            = zone_start + CFG.ZONE_LENGTH / 2.0
        sts_y            = walk_y
    else: # Polygon-based
        # Fallback to room center if no corridor
        _room_cx = sum(c[0] for c in CFG.ROOM_CORNERS) / len(CFG.ROOM_CORNERS)
        _room_cy = sum(c[1] for c in CFG.ROOM_CORNERS) / len(CFG.ROOM_CORNERS)
        walk_y = _room_cy
        corridor_x_start, corridor_x_end = _room_cx - 1, _room_cx + 1
        zone_start, zone_end = corridor_x_start, corridor_x_end
        sts_x, sts_y = _room_cx, _room_cy

    state = build_state(CFG, walk_y, corridor_x_start, corridor_x_end,
                        zone_start, zone_end, sts_x, sts_y)

    # ── Score and visualize ───────────────────────────────────────────────
    sample_points = build_sample_points(CFG, state)
    LOG.log(f"Evaluating against {len(sample_points)} sample points.")

    score, _, south_s, north_s = score_configuration(
        cam_A_list, cam_B_list, sample_points, CFG, state
    )

    print_results(cam_A_list, cam_B_list, score, (south_s, north_s), CFG, state)

    final_graph_path = os.path.join(run_dir, f"EVALUATION_RESULT_score{score:.0f}.png")
    LOG.log(f"\nSaving result graph to: {final_graph_path}")

    visualize_solution(cam_A_list, cam_B_list, score, CFG, state,
                       show_window=True, save_path=final_graph_path)

    LOG.log(f"\nEvaluation complete. All outputs in: {run_dir}")
    LOG.close()


if __name__ == "__main__":
    main()