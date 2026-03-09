"""
config_loader.py
Reads a YAML configuration file and exposes a Config object used by
optimize_camera_placement.py.

All hard-coded constants in the main script are replaced by attributes
of the Config object returned by load_config().
"""

import yaml
import os
import math


# ─────────────────────────────────────────────────────────────────────────────
# Helper dataclasses (simple namespaces — no external dependency)
# ─────────────────────────────────────────────────────────────────────────────

class _Ns:
    """Generic namespace: _Ns(a=1, b=2) → obj.a, obj.b"""
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def __repr__(self):
        return f"{self.__class__.__name__}({self.__dict__})"


# ─────────────────────────────────────────────────────────────────────────────
# Public entry point
# ─────────────────────────────────────────────────────────────────────────────

def load_config(yaml_path: str) -> "_Ns":
    """
    Loads a YAML config file and returns a Config namespace with all
    parameters needed by the optimiser.

    Parameters
    ----------
    yaml_path : str
        Path to the .yaml configuration file.

    Returns
    -------
    cfg : _Ns
        Namespace with the following top-level attributes:
          cfg.room           — room geometry
          cfg.obstacles      — list of obstacle dicts (normalised)
          cfg.subject        — subject height / foot_z
          cfg.camera_sets    — list of CameraSet namespaces
          cfg.capture_zones  — list of CaptureZone namespaces
          cfg.opt            — optimisation parameters
    """
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"Config file not found: {yaml_path}")

    with open(yaml_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    cfg = _Ns()

    # ── Room ──────────────────────────────────────────────────────────────
    r = raw["room"]
    cfg.room = _Ns(
        corners = [tuple(c) for c in r["corners"]],
        height  = float(r["height"]),
    )

    # ── Obstacles ─────────────────────────────────────────────────────────
    cfg.obstacles = []
    for obs in raw.get("obstacles", []):
        o = dict(obs)  # shallow copy
        o["height"] = float(o["height"])
        o["can_mount_camera"] = bool(o.get("can_mount_camera", False))
        # Normalise to always have a "vertices" key
        if o["type"] == "rect":
            x0, y0, x1, y1 = o["bounds"]
            o["vertices"] = [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
        elif o["type"] == "polygon":
            o["vertices"] = [tuple(v) for v in o["vertices"]]
        cfg.obstacles.append(o)

    # ── Subject ───────────────────────────────────────────────────────────
    s = raw.get("subject", {})
    cfg.subject = _Ns(
        height = float(s.get("height", 1.9)),
        foot_z = float(s.get("foot_z", 0.0)),
    )
    cfg.subject.head_z = cfg.subject.height  # alias

    # ── Camera sets ───────────────────────────────────────────────────────
    cfg.camera_sets = []
    for cs in raw.get("camera_sets", []):
        cam = _Ns(
            id           = cs["id"],
            name         = cs.get("name", cs["id"]),
            mounting     = cs.get("mounting", "wall"),
            optional     = bool(cs.get("optional", False)),

            fov_h_L = float(cs.get("fov_h_landscape", 110.0)),
            fov_v_L = float(cs.get("fov_v_landscape",  70.0)),
            fov_h_P = float(cs.get("fov_h_portrait",   70.0)),
            fov_v_P = float(cs.get("fov_v_portrait",  110.0)),

            max_range  = float(cs.get("max_range", 15.0)),
            min_range  = float(cs.get("min_range",  0.5)),

            height_options = [float(h) for h in cs.get("height_options", [2.0])],
            max_count      = int(cs.get("max_count", 10)),
            min_spacing    = float(cs.get("min_spacing", 1.5)),

            score_weight     = float(cs.get("score_weight", 1.0)),
            walk_axis_margin = float(cs.get("walk_axis_margin", 0.0)),

            color = str(cs.get("color", "#1f77b4")),
        )
        # Derived: is this set effectively disabled?
        cam.enabled = (not cam.optional) or (cam.max_count > 0)
        cfg.camera_sets.append(cam)

    # ── Capture zones ─────────────────────────────────────────────────────
    cfg.capture_zones = []
    zone_by_id = {}
    for z in raw.get("capture_zones", []):
        pl = z.get("placement", {})
        zone = _Ns(
            id           = z["id"],
            type         = z.get("type", "corridor"),
            priority     = float(z.get("priority", 1.0)),
            contained_in = z.get("contained_in", None),
            auto_optimize = bool(z.get("auto_optimize", False)),

            # corridor / sub_zone
            length         = float(z.get("length", 10.0)),
            width          = float(z.get("width",   1.0)),
            offset_options = [float(o) for o in z.get("offset_options", [0.0])],

            # corridor placement sweep
            x_start_options = [float(x) for x in pl.get("x_start_options", [0.0])],
            y_options        = [float(y) for y in pl.get("y_options", [1.4])],

            # point zone
            radius = float(z.get("radius", 0.5)),

            # polygon zone — arbitrary shape (L, T, U, cross…)
            # vertices are in RELATIVE coordinates (origin = 0,0)
            # placement.x_offsets / y_offsets define the sweep translations
            vertices  = [tuple(v) for v in z.get("vertices", [])],
            grid_step = float(z.get("grid_step", 0.20)),
            x_offsets = [float(v) for v in pl.get("x_offsets", [0.0])],
            y_offsets = [float(v) for v in pl.get("y_offsets", [0.0])],
        )
        cfg.capture_zones.append(zone)
        zone_by_id[zone.id] = zone

    # Resolve parent references
    for zone in cfg.capture_zones:
        zone.parent = zone_by_id.get(zone.contained_in, None)

    # ── Optimisation parameters ───────────────────────────────────────────
    opt = raw.get("optimization", {})
    cfg.opt = _Ns(
        target_coverage              = int(opt.get("target_coverage", 3)),
        bilateral_weight             = float(opt.get("bilateral_weight", 0.8)),
        vertical_coverage_threshold  = float(opt.get("vertical_coverage_threshold", 0.9)),
        restarts_per_combo           = int(opt.get("restarts_per_combo", 20)),
        wall_step                    = float(opt.get("wall_step", 0.35)),
        angle_steps                  = int(opt.get("angle_steps", 24)),
        tripod_grid_step             = float(opt.get("tripod_grid_step", 0.70)),
        distance_quality_factor      = float(opt.get("distance_quality_factor", 0.01)),
        graph_mode                   = str(opt.get("graph_mode", "records_only")),
        algo                         = str(opt.get("algo", "greedy_1opt")),
        early_stop                   = int(opt.get("early_stop", 0)),  # 0 = auto (n_restarts//3)
    )

    # ── Convenience shortcuts (mirrors old global constants) ──────────────
    # These allow the main script to reference cfg.ROOM_CORNERS etc.
    # during the transition period before full refactor.
    cfg.ROOM_CORNERS = cfg.room.corners
    cfg.ROOM_HEIGHT  = cfg.room.height

    cfg.HUMAN_HEIGHT = cfg.subject.height
    cfg.HUMAN_FOOT_Z = cfg.subject.foot_z
    cfg.HUMAN_HEAD_Z = cfg.subject.head_z

    cfg.TARGET_COVERAGE  = cfg.opt.target_coverage
    cfg.BILATERAL_WEIGHT = cfg.opt.bilateral_weight

    # Camera set A (first wall-mounted set) — backward-compat shortcuts
    cam_A = next((c for c in cfg.camera_sets if c.mounting == "wall"), None)
    if cam_A:
        cfg.ZED_FOV_H_L         = cam_A.fov_h_L
        cfg.ZED_FOV_V_L         = cam_A.fov_v_L
        cfg.ZED_FOV_H_P         = cam_A.fov_h_P
        cfg.ZED_FOV_V_P         = cam_A.fov_v_P
        cfg.ZED_RADIUS          = cam_A.max_range
        cfg.ZED_MIN_DIST        = cam_A.min_range
        cfg.ZED_HEIGHT          = cam_A.height_options[0]
        cfg.ZED_HEIGHT_OPTIONS  = cam_A.height_options
        cfg.MIN_DIST_BETWEEN_ZED = cam_A.min_spacing
        cfg.COLOR_ZED           = cam_A.color
        cfg.COLOR_CONE_ZED      = cam_A.color
        cfg.N_ZED               = cam_A.max_count

    # Camera set B (first tripod set) — backward-compat shortcuts
    cam_B = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)
    if cam_B:
        cfg.IPAD_FOV_H_L          = cam_B.fov_h_L
        cfg.IPAD_FOV_V_L          = cam_B.fov_v_L
        cfg.IPAD_FOV_H_P          = cam_B.fov_h_P
        cfg.IPAD_FOV_V_P          = cam_B.fov_v_P
        cfg.IPAD_RADIUS           = cam_B.max_range
        cfg.IPAD_HEIGHT           = cam_B.height_options[0]
        cfg.IPAD_HEIGHT_OPTIONS   = cam_B.height_options
        cfg.MIN_DIST_BETWEEN_IPAD = cam_B.min_spacing
        cfg.IPAD_WALK_MARGIN      = cam_B.walk_axis_margin
        cfg.COLOR_IPAD            = cam_B.color
        cfg.COLOR_CONE_IPAD       = cam_B.color
        cfg.N_IPAD                = cam_B.max_count
    else:
        # No tripod set defined → safe defaults (tripod cameras disabled)
        cfg.IPAD_FOV_H_L = cfg.IPAD_FOV_V_L = 80.0
        cfg.IPAD_FOV_H_P = cfg.IPAD_FOV_V_P = 58.0
        cfg.IPAD_RADIUS           = 10.0
        cfg.IPAD_HEIGHT           = 1.5
        cfg.IPAD_HEIGHT_OPTIONS   = [1.5]
        cfg.MIN_DIST_BETWEEN_IPAD = 1.5
        cfg.IPAD_WALK_MARGIN      = 0.7
        cfg.COLOR_IPAD            = "#d62728"
        cfg.COLOR_CONE_IPAD       = "#d62728"
        cfg.N_IPAD                = 0

    # Capture zone shortcuts
    corridor = next((z for z in cfg.capture_zones if z.type == "corridor"), None)
    analysis = next((z for z in cfg.capture_zones if z.type == "sub_zone"), None)
    sts      = next((z for z in cfg.capture_zones if z.type == "point"), None)

    # Room centre — used as fallback for all positional defaults
    _cx = sum(c[0] for c in cfg.ROOM_CORNERS) / len(cfg.ROOM_CORNERS)
    _cy = sum(c[1] for c in cfg.ROOM_CORNERS) / len(cfg.ROOM_CORNERS)

    if corridor:
        cfg.CORRIDOR_LENGTH    = corridor.length
        cfg.CORRIDOR_X_STARTS  = corridor.x_start_options
        cfg.ZONE_Y_WALKS       = corridor.y_options
        cfg.WALK_X_START       = corridor.x_start_options[0]
        cfg.WALK_X_END         = corridor.x_start_options[0] + corridor.length
        cfg.WALK_Y             = corridor.y_options[0]
    else:
        # No corridor — fall back to room centre; polygon sweep drives everything
        cfg.CORRIDOR_LENGTH    = 0.0
        cfg.CORRIDOR_X_STARTS  = [_cx]
        cfg.ZONE_Y_WALKS       = [_cy]
        cfg.WALK_X_START       = _cx
        cfg.WALK_X_END         = _cx
        cfg.WALK_Y             = _cy

    if analysis:
        cfg.ZONE_LENGTH        = analysis.length
        cfg.ZONE_OFFSETS       = analysis.offset_options
        cfg.ANALYSIS_X_START   = cfg.WALK_X_START + analysis.offset_options[0]
        cfg.ANALYSIS_X_END     = cfg.ANALYSIS_X_START + analysis.length
    else:
        cfg.ZONE_LENGTH        = 0.0
        cfg.ZONE_OFFSETS       = [0.0]
        cfg.ANALYSIS_X_START   = cfg.WALK_X_START
        cfg.ANALYSIS_X_END     = cfg.WALK_X_END

    if sts:
        cfg.STS_RADIUS = sts.radius
        cfg.STS_X      = cfg.ANALYSIS_X_START + cfg.ZONE_LENGTH / 2.0 if analysis else _cx
        cfg.STS_Y      = cfg.WALK_Y
    else:
        # No STS point — put it at the room centre (used only as a fallback reference)
        cfg.STS_RADIUS = 0.5
        cfg.STS_X      = _cx
        cfg.STS_Y      = _cy

    # Optimisation resolution shortcuts
    cfg.WALL_STEP        = cfg.opt.wall_step
    cfg.ANGLE_STEPS      = cfg.opt.angle_steps
    cfg.TRIPOD_GRID_STEP = cfg.opt.tripod_grid_step
    cfg.DIST_QUALITY_K   = cfg.opt.distance_quality_factor

    return cfg


# ─────────────────────────────────────────────────────────────────────────────
# Quick validation (run as script: python config_loader.py <config.yaml>)
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else "configs/labo_CHU.yaml"
    cfg = load_config(path)
    print("=== Config loaded successfully ===")
    print(f"Room corners   : {cfg.ROOM_CORNERS}")
    print(f"Room height    : {cfg.ROOM_HEIGHT} m")
    print(f"Obstacles      : {len(cfg.obstacles)}")
    print(f"Camera sets    : {[c.name for c in cfg.camera_sets]}")
    print(f"Capture zones  : {[z.id for z in cfg.capture_zones]}")
    print(f"ZED heights    : {cfg.ZED_HEIGHT_OPTIONS}")
    print(f"iPad heights   : {cfg.IPAD_HEIGHT_OPTIONS}")
    print(f"Target coverage: {cfg.TARGET_COVERAGE}")
    print(f"Bilateral w    : {cfg.BILATERAL_WEIGHT}")
    print(f"Restarts/combo : {cfg.opt.restarts_per_combo}")

