"""
candidates.py
Generates all candidate camera placements (wall-mounted and tripod).
"""

import math
import numpy as np
from .room import (point_in_room, point_in_polygon, has_line_of_sight,
                   wall_angular_limits)
from .scoring import get_fov


# =============================================================
# ANGLE HELPERS  (mirrors optimize_camera_placement.py logic)
# =============================================================

def _aperture_midpoint(wall_lim):
    """Circular mean of lim_left / lim_right → ideal inward direction."""
    (lim_left, _), (lim_right, _) = wall_lim
    if lim_left is None:
        return None
    sin_m = (math.sin(math.radians(lim_left)) + math.sin(math.radians(lim_right))) / 2.0
    cos_m = (math.cos(math.radians(lim_left)) + math.cos(math.radians(lim_right))) / 2.0
    return math.degrees(math.atan2(sin_m, cos_m))


def _aperture_span(wall_lim, ref_angle):
    """
    Returns (lo, hi) degrees RELATIVE to ref_angle so that lo ≤ 0 ≤ hi.
    Falls back to (-180, 180) when the aperture wraps across ±180°.
    """
    (lim_left, _), (lim_right, _) = wall_lim
    if lim_left is None:
        return -180.0, 180.0

    def _rel(a):
        return (a - ref_angle + 180) % 360 - 180

    lo = min(_rel(lim_left), _rel(lim_right))
    hi = max(_rel(lim_left), _rel(lim_right))
    if not (lo <= 0 <= hi):
        return -180.0, 180.0
    return lo, hi


def _clamp_angle_to_aperture(cam_angle, fov_h, wall_lim):
    """
    Returns the angle closest to cam_angle such that the cone
    [angle ± fov_h/2] stays within the aperture.

    When fov_h < aperture  → standard clamp: keep as close to cam_angle as possible.
    When fov_h >= aperture → push cone to butt against its own wall on the side
                             of the NEAREST adjacent wall, so the overflow is
                             toward the FAR adjacent wall.
                             This maximises the in-room fraction of the cone.

    Returns (clamped_angle, residual_overflow_degrees).
    """
    mid = _aperture_midpoint(wall_lim)
    if mid is None:
        return cam_angle, 0.0

    lo, hi = _aperture_span(wall_lim, mid)
    aperture = hi - lo
    half = fov_h / 2.0

    if aperture <= 0:
        return mid, half

    if fov_h >= aperture:
        # FOV is wider than the available aperture — some overflow is unavoidable.
        # Strategy: push the cone so its edge butts against the own wall on the
        # side of the NEAREST adjacent wall.
        #   dist_left  = distance to the adjacent wall at lim_left
        #   dist_right = distance to the adjacent wall at lim_right
        # Nearest adjacent wall → push cone edge against that side of own wall
        # → overflow goes toward the FAR adjacent wall (less problematic).
        (_, dist_left), (_, dist_right) = wall_lim
        d_left  = dist_left  if dist_left  > 0 else 1e6
        d_right = dist_right if dist_right > 0 else 1e6

        if d_left <= d_right:
            # Left adjacent wall is closer → push cone left edge to lim_left
            # cone_left = mid + offset - half = lo  →  offset = lo + half
            offset = lo + half
        else:
            # Right adjacent wall is closer → push cone right edge to lim_right
            # cone_right = mid + offset + half = hi  →  offset = hi - half
            offset = hi - half

        overflow = fov_h - aperture
        return mid + offset, overflow / 2.0

    # Standard clamp: fov fits inside aperture
    offset = (cam_angle - mid + 180) % 360 - 180
    offset = max(offset, lo + half)
    offset = min(offset, hi - half)
    return mid + offset, 0.0


def _aim_quality(cam_angle, fov_h, cam_x, cam_y, targets):
    """
    Score in [0,1]: how well cam_angle covers the target zone.
    targets: list of (tx, ty, weight)
    """
    if not targets:
        return 0.0
    half = fov_h / 2.0
    total_w = sum(w for (_, _, w) in targets)
    if total_w <= 0:
        return 0.0
    score = 0.0
    for (tx, ty, w) in targets:
        angle_to = math.degrees(math.atan2(ty - cam_y, tx - cam_x))
        delta = abs((angle_to - cam_angle + 180) % 360 - 180)
        if delta <= half:
            score += w * (1.0 - delta / half)
    return score / total_w


# =============================================================
# SAMPLE POINTS
# =============================================================

def build_sample_points(cfg, state):
    """
    Returns the list of (px, py, weight) evaluation points for the current
    zone configuration stored in state.
    """
    walk_y           = state["walk_y"]
    walk_x_start     = state["walk_x_start"]
    walk_x_end       = state["walk_x_end"]
    analysis_x_start = state["analysis_x_start"]
    analysis_x_end   = state["analysis_x_end"]
    sts_x            = state["sts_x"]
    sts_y            = state["sts_y"]
    sts_radius       = cfg.STS_RADIUS
    obstacles        = cfg.obstacles
    room_corners     = cfg.ROOM_CORNERS
    room_height      = cfg.ROOM_HEIGHT

    zone_w = next((z.priority for z in cfg.capture_zones if z.type == "sub_zone"),  1.0)
    corr_w = next((z.priority for z in cfg.capture_zones if z.type == "corridor"),  0.5)
    sts_w  = next((z.priority for z in cfg.capture_zones if z.type == "point"),     2.0)

    pts = []

    for x in np.arange(analysis_x_start, analysis_x_end + 0.01, 0.15):
        for dy in np.arange(-0.5, 0.51, 0.2):
            py = walk_y + dy
            if point_in_room(x, py, room_corners, obstacles, room_height):
                pts.append((x, py, zone_w))

    for x in np.arange(walk_x_start, walk_x_end + 0.01, 0.2):
        if x < analysis_x_start or x > analysis_x_end:
            if point_in_room(x, walk_y, room_corners, obstacles, room_height):
                pts.append((x, walk_y, corr_w))

    for angle_deg in range(0, 360, 20):
        for r in [0.2, 0.4, min(0.6, sts_radius)]:
            px = sts_x + r * math.cos(math.radians(angle_deg))
            py = sts_y + r * math.sin(math.radians(angle_deg))
            if point_in_room(px, py, room_corners, obstacles, room_height):
                pts.append((px, py, sts_w))

    for zone in cfg.capture_zones:
        if zone.type != "polygon":
            continue
        verts = getattr(zone, "_translated_vertices", None) or zone.vertices
        if len(verts) < 3:
            continue
        xs_v = [v[0] for v in verts]; ys_v = [v[1] for v in verts]
        step = zone.grid_step
        for px in np.arange(min(xs_v), max(xs_v) + step, step):
            for py in np.arange(min(ys_v), max(ys_v) + step, step):
                if (point_in_polygon(px, py, verts) and
                        point_in_room(px, py, room_corners, obstacles, room_height)):
                    pts.append((px, py, zone.priority))

    return pts


# =============================================================
# CANDIDATE GENERATION
# =============================================================

def generate_candidates(cfg, state, wall_step=0.35, angle_steps=24,
                        tripod_grid_step=0.7, sample_points=None):
    """
    Generates all camera placement candidates.

    Parameters
    ----------
    sample_points : list of (px, py, weight), optional
        If provided, used to compute realistic zone_targets for each wall
        position (camera aims toward the weighted centroid of nearby points
        it can plausibly reach). Falls back to a fixed centre point if None.

    Returns
    -------
    cam_A_cands : list of (x, y, angle, orient, height) — wall-mounted cameras
    cam_B_cands : list of (x, y, angle, orient, height) — tripod cameras
    """
    walk_y           = state["walk_y"]
    analysis_x_start = state["analysis_x_start"]
    analysis_x_end   = state["analysis_x_end"]
    sts_x            = state["sts_x"]
    sts_y            = state["sts_y"]
    wall_segs        = state["wall_segments"]
    room_corners     = cfg.ROOM_CORNERS
    obstacles        = cfg.obstacles
    room_height      = cfg.ROOM_HEIGHT
    human_height     = cfg.HUMAN_HEIGHT

    cam_A = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)

    tgt_x = (analysis_x_start + analysis_x_end) / 2
    tgt_y = walk_y

    # Build zone_targets from real sample points (subsampled for speed).
    # Each wall candidate will use only the sample points it can plausibly
    # reach (within max_range), giving a much more realistic aim direction
    # than a single fixed centre point.
    if sample_points:
        # Subsample: take every 3rd point to limit compute
        _sp_sub = sample_points[::3]
        zone_targets_global = [(px, py, w) for (px, py, w) in _sp_sub]
    else:
        zone_targets_global = [
            (tgt_x, tgt_y, 1.0),
            ((analysis_x_start + analysis_x_end) / 2.0, walk_y, 0.5),
        ]

    corners_loop = room_corners + [room_corners[0]]

    cam_A_cands = []
    cam_B_cands = []
    seen_A = set()
    seen_B = set()

    def _adjacent_wall_penalty(cam_angle, fov_h, wall_lim):
        """
        Returns a multiplier in (0, 1] that penalises orientations whose cone
        overlaps nearby adjacent walls.

        Logic:
        - The cone spans [cam_angle - fov_h/2, cam_angle + fov_h/2].
        - wall_lim gives the angular limits of the open aperture, with the
          distance to each adjacent wall (dist_left, dist_right).
        - If the cone extends beyond the aperture limit toward an adjacent wall,
          the penalty is proportional to (overlap_deg / fov_h) and inversely
          proportional to the distance to that wall (closer = worse).
        - A camera in a corner (very small aperture) is handled gracefully:
          we just return the least-bad multiplier.

        Returns float in (0, 1]:
          1.0 → no overlap with adjacent walls
          → 0  → cone fully buried in adjacent wall
        """
        (lim_left, dist_left), (lim_right, dist_right) = wall_lim
        if lim_left is None:
            return 1.0

        half = fov_h / 2.0
        cone_left  = cam_angle - half
        cone_right = cam_angle + half

        penalty = 0.0

        # ── Left adjacent wall ────────────────────────────────────────────
        # cone extends past lim_left → overlap on the left side
        overlap_left = lim_left - cone_left   # positive if cone pokes left
        if overlap_left > 0:
            d_left = max(dist_left, 0.3)      # cap at 0.3m to avoid /0
            # weight: close wall matters more (1/d), normalise by fov
            penalty += (overlap_left / fov_h) * min(1.0, 1.5 / d_left)

        # ── Right adjacent wall ───────────────────────────────────────────
        overlap_right = cone_right - lim_right
        if overlap_right > 0:
            d_right = max(dist_right, 0.3)
            penalty += (overlap_right / fov_h) * min(1.0, 1.5 / d_right)

        return max(0.05, 1.0 - penalty)


    def _best_angle_in_aperture(x, y, normal_angle, zone_targets):
        """
        For each (orient, height): sweep angles that fit entirely inside the
        wall aperture, rank by aim_quality penalised by adjacent-wall overlap,
        return best clamped candidate.
        Yields (orient, zh, clamped_angle).
        """
        wall_lim = wall_angular_limits(x, y, wall_segs)
        mid = _aperture_midpoint(wall_lim)
        if mid is None:
            return
        lo, hi = _aperture_span(wall_lim, mid)

        for orient in ("L", "P"):
            fov_h_ori, _ = get_fov(cam_A, orient)
            half_ori = fov_h_ori / 2.0

            if fov_h_ori >= (hi - lo):
                # FOV wider than aperture: test BOTH butt positions.
                # Butt-left  : push cone so its left  edge touches lim_left
                # Butt-right : push cone so its right edge touches lim_right
                # Pick whichever gives better (aim_quality × adj_wall_penalty).
                valid_offsets = [lo + half_ori, hi - half_ori]
            else:
                valid_offsets = list(np.arange(lo + half_ori,
                                               hi - half_ori + 0.1, 5.0))
                if not valid_offsets:
                    valid_offsets = [0.0]

            best_angle = None
            best_q = -1.0
            for offset in valid_offsets:
                ta = mid + offset
                dn = (ta - normal_angle + 180) % 360 - 180
                if abs(dn) + half_ori > 92:
                    continue
                # aim quality: how well the cone covers the target zone
                q = _aim_quality(ta, fov_h_ori, x, y, zone_targets)
                # adjacent wall penalty: penalise cones pointing into nearby walls
                adj_mult = _adjacent_wall_penalty(ta, fov_h_ori, wall_lim)
                q *= adj_mult
                if q > best_q:
                    best_q = q
                    best_angle = ta

            if best_angle is None:
                best_angle = mid

            clamped_ta, _ = _clamp_angle_to_aperture(best_angle, fov_h_ori, wall_lim)

            for zh in cam_A.height_options:
                yield (orient, zh, clamped_ta)

    def _add_wall_cands(x, y, normal_angle, cands, seen):
        """Generate candidates at wall position (x,y)."""
        # Filter zone_targets to those within max_range of this camera position
        # → the camera aims toward what it can actually see, not a fixed centre
        max_r = cam_A.max_range
        reachable = [(px, py, w) for (px, py, w) in zone_targets_global
                     if math.sqrt((px-x)**2 + (py-y)**2) <= max_r]
        zone_targets = reachable if reachable else zone_targets_global
        for (orient, zh, clamped_ta) in _best_angle_in_aperture(x, y, normal_angle, zone_targets):
            key = (round(x, 2), round(y, 2), round(clamped_ta % 360, 1), orient, zh)
            if key not in seen:
                seen.add(key)
                cands.append((x, y, clamped_ta, orient, zh))

    def _add_corner_cands(xc, yc, cands, seen):
        """Generate candidates at corner position (xc,yc)."""
        tgt_x_c = float(np.clip(xc, analysis_x_start + 1.0, analysis_x_end - 1.0))
        # Check LOS to at least one target
        targets_c = [(tgt_x_c, walk_y),
                     ((analysis_x_start + analysis_x_end) / 2.0, walk_y)]
        has_los = any(
            has_line_of_sight(xc, yc, tx, ty, wall_segs, obstacles, room_height,
                              cam_z=max(cam_A.height_options),
                              target_z=human_height / 2.0)
            for (tx, ty) in targets_c
        )
        if not has_los:
            return

        wall_lim = wall_angular_limits(xc, yc, wall_segs)
        mid = _aperture_midpoint(wall_lim)
        if mid is None:
            return
        lo, hi = _aperture_span(wall_lim, mid)

        max_r = cam_A.max_range
        reachable_c = [(px, py, w) for (px, py, w) in zone_targets_global
                       if math.sqrt((px-xc)**2 + (py-yc)**2) <= max_r]
        zone_targets = reachable_c if reachable_c else zone_targets_global

        for orient in ("L", "P"):
            fov_h_ori, _ = get_fov(cam_A, orient)
            half_ori = fov_h_ori / 2.0

            if fov_h_ori >= (hi - lo):
                # Test both butt positions — pick the one with best score
                valid_offsets = [lo + half_ori, hi - half_ori]
            else:
                valid_offsets = list(np.arange(lo + half_ori,
                                               hi - half_ori + 0.1, 5.0))
                if not valid_offsets:
                    valid_offsets = [0.0]

            best_angle = None
            best_q = -1.0
            for offset in valid_offsets:
                ta = mid + offset
                q = _aim_quality(ta, fov_h_ori, xc, yc, zone_targets)
                q *= _adjacent_wall_penalty(ta, fov_h_ori, wall_lim)
                if q > best_q:
                    best_q = q
                    best_angle = ta

            if best_angle is None:
                best_angle = mid

            clamped_ta, _ = _clamp_angle_to_aperture(best_angle, fov_h_ori, wall_lim)

            for zh in cam_A.height_options:
                key = (round(xc, 2), round(yc, 2),
                       round(clamped_ta % 360, 1), orient, zh)
                if key not in seen:
                    seen.add(key)
                    cands.append((xc, yc, clamped_ta, orient, zh))

    # ── Room wall segments ────────────────────────────────────────────────
    for i in range(len(corners_loop) - 1):
        x0, y0 = corners_loop[i]
        x1, y1 = corners_loop[i+1]
        seg_len = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        if seg_len < 0.01:
            continue

        dx_w, dy_w  = (x1-x0)/seg_len, (y1-y0)/seg_len
        xm, ym      = (x0+x1)/2, (y0+y1)/2
        nx, ny      = -dy_w, dx_w
        if not point_in_room(xm + nx*0.1, ym + ny*0.1, room_corners, obstacles, room_height):
            nx, ny = dy_w, -dx_w
        normal_angle = math.degrees(math.atan2(ny, nx))
        n_pts        = max(2, int(seg_len / wall_step))

        for j in range(n_pts + 1):
            t = j / n_pts
            x = x0 + t*(x1-x0)
            y = y0 + t*(y1-y0)
            _add_wall_cands(x, y, normal_angle, cam_A_cands, seen_A)

        # Corner at start of segment
        _add_corner_cands(x0, y0, cam_A_cands, seen_A)

    # ── Obstacle walls (can_mount_camera=True, floor-to-ceiling) ─────────
    for obs in obstacles:
        if obs.get("height", 0) < room_height:
            continue
        if not obs.get("can_mount_camera", False):
            continue
        verts_obs = obs["vertices"]
        for i in range(len(verts_obs)):
            w0 = verts_obs[i]
            w1 = verts_obs[(i+1) % len(verts_obs)]
            x0, y0 = w0; x1, y1 = w1
            seg_len = math.sqrt((x1-x0)**2 + (y1-y0)**2)
            if seg_len < 0.05:
                continue
            dx_w, dy_w = (x1-x0)/seg_len, (y1-y0)/seg_len
            xm, ym     = (x0+x1)/2, (y0+y1)/2
            nx, ny     = -dy_w, dx_w
            test_a = point_in_room(xm + nx*0.15, ym + ny*0.15, room_corners, obstacles, room_height)
            test_b = point_in_room(xm - nx*0.15, ym - ny*0.15, room_corners, obstacles, room_height)
            if test_a and not test_b:
                pass   # normal already correct
            elif test_b and not test_a:
                nx, ny = -nx, -ny
            elif test_a and test_b:
                # Both sides inside room (e.g. a partition wall).
                # Pick the normal that points toward the capture zone centre.
                dot = nx*(tgt_x - xm) + ny*(tgt_y - ym)
                if dot < 0:
                    nx, ny = -nx, -ny
            else:
                continue   # neither side is in the room — skip
            normal_angle = math.degrees(math.atan2(ny, nx))
            n_pts = max(2, int(seg_len / wall_step))
            for j in range(n_pts + 1):
                t = j / n_pts
                x = x0 + t*(x1-x0)
                y = y0 + t*(y1-y0)
                _add_wall_cands(x, y, normal_angle, cam_A_cands, seen_A)

    # ── Tripod cameras (cam_B) ────────────────────────────────────────────
    if cam_B and cam_B.max_count > 0:
        xs_r = [c[0] for c in room_corners]
        ys_r = [c[1] for c in room_corners]
        walk_margin = cam_B.walk_axis_margin

        for xi in np.arange(min(xs_r)+0.4, max(xs_r), tripod_grid_step):
            for yi in np.arange(min(ys_r)+0.4, max(ys_r), tripod_grid_step):
                if not point_in_room(xi, yi, room_corners, obstacles, room_height):
                    continue
                if abs(yi - walk_y) < walk_margin:
                    continue
                if not has_line_of_sight(xi, yi, tgt_x, tgt_y,
                                         wall_segs, obstacles, room_height,
                                         cam_z=max(cam_B.height_options),
                                         target_z=human_height / 2.0):
                    continue
                for a_deg in range(0, 360, 20):
                    sts_ang   = math.degrees(math.atan2(sts_y - yi, sts_x - xi))
                    delta_sts = (sts_ang - a_deg + 180) % 360 - 180
                    for orient in ("L", "P"):
                        fov_h_b, _ = get_fov(cam_B, orient)
                        if abs(delta_sts) > (fov_h_b / 2.0 - 10):
                            continue
                        for ih in cam_B.height_options:
                            key = (round(xi, 1), round(yi, 1), a_deg, orient, ih)
                            if key not in seen_B:
                                seen_B.add(key)
                                cam_B_cands.append((xi, yi, float(a_deg), orient, ih))

    return cam_A_cands, cam_B_cands

