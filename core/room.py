"""
room.py
Room geometry, obstacle helpers, line-of-sight, coverage geometry.
All functions are pure (no global state) — they receive cfg as argument.
"""

import math
import numpy as np


# =============================================================
# OBSTACLE HELPERS
# =============================================================

def obs_height(obs):
    return obs["height"]

def obs_label(obs):
    return obs.get("label", "")

def obs_poly_vertices(obs):
    """Returns the list of (x,y) vertices of the obstacle footprint."""
    return obs["vertices"]

def obstacle_segments(obs):
    """Returns all border segments of an obstacle footprint."""
    verts = obs_poly_vertices(obs)
    n = len(verts)
    return [(verts[i], verts[(i+1) % n]) for i in range(n)]

def obs_centroid(obs):
    """Returns the visual centroid (average of vertices)."""
    verts = obs_poly_vertices(obs)
    return (sum(v[0] for v in verts) / len(verts),
            sum(v[1] for v in verts) / len(verts))


# =============================================================
# POINT-IN-POLYGON
# =============================================================

def point_in_polygon(x, y, verts):
    """Ray-casting point-in-polygon test."""
    n = len(verts); inside = False; j = n - 1
    for i in range(n):
        xi, yi = verts[i]; xj, yj = verts[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def point_in_obstacle(x, y, obstacles, room_height):
    """Returns True if (x,y) is inside any floor-to-ceiling obstacle footprint."""
    for obs in obstacles:
        if obs_height(obs) >= room_height:
            if point_in_polygon(x, y, obs_poly_vertices(obs)):
                return True
    return False


def point_in_room(x, y, room_corners, obstacles, room_height):
    """Returns True if (x,y) is inside the room polygon and not inside an obstacle."""
    poly = room_corners; n = len(poly); inside = False; j = n - 1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    if inside and point_in_obstacle(x, y, obstacles, room_height):
        return False
    return inside


# =============================================================
# WALL SEGMENTS (room perimeter + floor-to-ceiling obstacles)
# =============================================================

def build_wall_segments(room_corners, obstacles, room_height):
    """
    Returns the list of all wall segments that block line-of-sight:
    - Room perimeter edges
    - Floor-to-ceiling obstacle border edges
    """
    segments = []
    corners = room_corners
    for i in range(len(corners)):
        segments.append((corners[i], corners[(i + 1) % len(corners)]))
    for obs in obstacles:
        if obs_height(obs) >= room_height:
            for seg in obstacle_segments(obs):
                segments.append(seg)
    return segments


# =============================================================
# LINE-OF-SIGHT
# =============================================================

def _seg_intersect(p1, p2, p3, p4):
    """Returns True if segment p1-p2 strictly intersects p3-p4."""
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    d1 = cross(p3, p4, p1); d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3); d4 = cross(p1, p2, p4)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def has_line_of_sight(x1, y1, x2, y2,
                      wall_segments, obstacles, room_height,
                      cam_z, target_z):
    """
    Checks that the line (x1,y1,cam_z) → (x2,y2,target_z) is not blocked.
    Stage 1: 2D check against wall_segments (room + floor-to-ceiling obstacles).
    Stage 2: 3D height check for partial-height obstacles.
    """
    EPS = 0.05
    dx, dy = x2 - x1, y2 - y1
    dist = math.sqrt(dx*dx + dy*dy)
    if dist < 1e-6:
        return True
    nx, ny = dx / dist, dy / dist
    p1 = (x1 + nx*EPS, y1 + ny*EPS)
    p2 = (x2 - nx*EPS, y2 - ny*EPS)

    # Stage 1: room walls + floor-to-ceiling obstacles
    for (w1, w2) in wall_segments:
        if _seg_intersect(p1, p2, w1, w2):
            return False

    # Stage 2: partial-height obstacles (3D check)
    for obs in obstacles:
        oh = obs_height(obs)
        if oh >= room_height:
            continue
        verts = obs_poly_vertices(obs)
        segs  = obstacle_segments(obs)
        hits = any(_seg_intersect(p1, p2, w1, w2) for (w1, w2) in segs)
        if not hits:
            if point_in_polygon((x1+x2)/2, (y1+y2)/2, verts):
                hits = True
        if not hits:
            continue
        t_vals = []
        for (w1, w2) in segs:
            rx, ry = p2[0]-p1[0], p2[1]-p1[1]
            sx, sy = w2[0]-w1[0], w2[1]-w1[1]
            denom = rx*sy - ry*sx
            if abs(denom) < 1e-9:
                continue
            t = ((w1[0]-p1[0])*sy - (w1[1]-p1[1])*sx) / denom
            u = ((w1[0]-p1[0])*ry - (w1[1]-p1[1])*rx) / denom
            if -EPS <= t <= 1+EPS and -EPS <= u <= 1+EPS:
                t_vals.append(max(0.0, min(1.0, t)))
        if len(t_vals) < 2:
            if not t_vals:
                continue
            t_vals = [0.0, t_vals[0]] if t_vals[0] > 0.5 else [t_vals[0], 1.0]
        t_mid   = (min(t_vals) + max(t_vals)) / 2.0
        z_at_mid = cam_z + t_mid * (target_z - cam_z)
        if z_at_mid <= oh:
            return False
    return True


# =============================================================
# WALL GEOMETRY HELPERS (for candidate generation)
# =============================================================

def wall_normal_at(cx, cy, wall_segments, room_corners,
                   room_height, obstacles, tol=0.08):
    """
    Returns the inward normal angle (degrees) of the wall the camera is on.
    Falls back to 90° if no surface found within tol.
    """
    best_dist  = tol
    best_angle = 90.0

    for (w0, w1) in wall_segments:
        x0, y0 = w0; x1, y1 = w1
        seg_dx, seg_dy = x1-x0, y1-y0
        seg_len2 = seg_dx**2 + seg_dy**2
        if seg_len2 < 1e-9:
            continue
        t  = max(0.0, min(1.0, ((cx-x0)*seg_dx + (cy-y0)*seg_dy) / seg_len2))
        px, py = x0 + t*seg_dx, y0 + t*seg_dy
        d = math.sqrt((cx-px)**2 + (cy-py)**2)
        if d < best_dist:
            best_dist = d
            seg_len = math.sqrt(seg_len2)
            nx, ny  = -seg_dy/seg_len, seg_dx/seg_len
            mid_x, mid_y = (x0+x1)/2, (y0+y1)/2
            if not point_in_room(mid_x + nx*0.1, mid_y + ny*0.1,
                                 room_corners, obstacles, room_height):
                nx, ny = -nx, -ny
            best_angle = math.degrees(math.atan2(ny, nx))
    return best_angle


def wall_angular_limits(cx, cy, wall_segments, tol=0.12):
    """
    Returns ((lim_left, dist_left), (lim_right, dist_right)):
    angular limits of the open inward aperture from (cx,cy) on its wall.
    Handles corners (two segments meeting) correctly.
    """
    from collections import Counter

    close_segs = []
    for (w0, w1) in wall_segments:
        x0, y0 = w0; x1, y1 = w1
        seg_dx, seg_dy = x1-x0, y1-y0
        seg_len2 = seg_dx**2 + seg_dy**2
        if seg_len2 < 1e-9:
            continue
        t  = max(0.0, min(1.0, ((cx-x0)*seg_dx + (cy-y0)*seg_dy) / seg_len2))
        px, py = x0 + t*seg_dx, y0 + t*seg_dy
        d = math.sqrt((cx-px)**2 + (cy-py)**2)
        if d < tol:
            close_segs.append((d, w0, w1))

    if not close_segs:
        return ((None, 0.0), (None, 0.0))

    min_d      = min(d for (d, _, _) in close_segs)
    CORNER_TOL = 0.04
    active     = [(d, w0, w1) for (d, w0, w1) in close_segs if d <= min_d + CORNER_TOL]

    all_ep = []
    for (d, w0, w1) in active:
        all_ep.append(w0); all_ep.append(w1)

    def _key(pt): return (round(pt[0], 3), round(pt[1], 3))

    cnt = Counter(_key(ep) for ep in all_ep)

    ep_info = []
    for (d_seg, w0, w1) in active:
        for ep in (w0, w1):
            if len(active) >= 2 and cnt[_key(ep)] > 1:
                continue
            ang      = math.degrees(math.atan2(ep[1]-cy, ep[0]-cx))
            dist_adj = _dist_to_adjacent_wall(cx, cy, ang, wall_segments)
            ep_info.append((ang, dist_adj))

    if len(ep_info) < 2:
        return ((None, 0.0), (None, 0.0))

    # ── Sort by angle relative to inward wall normal ──────────────────────
    # Raw degree sort (e.g. 350° vs 10°) gives wrong left/right assignment.
    # Sort relative to the inward normal so "left" and "right" are geometrically
    # correct from the camera's perspective looking into the room.
    normal_angles = []
    # Centroid of ALL wall segment endpoints ≈ room centroid (always inside)
    all_wall_pts = [p for (w0, w1) in wall_segments for p in (w0, w1)]
    room_cx = sum(p[0] for p in all_wall_pts) / len(all_wall_pts)
    room_cy = sum(p[1] for p in all_wall_pts) / len(all_wall_pts)
    for (d_seg, w0, w1) in active:
        seg_dx = w1[0]-w0[0]; seg_dy = w1[1]-w0[1]
        seg_len = math.sqrt(seg_dx**2 + seg_dy**2)
        if seg_len < 1e-6:
            continue
        nx, ny = -seg_dy/seg_len, seg_dx/seg_len
        mid_x, mid_y = (w0[0]+w1[0])/2, (w0[1]+w1[1])/2
        # Point normal toward the room centroid
        dot = nx*(room_cx - mid_x) + ny*(room_cy - mid_y)
        if dot < 0:
            nx, ny = -nx, -ny
        normal_angles.append(math.degrees(math.atan2(ny, nx)))

    if not normal_angles:
        ep_info.sort(key=lambda v: v[0])
        return (ep_info[0], ep_info[-1])

    sin_avg = sum(math.sin(math.radians(a)) for a in normal_angles) / len(normal_angles)
    cos_avg = sum(math.cos(math.radians(a)) for a in normal_angles) / len(normal_angles)
    ref_angle = math.degrees(math.atan2(sin_avg, cos_avg))

    def _rel_to_normal(ang_abs):
        return (ang_abs - ref_angle + 180) % 360 - 180

    ep_info_rel = [(ang, dist, _rel_to_normal(ang)) for (ang, dist) in ep_info]
    ep_info_rel.sort(key=lambda v: v[2])   # sort by relative angle
    lim_left,  dist_left  = ep_info_rel[0][0],  ep_info_rel[0][1]
    lim_right, dist_right = ep_info_rel[-1][0], ep_info_rel[-1][1]
    return ((lim_left, dist_left), (lim_right, dist_right))


def _dist_to_adjacent_wall(cx, cy, limit_angle_deg, wall_segments, own_seg_tol=0.12):
    """
    Ray-cast from (cx,cy) in the given direction; return distance to the
    nearest wall segment that is NOT the camera's own mounting surface.
    """
    best = 999.0
    rad  = math.radians(limit_angle_deg)
    dx, dy = math.cos(rad), math.sin(rad)

    for (w0, w1) in wall_segments:
        x0, y0 = w0; x1, y1 = w1
        seg_dx, seg_dy = x1-x0, y1-y0
        seg_len2 = seg_dx**2 + seg_dy**2
        if seg_len2 < 1e-9:
            continue
        t_own = max(0.0, min(1.0, ((cx-x0)*seg_dx + (cy-y0)*seg_dy) / seg_len2))
        px_own = x0 + t_own*seg_dx; py_own = y0 + t_own*seg_dy
        if math.sqrt((cx-px_own)**2 + (cy-py_own)**2) < own_seg_tol:
            continue
        denom = dx*seg_dy - dy*seg_dx
        if abs(denom) < 1e-9:
            continue
        t_ray = ((x0-cx)*seg_dy - (y0-cy)*seg_dx) / denom
        s_seg = ((x0-cx)*dy    - (y0-cy)*dx)    / denom
        if t_ray > 0.01 and 0.0 <= s_seg <= 1.0:
            best = min(best, t_ray)
    return best


def cone_wall_spill(cam_angle, fov_h, wall_lim):
    """
    Weighted spill of cone [cam_angle ± fov_h/2] beyond the wall aperture.
    Spill on each side weighted by 1/distance_to_adjacent_wall.
    Returns 0.0 if cone fits within the aperture.
    """
    (lim_left, dist_left), (lim_right, dist_right) = wall_lim
    if lim_left is None:
        return 0.0

    half = fov_h / 2.0
    def rel(a, ref): return (a - ref + 180) % 360 - 180

    rl_left  = rel(lim_left,  cam_angle)
    rl_right = rel(lim_right, cam_angle)
    lo = min(rl_left, rl_right)
    hi = max(rl_left, rl_right)

    if not (lo <= 0 <= hi):
        spill_left  = max(0.0, lo - (-half)) if (-half) > lo else 0.0
        spill_right = max(0.0, half - hi)    if half    > hi else 0.0
    else:
        spill_left  = max(0.0, lo - (-half))
        spill_right = max(0.0, half - hi)

    w_left  = 1.0 / max(dist_left,  0.3)
    w_right = 1.0 / max(dist_right, 0.3)
    return spill_left * w_left + spill_right * w_right


# =============================================================
# COVERAGE GEOMETRY
# =============================================================

def cam_d_min(cam_z, fov_v_deg, fixed_tilt_rad,
              human_height, human_foot_z, human_head_z,
              v_thresh=0.9, step=0.01, max_search=15.0):
    """
    Minimum horizontal distance at which this camera can see v_thresh of the body.
    """
    half_fov = math.radians(fov_v_deg / 2.0)
    for d in np.arange(step, max_search, step):
        tilt = fixed_tilt_rad if fixed_tilt_rad is not None \
               else math.atan2(human_height / 2.0 - cam_z, d)
        cone_top   = tilt + half_fov
        cone_bot   = tilt - half_fov
        angle_feet = math.atan2(human_foot_z - cam_z, d)
        angle_head = math.atan2(human_head_z  - cam_z, d)
        body_span  = angle_head - angle_feet
        if body_span <= 0:
            continue
        vis = max(0.0, min(angle_head, cone_top) - max(angle_feet, cone_bot))
        if vis / body_span >= v_thresh:
            return d
    return max_search


def vertical_body_coverage(cam_x, cam_y, cam_z, pt_x, pt_y, fov_v_deg,
                            human_height, human_foot_z, human_head_z,
                            v_thresh=0.9, fixed_tilt_rad=None):
    """
    Fraction of the human body (foot→head) seen by the camera at (pt_x, pt_y).
    Returns 0.0 if point is closer than d_min (hard geometry filter).
    """
    horiz_dist = math.sqrt((pt_x - cam_x)**2 + (pt_y - cam_y)**2)
    if horiz_dist < 0.01:
        return 0.0
    d_min = cam_d_min(cam_z, fov_v_deg, fixed_tilt_rad,
                      human_height, human_foot_z, human_head_z,
                      v_thresh=v_thresh)
    if horiz_dist < d_min:
        return 0.0
    tilt = fixed_tilt_rad if fixed_tilt_rad is not None \
           else math.atan2(human_height / 2.0 - cam_z, horiz_dist)
    half_fov    = math.radians(fov_v_deg / 2.0)
    cone_top    = tilt + half_fov
    cone_bot    = tilt - half_fov
    angle_feet  = math.atan2(human_foot_z - cam_z, horiz_dist)
    angle_head  = math.atan2(human_head_z  - cam_z, horiz_dist)
    vis_min     = max(angle_feet, cone_bot)
    vis_max     = min(angle_head, cone_top)
    if vis_max <= vis_min:
        return 0.0
    body_span = angle_head - angle_feet
    return min(1.0, (vis_max - vis_min) / body_span) if body_span > 0 else 0.0


def point_in_wedge(px, py, cx, cy, angle_deg, fov_h, radius, min_range,
                   wall_segments, obstacles, room_height,
                   human_height, cam_z):
    """
    Returns (in_wedge: bool, distance: float).
    True if (px,py) is inside the horizontal cone AND has line-of-sight.
    """
    dx, dy = px - cx, py - cy
    dist   = math.sqrt(dx*dx + dy*dy)
    if dist > radius or dist < min_range:
        return False, 0.0
    pt_angle = math.degrees(math.atan2(dy, dx))
    delta    = (pt_angle - angle_deg + 180) % 360 - 180
    if abs(delta) > fov_h / 2:
        return False, 0.0
    if not has_line_of_sight(cx, cy, px, py,
                             wall_segments, obstacles, room_height,
                             cam_z=cam_z, target_z=human_height / 2.0):
        return False, 0.0
    return True, dist


def cam_fixed_tilt(cx, cy, cam_z, angle_deg, walk_y, human_height):
    """
    Fixed tilt of a wall-mounted camera pointing toward the corridor at walk_y.
    """
    angle_rad = math.radians(angle_deg)
    dy_dir    = math.sin(angle_rad)
    if abs(dy_dir) > 0.01:
        dist_to_walk = (walk_y - cy) / dy_dir
        dist_ref = max(dist_to_walk, 0.1) if dist_to_walk > 0 \
                   else max(abs(cy - walk_y), 0.1)
    else:
        dist_ref = max(abs(cy - walk_y), 0.1)
    return math.atan2(human_height / 2.0 - cam_z, dist_ref)


def cam_side(cy, walk_y):
    """Returns 'S' if camera is south of the corridor, 'N' otherwise."""
    return 'S' if cy < walk_y else 'N'

