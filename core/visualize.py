"""
visualize.py
All visualisation functions for the camera optimiser.
Receives explicit cfg/state — no global state.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
from matplotlib.patches import Wedge, Rectangle, Polygon

from .room import (obs_poly_vertices, obs_label, obs_centroid, obs_height,
                   cam_d_min, cam_side, point_in_room, point_in_wedge,
                   vertical_body_coverage, cam_fixed_tilt)
from .scoring import get_fov, count_cameras_3d, score_at_point


# =============================================================
# HELPERS
# =============================================================

def _min_dist_full_coverage(cam_z, fov_v_deg, human_h, human_fz, human_hz,
                             v_thresh=0.9, fixed_tilt_rad=None):
    return cam_d_min(cam_z, fov_v_deg, fixed_tilt_rad,
                     human_h, human_fz, human_hz, v_thresh=v_thresh)


# =============================================================
# TOP-DOWN VIEW (cones)
# =============================================================

def draw_top_view(ax, cam_A_list, cam_B_list, score, cfg, state):
    cam_A   = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B   = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)
    human_h = cfg.HUMAN_HEIGHT
    human_fz = cfg.HUMAN_FOOT_Z
    human_hz = cfg.HUMAN_HEAD_Z

    walk_y           = state["walk_y"]
    walk_x_start     = state["walk_x_start"]
    walk_x_end       = state["walk_x_end"]
    analysis_x_start = state["analysis_x_start"]
    analysis_x_end   = state["analysis_x_end"]
    sts_x            = state["sts_x"]
    sts_y            = state["sts_y"]
    sts_radius       = cfg.STS_RADIUS

    ax.set_facecolor('white')

    # Room
    corners = cfg.ROOM_CORNERS + [cfg.ROOM_CORNERS[0]]
    ax.fill([c[0] for c in corners], [c[1] for c in corners], color='#f5f5f5', zorder=0)
    ax.plot([c[0] for c in corners], [c[1] for c in corners], 'k-', lw=2.5, zorder=5)

    # Corridor & zones
    ax.plot([walk_x_start, walk_x_end], [walk_y, walk_y],
            color='#2ca02c', lw=1.5, ls='--', zorder=4, alpha=0.7)
    ax.add_patch(Rectangle((analysis_x_start, walk_y - 0.55),
                            analysis_x_end - analysis_x_start, 1.1,
                            fill=True, facecolor='#d5f5d5', edgecolor='#2ca02c',
                            lw=2, ls='--', zorder=2, alpha=0.6))
    ax.text((analysis_x_start + analysis_x_end) / 2, walk_y + 0.68,
            "Analysis zone", ha='center', fontsize=8, color='#1a7a1a', fontweight='bold')
    ax.add_patch(plt.Circle((sts_x, sts_y), sts_radius,
                             facecolor='#ffe0b2', edgecolor='darkorange', lw=2, zorder=3, alpha=0.8))
    ax.text(sts_x, sts_y, "STS", ha='center', va='center',
            fontsize=8, color='darkorange', fontweight='bold')

    # ── Polygon capture zones ─────────────────────────────────────────────
    for zone in cfg.capture_zones:
        if zone.type != "polygon":
            continue
        verts = getattr(zone, "_translated_vertices", None) or zone.vertices
        if len(verts) < 3:
            continue
        ax.add_patch(Polygon(verts, closed=True,
                             facecolor='#d5f5d5', edgecolor='#2ca02c',
                             lw=2, ls='-.', zorder=2, alpha=0.5))
        cx_z = sum(v[0] for v in verts) / len(verts)
        cy_z = sum(v[1] for v in verts) / len(verts)
        ax.text(cx_z, cy_z, zone.id, ha='center', va='center',
                fontsize=7, color='#1a7a1a', fontweight='bold', zorder=3)

    # Obstacles
    for obs in cfg.obstacles:
        oh     = obs_height(obs)
        alpha  = 0.85 if oh >= cfg.ROOM_HEIGHT else 0.45
        color  = '#555555' if oh >= cfg.ROOM_HEIGHT else '#aaaaaa'
        verts  = obs_poly_vertices(obs)
        ax.add_patch(Polygon(verts, closed=True, facecolor=color,
                             edgecolor='black', lw=1.5, zorder=6, alpha=alpha))
        cx_obs, cy_obs = obs_centroid(obs)
        ax.text(cx_obs, cy_obs, obs_label(obs), ha='center', va='center',
                fontsize=5.5, color='white', fontweight='bold', zorder=7)

    # Cam A cones
    for idx, (cx, cy, angle, orient, zh) in enumerate(cam_A_list):
        fov_h, fov_v = get_fov(cam_A, orient)
        theta1, theta2 = angle - fov_h/2, angle + fov_h/2

        angle_rad = math.radians(angle)
        dy_dir    = math.sin(angle_rad)
        if abs(dy_dir) > 0.01:
            dtw = (walk_y - cy) / dy_dir
            dist_perp = max(dtw, 0.1) if dtw > 0 else max(abs(cy - walk_y), 0.1)
        else:
            dist_perp = max(abs(cy - walk_y), 0.1)
        fixed_tilt  = math.atan2(human_h / 2.0 - zh, dist_perp)
        d_min_draw  = max(_min_dist_full_coverage(zh, fov_v, human_h, human_fz, human_hz,
                                                   fixed_tilt_rad=fixed_tilt),
                          cam_A.min_range)

        ax.add_patch(Wedge((cx, cy), d_min_draw, theta1, theta2,
                           facecolor='#888888', alpha=0.35, linewidth=0, zorder=3))
        ax.add_patch(Wedge((cx, cy), d_min_draw, theta1, theta2,
                           width=0.04, facecolor='white', alpha=0.8, linewidth=0, zorder=4))
        n_steps = 25
        radii   = np.linspace(d_min_draw, cam_A.max_range, n_steps + 1)
        for k in range(n_steps):
            ri = radii[k]; ro = radii[k+1]
            a_step = 0.30 * (1.0 / (1.0 + 0.025 * ((ri+ro)/2)**2))
            ax.add_patch(Wedge((cx, cy), ro, theta1, theta2, width=ro-ri,
                               color=cam_A.color, alpha=a_step, linewidth=0, zorder=3))
        ax.add_patch(Wedge((cx, cy), cam_A.max_range, theta1, theta2,
                           fill=False, edgecolor=cam_A.color, lw=0.8, alpha=0.6, zorder=4))
        marker = 'o' if orient == 'L' else '^'
        ax.plot(cx, cy, marker=marker, color=cam_A.color, ms=10,
                markeredgecolor='white', markeredgewidth=1.0, zorder=7)
        ax.text(cx, cy+0.18, f"A{idx+1}{'P' if orient=='P' else ''}\n{zh:.1f}m",
                fontsize=5.5, ha='center', color=cam_A.color, fontweight='bold', zorder=8)

    # Cam B cones
    if cam_B:
        for idx, (cx, cy, angle, orient, ih) in enumerate(cam_B_list):
            fov_h, fov_v = get_fov(cam_B, orient)
            theta1, theta2 = angle - fov_h/2, angle + fov_h/2
            d_min_draw = max(_min_dist_full_coverage(ih, fov_v, human_h, human_fz, human_hz),
                             cam_B.min_range)
            ax.add_patch(Wedge((cx, cy), d_min_draw, theta1, theta2,
                               facecolor='#888888', alpha=0.35, linewidth=0, zorder=3))
            ax.add_patch(Wedge((cx, cy), d_min_draw, theta1, theta2,
                               width=0.04, facecolor='white', alpha=0.8, linewidth=0, zorder=4))
            radii = np.linspace(d_min_draw, cam_B.max_range, 26)
            for k in range(25):
                ri = radii[k]; ro = radii[k+1]
                a_step = 0.30 * (1.0 / (1.0 + 0.025 * ((ri+ro)/2)**2))
                ax.add_patch(Wedge((cx, cy), ro, theta1, theta2, width=ro-ri,
                                   color=cam_B.color, alpha=a_step, linewidth=0, zorder=3))
            ax.add_patch(Wedge((cx, cy), cam_B.max_range, theta1, theta2,
                               fill=False, edgecolor=cam_B.color, lw=0.8, alpha=0.6, zorder=4))
            marker_b = 's' if orient == 'P' else 'D'
            ax.plot(cx, cy, marker=marker_b, color=cam_B.color, ms=10,
                    markeredgecolor='white', markeredgewidth=1.0, zorder=7)
            ax.text(cx, cy+0.18, f"B{idx+1}{'P' if orient=='P' else ''}\n{ih:.1f}m",
                    fontsize=5.5, ha='center', color=cam_B.color, fontweight='bold', zorder=8)

    ax.set_aspect('equal')
    _xs = [c[0] for c in cfg.ROOM_CORNERS]; _ys = [c[1] for c in cfg.ROOM_CORNERS]
    _pad = max((max(_xs)-min(_xs)), (max(_ys)-min(_ys))) * 0.08 + 0.5
    ax.set_xlim(min(_xs) - _pad, max(_xs) + _pad)
    ax.set_ylim(min(_ys) - _pad, max(_ys) + _pad)
    ax.set_xlabel("X (m)", fontsize=9); ax.set_ylabel("Y (m)", fontsize=9)
    ax.set_title(f"Top view  —  Score: {score:.1f}  |  bilateral={cfg.BILATERAL_WEIGHT}",
                 fontsize=9, fontweight='bold')
    ax.grid(True, ls='--', alpha=0.25, color='gray')


# =============================================================
# XZ SIDE VIEW
# =============================================================

def draw_side_view_xz(ax, cam_A_list, cam_B_list, cfg, state):
    cam_A    = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B    = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)
    walk_y   = state["walk_y"]
    walk_x_start = state["walk_x_start"]; walk_x_end = state["walk_x_end"]
    analysis_x_start = state["analysis_x_start"]; analysis_x_end = state["analysis_x_end"]
    sts_x    = state["sts_x"]
    human_h  = cfg.HUMAN_HEIGHT
    obstacles = cfg.obstacles
    wall_segs = state["wall_segments"]
    room_h   = cfg.ROOM_HEIGHT

    CELL    = 0.25
    MAX_CAM = 10
    base_cmap = plt.get_cmap('YlOrRd')
    colors_d = ['#1a1a2e'] + [base_cmap(i / MAX_CAM) for i in range(1, MAX_CAM + 1)]
    cmap = mcolors.ListedColormap(colors_d)
    norm = mcolors.BoundaryNorm(list(range(0, MAX_CAM + 2)), cmap.N)

    ax.set_facecolor('#1a1a2e')
    xs = np.arange(min(c[0] for c in cfg.ROOM_CORNERS), max(c[0] for c in cfg.ROOM_CORNERS), CELL)
    zs = np.arange(0.0, room_h, CELL)

    for xi in xs:
        xm = xi + CELL / 2
        for zi in zs:
            zm = zi + CELL / 2
            count = 0.0
            for (cx, cy, angle, orient, zh) in cam_A_list:
                fov_h, fov_v = get_fov(cam_A, orient)
                in_h, _ = point_in_wedge(xm, walk_y, cx, cy, angle, fov_h,
                                          cam_A.max_range, cam_A.min_range,
                                          wall_segs, obstacles, room_h, human_h, cam_z=zh)
                if not in_h: continue
                horiz_d = math.sqrt((xm-cx)**2 + (walk_y-cy)**2)
                if horiz_d < 0.01: continue
                dp = max(abs(cy - walk_y), 0.1)
                ft = math.atan2(human_h / 2.0 - zh, dp)
                hfv = math.radians(fov_v / 2.0)
                if ft - hfv <= math.atan2(zm - zh, horiz_d) <= ft + hfv:
                    count += 1.0
            if cam_B:
                for (cx, cy, angle, orient, ih) in cam_B_list:
                    fov_h, fov_v = get_fov(cam_B, orient)
                    in_h, _ = point_in_wedge(xm, walk_y, cx, cy, angle, fov_h,
                                              cam_B.max_range, cam_B.min_range,
                                              wall_segs, obstacles, room_h, human_h, cam_z=ih)
                    if not in_h: continue
                    horiz_d = math.sqrt((xm-cx)**2 + (walk_y-cy)**2)
                    if horiz_d < 0.01: continue
                    tilt_i = math.atan2(human_h / 2.0 - ih, horiz_d)
                    hfv    = math.radians(fov_v / 2.0)
                    if tilt_i - hfv <= math.atan2(zm - ih, horiz_d) <= tilt_i + hfv:
                        count += 0.5
            color = cmap(norm(int(min(count, MAX_CAM))))
            ax.add_patch(Rectangle((xi, zi), CELL, CELL, facecolor=color, edgecolor='none', zorder=2))

    ax.axhline(0, color='saddlebrown', lw=2.5, zorder=6)
    ax.axhline(room_h, color='white', lw=1.5, ls='--', alpha=0.5, zorder=6)
    ax.axvline(walk_x_start, color='#74b9ff', lw=2, zorder=7)
    ax.axvline(walk_x_end,   color='#74b9ff', lw=2, zorder=7)
    ax.axvline(analysis_x_start, color='#55efc4', lw=1.5, ls='--', zorder=7)
    ax.axvline(analysis_x_end,   color='#55efc4', lw=1.5, ls='--', zorder=7)
    ax.axvline(sts_x, color='#fdcb6e', lw=1.5, ls=':', zorder=7)

    for obs in obstacles:
        oh = obs_height(obs)
        verts = obs_poly_vertices(obs)
        ox0 = min(v[0] for v in verts); ox1 = max(v[0] for v in verts)
        ax.add_patch(Rectangle((ox0, 0), ox1-ox0, oh,
                               facecolor='#666666', edgecolor='white', lw=1, zorder=8, alpha=0.75))
        ax.text((ox0+ox1)/2, oh/2, obs_label(obs), ha='center', va='center',
                fontsize=5, color='white', fontweight='bold', zorder=9,
                rotation=90 if (ox1-ox0) < 0.5 else 0)

    colors_z = plt.cm.tab20(np.linspace(0, 0.9, max(len(cam_A_list), 1)))
    for idx, (cx, cy, _, orient, zh) in enumerate(cam_A_list):
        ax.plot(cx, zh, 'o', color=colors_z[idx], ms=8, markeredgecolor='white', markeredgewidth=1, zorder=9)
        ax.text(cx, zh+0.12, f"A{idx+1}\n{zh:.1f}m", ha='center', fontsize=5.5, color=colors_z[idx], fontweight='bold', zorder=10)
    if cam_B:
        for idx, (cx, cy, _, orient, ih) in enumerate(cam_B_list):
            ax.plot(cx, ih, 's', color=cam_B.color, ms=8, markeredgecolor='white', markeredgewidth=1, zorder=9)
            ax.text(cx, ih+0.12, f"B{idx+1}\n{ih:.1f}m", ha='center', fontsize=5.5, color=cam_B.color, fontweight='bold', zorder=10)

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.01, fraction=0.03,
                        ticks=[i + 0.5 for i in range(0, MAX_CAM + 1)])
    cbar.set_ticklabels([f"{i} cam{'s' if i > 1 else ''}" for i in range(0, MAX_CAM + 1)])
    cbar.set_label("Nb cameras (≥90% body)", fontsize=8, color='white')
    cbar.ax.yaxis.set_tick_params(color='white', labelcolor='white', labelsize=7)
    for y in range(1, MAX_CAM + 1):
        cbar.ax.axhline(y, color='white', lw=0.5, alpha=0.6)

    _xs_r = [c[0] for c in cfg.ROOM_CORNERS]
    ax.set_xlim(min(_xs_r) - 0.2, max(_xs_r) + 0.2); ax.set_ylim(-0.25, room_h + 0.25)
    ax.set_xlabel("X (m)", fontsize=9, color='white'); ax.set_ylabel("Z (m) height", fontsize=9, color='white')
    ax.tick_params(colors='white')
    ax.set_title("XZ side view — cameras seeing each height point (≥90% body)", fontsize=9, fontweight='bold', color='white')
    ax.set_facecolor('#1a1a2e')
    for spine in ax.spines.values(): spine.set_edgecolor('white')
    ax.grid(False)


# =============================================================
# XY TOP-DOWN HEATMAP
# =============================================================

def draw_top_heatmap(ax, cam_A_list, cam_B_list, cfg, state):
    CELL    = 0.2
    MAX_CAM = 10
    base_cmap = plt.get_cmap('YlOrRd')
    colors_d = ['#1a1a2e'] + [base_cmap(i / MAX_CAM) for i in range(1, MAX_CAM + 1)]
    cmap = mcolors.ListedColormap(colors_d)
    norm = mcolors.BoundaryNorm(list(range(0, MAX_CAM + 2)), cmap.N)

    walk_y = state["walk_y"]
    analysis_x_start = state["analysis_x_start"]; analysis_x_end = state["analysis_x_end"]
    sts_x = state["sts_x"]; sts_y = state["sts_y"]

    ax.set_facecolor('#1a1a2e')
    corners = cfg.ROOM_CORNERS + [cfg.ROOM_CORNERS[0]]
    ax.fill([c[0] for c in corners], [c[1] for c in corners], color='#16213e', zorder=1)
    ax.plot([c[0] for c in corners], [c[1] for c in corners], color='white', lw=2, zorder=8)

    for xi in np.arange(min(c[0] for c in cfg.ROOM_CORNERS),
                        max(c[0] for c in cfg.ROOM_CORNERS) + CELL, CELL):
        xm = xi + CELL / 2
        for yi in np.arange(min(c[1] for c in cfg.ROOM_CORNERS),
                            max(c[1] for c in cfg.ROOM_CORNERS) + CELL, CELL):
            ym = yi + CELL / 2
            if not point_in_room(xm, ym, cfg.ROOM_CORNERS, cfg.obstacles, cfg.ROOM_HEIGHT):
                continue
            count = count_cameras_3d(xm, ym, cam_A_list, cam_B_list, cfg, state)
            if count <= 0: continue
            color = cmap(norm(int(min(count, MAX_CAM))))
            ax.add_patch(Rectangle((xi, yi), CELL, CELL, facecolor=color, edgecolor='none', alpha=0.85, zorder=3))

    ax.plot([state["walk_x_start"], state["walk_x_end"]], [walk_y, walk_y],
            color='#74b9ff', lw=1.5, ls='--', zorder=9, alpha=0.8)
    ax.add_patch(Rectangle((analysis_x_start, walk_y - 0.5),
                            analysis_x_end - analysis_x_start, 1.0,
                            fill=False, edgecolor='#55efc4', lw=2, ls='--', zorder=9))
    ax.add_patch(plt.Circle((sts_x, sts_y), cfg.STS_RADIUS,
                             facecolor='none', edgecolor='#fdcb6e', lw=2, zorder=9))
    ax.text(sts_x, sts_y, "STS", ha='center', va='center', fontsize=7, color='#fdcb6e', fontweight='bold', zorder=10)

    # ── Polygon capture zones ─────────────────────────────────────────────
    for zone in cfg.capture_zones:
        if zone.type != "polygon":
            continue
        verts = getattr(zone, "_translated_vertices", None) or zone.vertices
        if len(verts) < 3:
            continue
        ax.add_patch(Polygon(verts, closed=True,
                             facecolor='none', edgecolor='#55efc4',
                             lw=2, ls='-.', zorder=9, alpha=0.9))
        cx_z = sum(v[0] for v in verts) / len(verts)
        cy_z = sum(v[1] for v in verts) / len(verts)
        ax.text(cx_z, cy_z, zone.id, ha='center', va='center',
                fontsize=6, color='#55efc4', fontweight='bold', zorder=10)

    for obs in cfg.obstacles:
        verts = obs_poly_vertices(obs)
        ax.add_patch(Polygon(verts, closed=True, facecolor='#444444', edgecolor='white', lw=1.2, zorder=10, alpha=0.9))
        cx_obs, cy_obs = obs_centroid(obs)
        ax.text(cx_obs, cy_obs, obs_label(obs), ha='center', va='center', fontsize=5, color='white', fontweight='bold', zorder=11)

    cam_A = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)
    colors_z = plt.cm.tab20(np.linspace(0, 0.9, max(len(cam_A_list), 1)))
    for idx, (cx, cy, angle, orient, zh) in enumerate(cam_A_list):
        ax.plot(cx, cy, 'o', color=colors_z[idx], ms=9, markeredgecolor='white', markeredgewidth=1.2, zorder=11)
        ax.text(cx, cy+0.15, f"A{idx+1}\n{zh:.1f}m", ha='center', fontsize=5.5, color=colors_z[idx], fontweight='bold', zorder=12)
    if cam_B:
        for idx, (cx, cy, angle, orient, ih) in enumerate(cam_B_list):
            ax.plot(cx, cy, 's', color=cam_B.color, ms=9, markeredgecolor='white', markeredgewidth=1.2, zorder=11)
            ax.text(cx, cy+0.15, f"B{idx+1}\n{ih:.1f}m", ha='center', fontsize=5.5, color=cam_B.color, fontweight='bold', zorder=12)

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm); sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.01, fraction=0.03,
                        ticks=[i + 0.5 for i in range(0, MAX_CAM + 1)])
    cbar.set_ticklabels([f"{i} cam{'s' if i > 1 else ''}" for i in range(0, MAX_CAM + 1)])
    cbar.set_label("Nb cameras (≥90% body)", fontsize=8, color='white')
    cbar.ax.yaxis.set_tick_params(color='white', labelcolor='white', labelsize=7)
    for y in range(1, MAX_CAM + 1): cbar.ax.axhline(y, color='white', lw=0.5, alpha=0.6)

    ax.set_aspect('equal')
    _xs_r = [c[0] for c in cfg.ROOM_CORNERS]; _ys_r = [c[1] for c in cfg.ROOM_CORNERS]
    ax.set_xlim(min(_xs_r) - 0.5, max(_xs_r) + 0.5)
    ax.set_ylim(min(_ys_r) - 0.5, max(_ys_r) + 0.5)
    ax.set_xlabel("X (m)", fontsize=9, color='white'); ax.set_ylabel("Y (m)", fontsize=9, color='white')
    ax.tick_params(colors='white')
    for spine in ax.spines.values(): spine.set_edgecolor('white')
    ax.set_title("XY heatmap — 3D coverage (≥90% body)", fontsize=9, fontweight='bold', color='white')
    ax.grid(False)


# =============================================================
# COVERAGE BAR CHART
# =============================================================

def draw_coverage_bar_chart(ax, cam_A_list, cam_B_list, score, cfg, state):
    cam_A = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)
    wall_segs = state["wall_segments"]
    obstacles = cfg.obstacles
    room_h    = cfg.ROOM_HEIGHT
    human_h   = cfg.HUMAN_HEIGHT; human_fz = cfg.HUMAN_FOOT_Z; human_hz = cfg.HUMAN_HEAD_Z
    walk_y    = state["walk_y"]
    walk_x_start = state["walk_x_start"]; walk_x_end = state["walk_x_end"]
    analysis_x_start = state["analysis_x_start"]; analysis_x_end = state["analysis_x_end"]
    sts_x = state["sts_x"]
    target_coverage = cfg.TARGET_COVERAGE
    v_thresh = cfg.opt.vertical_coverage_threshold
    k_dist   = cfg.DIST_QUALITY_K

    _x_min_r = min(c[0] for c in cfg.ROOM_CORNERS)
    _x_max_r = max(c[0] for c in cfg.ROOM_CORNERS)
    x_full = np.arange(_x_min_r, _x_max_r + 0.01, 0.1)
    cov_total, cov_south, cov_north = [], [], []
    cov_scores = []

    for x in x_full:
        tot = 0; s_v = 0.0; n_v = 0.0
        for (cx, cy, angle, orient, zh) in cam_A_list:
            fov_h, fov_v = get_fov(cam_A, orient)
            in_c, _ = point_in_wedge(x, walk_y, cx, cy, angle, fov_h,
                                      cam_A.max_range, cam_A.min_range,
                                      wall_segs, obstacles, room_h, human_h, cam_z=zh)
            if in_c:
                dp = max(abs(cy - walk_y), 0.1)
                ft = math.atan2(human_h / 2.0 - zh, dp)
                v = vertical_body_coverage(cx, cy, zh, x, walk_y, fov_v,
                                           human_h, human_fz, human_hz,
                                           v_thresh=v_thresh, fixed_tilt_rad=ft)
                if v >= v_thresh:
                    tot += 1
                    if cam_side(cy, walk_y) == 'S': s_v = max(s_v, v)
                    else:                           n_v = max(n_v, v)
        if cam_B:
            for (cx, cy, angle, orient, ih) in cam_B_list:
                fov_h, fov_v = get_fov(cam_B, orient)
                in_c, _ = point_in_wedge(x, walk_y, cx, cy, angle, fov_h,
                                          cam_B.max_range, cam_B.min_range,
                                          wall_segs, obstacles, room_h, human_h, cam_z=ih)
                if in_c:
                    v = vertical_body_coverage(cx, cy, ih, x, walk_y, fov_v,
                                               human_h, human_fz, human_hz,
                                               v_thresh=v_thresh)
                    if v >= v_thresh:
                        tot += 0.5
                        if cam_side(cy, walk_y) == 'S': s_v = max(s_v, v * 0.6)
                        else:                           n_v = max(n_v, v * 0.6)
        cov_total.append(tot); cov_south.append(s_v); cov_north.append(n_v)
        cov_scores.append(score_at_point(x, walk_y, cam_A_list, cam_B_list, cfg, state))

    cov = np.array(cov_total)
    scores_arr = np.array(cov_scores)
    ax.axvspan(0, walk_x_start, alpha=0.06, color='gray', zorder=1)
    ax.axvspan(walk_x_end, 13.0, alpha=0.06, color='gray', zorder=1)
    ax.axvspan(walk_x_start, walk_x_end, alpha=0.05, color='steelblue', zorder=1)
    ax.axvspan(analysis_x_start, analysis_x_end, alpha=0.09, color='green', zorder=1)

    # Restauration des barres de couleur rouge->jaune->vert
    norm_bar = mcolors.Normalize(vmin=0, vmax=target_coverage + 1)
    for i in range(len(x_full) - 1):
        ax.bar(x_full[i], cov[i], width=0.1,
               color=plt.cm.RdYlGn(norm_bar(cov[i])), align='edge', zorder=2)
    qty_proxy = mpatches.Patch(facecolor='gray', edgecolor='none', alpha=0.5, label='Quantity (Nb cameras)')

    ax_score = ax.twinx()
    ax_score.plot(x_full, scores_arr, color="lime", linewidth=2.5, zorder=4, label="Quality Score")
    ax_score.set_ylabel("Quality Score", color="green", fontsize=9, fontweight="bold")
    ax_score.tick_params(axis='y', labelcolor="green")
    ax_score.set_ylim(0, 2.2)  # Échelle standardisée fixée à 2.2

    ax.axvline(walk_x_start, color='steelblue', lw=2, ls='-', zorder=5)
    ax.axvline(walk_x_end,   color='steelblue', lw=2, ls='-', zorder=5)
    ax.axvline(analysis_x_start, color='#2ca02c', lw=1.5, ls='--', zorder=5)
    ax.axvline(analysis_x_end,   color='#2ca02c', lw=1.5, ls='--', zorder=5)
    ax.axhline(target_coverage, color='#1a7a1a', lw=2, ls='--', zorder=5,
               label=f"Target {target_coverage} cams")
    ax.axvline(sts_x, color='darkorange', lw=1.5, ls=':', zorder=5, label="STS")
 
    # Calculate total physical cameras to set a fixed Y-axis scale for fair comparison
    total_cams = len(cam_A_list) + (len(cam_B_list) if cam_B_list else 0)
    y_max_scale = total_cams + 1.0
    ax.text((walk_x_start + walk_x_end) / 2, y_max_scale - 0.3, "Corridor", ha='center', fontsize=8, color='steelblue', fontweight='bold')
    ax.text((analysis_x_start + analysis_x_end) / 2, y_max_scale - 0.7, "Analysis zone", ha='center', fontsize=8, color='#1a7a1a', fontweight='bold')

    am6  = (x_full >= analysis_x_start) & (x_full <= analysis_x_end)
    am10 = (x_full >= walk_x_start)     & (x_full <= walk_x_end)
    cs = np.array(cov_south); cn = np.array(cov_north)
    bk6  = np.mean((cs[am6] > 0) & (cn[am6] > 0)) * 100
    bk10 = np.mean((cs[am10] > 0) & (cn[am10] > 0)) * 100
    stats = (f"Zone: >={target_coverage} cams = {100*np.mean(cov[am6]>=target_coverage):.0f}%  | Bilateral = {bk6:.0f}%\n"
             f"Corridor: >={target_coverage} cams = {100*np.mean(cov[am10]>=target_coverage):.0f}%  | Bilateral = {bk10:.0f}%\n"
             f"Score: {score:.2f}")
    ax.text(0.99, 0.99, stats, transform=ax.transAxes, fontsize=8, va='top', ha='right',
            bbox=dict(boxstyle='round', facecolor='white', edgecolor='gray', alpha=0.92))

    _xs_r = [c[0] for c in cfg.ROOM_CORNERS]
    _x_min_r, _x_max_r = min(_xs_r), max(_xs_r)
    ax.set_xlim(_x_min_r - 0.3, _x_max_r + 0.3)
    ax.set_ylim(0, y_max_scale) # Use the fixed scale
    ax.set_xlabel(f"X (m) — room length ({_x_min_r:.0f} → {_x_max_r:.0f}m)", fontsize=9)
    ax.set_ylabel("Nb cameras (≥90% body)", fontsize=9)
    ax.set_title("Coverage along the room — Blue: corridor | Green: analysis zone", fontsize=9, fontweight='bold')
    
    lines_1, labels_1 = ax.get_legend_handles_labels()
    lines_2, labels_2 = ax_score.get_legend_handles_labels()
    ax.legend([qty_proxy] + lines_1 + lines_2, [qty_proxy.get_label()] + labels_1 + labels_2, loc='upper left', fontsize=7, ncol=2, framealpha=0.9)
    ax.grid(True, ls='--', alpha=0.25, axis='y')
    ax.set_xticks(range(int(_x_min_r), int(_x_max_r) + 1))


# =============================================================
# MAIN VISUALISATION ENTRY POINT
# =============================================================

def visualize_solution(cam_A_list, cam_B_list, score, cfg, state,
                       show_window=True, save_path=None):
    """
    Builds and saves the 4-panel visualisation figure.
    """
    fig = plt.figure(figsize=(30, 18), facecolor='#111122')
    gs  = fig.add_gridspec(2, 2, hspace=0.38, wspace=0.32)
    ax_top     = fig.add_subplot(gs[0, 0])
    ax_heatmap = fig.add_subplot(gs[0, 1])
    ax_side    = fig.add_subplot(gs[1, 0])
    ax_bar     = fig.add_subplot(gs[1, 1])
    ax_bar.set_facecolor('white')

    draw_top_view(ax_top,        cam_A_list, cam_B_list, score, cfg, state)
    draw_top_heatmap(ax_heatmap, cam_A_list, cam_B_list, cfg, state)
    draw_side_view_xz(ax_side,   cam_A_list, cam_B_list, cfg, state)
    draw_coverage_bar_chart(ax_bar, cam_A_list, cam_B_list, score, cfg, state)

    cam_A = next(c for c in cfg.camera_sets if c.mounting == "wall")
    walk_y = state["walk_y"]
    n_S = sum(1 for c in cam_A_list if c[1] < walk_y)
    n_N = sum(1 for c in cam_A_list if c[1] >= walk_y)
    n_L = sum(1 for c in cam_A_list if c[3] == 'L')
    n_P = sum(1 for c in cam_A_list if c[3] == 'P')
    fig.suptitle(
        f"Camera optimisation — Markerless Biomechanics Lab\n"
        f"{cam_A.name}: {n_S} SOUTH + {n_N} NORTH  |  "
        f"{n_L}× Landscape + {n_P}× Portrait  |  "
        f"bilateral={cfg.BILATERAL_WEIGHT}",
        fontsize=12, fontweight='bold')

    out_path = save_path if save_path else "optimisation_cameras_resultat.png"
    if out_path and __import__('os').path.dirname(out_path):
        __import__('os').makedirs(__import__('os').path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=150, bbox_inches='tight', facecolor='white')
    if show_window:
        plt.show()
    else:
        plt.close(fig)
