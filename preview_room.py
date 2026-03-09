"""
preview_room.py
───────────────
Top-down visualisation of a lab config file.
Shows: room outline, obstacles, capture zones (colour-coded by priority),
       and a legend.

Usage
─────
    python preview_room.py --config configs/T_zone_direction_change.yaml
    python preview_room.py --config configs/labo_CHU.yaml --save my_room.png
"""

import argparse
import math
import os
import sys

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Polygon as MplPolygon
import matplotlib.patheffects as pe
import numpy as np

# ── allow running from the project root ───────────────────────────────────────
sys.path.insert(0, os.path.dirname(__file__))
from core.config_loader import load_config


# ─────────────────────────────────────────────────────────────────────────────
# Colour palette for capture zones (by priority rank)
# ─────────────────────────────────────────────────────────────────────────────
ZONE_PALETTE = [
    "#a8d8ea",   # light blue  — lowest priority
    "#a8e6cf",   # light green
    "#ffd3b6",   # light orange
    "#ffaaa5",   # light red
    "#ff6b6b",   # stronger red — highest priority
]


def _priority_color(priority, all_priorities):
    """Map a zone priority to a colour from the palette."""
    sorted_p = sorted(set(all_priorities))
    idx = sorted_p.index(priority)
    palette_idx = int(idx / max(len(sorted_p) - 1, 1) * (len(ZONE_PALETTE) - 1))
    return ZONE_PALETTE[palette_idx]


def _all_translated_vertices(zone):
    """
    Returns a list of translated vertex lists for every (dx, dy) combination
    defined in zone.x_offsets × zone.y_offsets.
    """
    base = zone.vertices
    if not base:
        return []
    x_offs = getattr(zone, "x_offsets", [0.0]) or [0.0]
    y_offs = getattr(zone, "y_offsets", [0.0]) or [0.0]
    result = []
    for dy in y_offs:
        for dx in x_offs:
            result.append([(v[0] + dx, v[1] + dy) for v in base])
    return result


def _median_translated_vertices(zone):
    """Returns vertices translated by the median x/y offset."""
    x_offs = sorted(getattr(zone, "x_offsets", [0.0]) or [0.0])
    y_offs = sorted(getattr(zone, "y_offsets", [0.0]) or [0.0])
    mid_dx = x_offs[len(x_offs) // 2]
    mid_dy = y_offs[len(y_offs) // 2]
    return [(v[0] + mid_dx, v[1] + mid_dy) for v in zone.vertices]


def _corridor_rects(zone, all_zones):
    """
    Returns a list of (x0, y0, w, h) rectangles for all sweep positions
    of a corridor or sub_zone zone.

    corridor  → x_start_options × y_options  (centre of strip)
    sub_zone  → inherits parent y_options, adds offset_options along X
    """
    length = float(getattr(zone, "length", 6.0))
    width  = float(getattr(zone, "width",  1.0))

    if zone.type == "corridor":
        x_starts = getattr(zone, "x_start_options", [0.0]) or [0.0]
        y_cents  = getattr(zone, "y_options",        [1.4]) or [1.4]
        rects = []
        for xs in x_starts:
            for yc in y_cents:
                rects.append((xs, yc - width / 2, length, width))
        return rects

    if zone.type == "sub_zone":
        # Find parent corridor
        parent = next((z for z in all_zones if z.id == getattr(zone, "contained_in", None)), None)
        if parent is None:
            return []
        p_x_starts = getattr(parent, "x_start_options", [0.0]) or [0.0]
        p_y_cents  = getattr(parent, "y_options",        [1.4]) or [1.4]
        p_width    = float(getattr(parent, "width",  1.0))
        offsets    = getattr(zone,   "offset_options",   [0.0]) or [0.0]
        rects = []
        for px in p_x_starts:
            for yc in p_y_cents:
                for off in offsets:
                    rects.append((px + off, yc - p_width / 2, length, p_width))
        return rects

    return []


def _point_circles(zone, all_zones):
    """
    Returns list of (cx, cy, radius) for all sweep positions of a point zone.
    Inherits parent sub_zone/corridor positions if contained_in is set.
    """
    radius = float(getattr(zone, "radius", 0.5))

    parent_id = getattr(zone, "contained_in", None)
    if parent_id is None:
        cx = getattr(zone, "x", None)
        cy = getattr(zone, "y", None)
        if cx is not None and cy is not None:
            return [(cx, cy, radius)]
        return []

    parent = next((z for z in all_zones if z.id == parent_id), None)
    if parent is None:
        return []

    # Use sub_zone rects to derive centre points
    sub_rects = _corridor_rects(parent, all_zones)
    circles = []
    for (x0, y0, w, h) in sub_rects:
        circles.append((x0 + w / 2, y0 + h / 2, radius))
    return circles


def draw_room(cfg, save_path=None, show=True):
    """
    Draws a top-down view of the room defined in cfg.
    """
    corners = cfg.ROOM_CORNERS
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    room_w = max(xs) - min(xs)
    room_h_m = max(ys) - min(ys)

    # Figure size: proportional to room, at least 8 inches wide
    scale = max(8.0 / room_w, 6.0 / room_h_m)
    fig_w = room_w * scale
    fig_h = room_h_m * scale + 1.5
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    # ── Room outline ──────────────────────────────────────────────────────
    room_poly = plt.Polygon(
        corners + [corners[0]],
        closed=True,
        facecolor="#f5f5f5",
        edgecolor="#333333",
        linewidth=2.5,
        zorder=1,
    )
    ax.add_patch(room_poly)

    # Dimension annotations
    ax.annotate(
        f"{room_w:.1f} m",
        xy=((min(xs) + max(xs)) / 2, min(ys)),
        xytext=(0, -18), textcoords="offset points",
        ha="center", va="top", fontsize=9, color="#555555",
    )
    ax.annotate(
        f"{room_h_m:.1f} m",
        xy=(min(xs), (min(ys) + max(ys)) / 2),
        xytext=(-18, 0), textcoords="offset points",
        ha="right", va="center", fontsize=9, color="#555555", rotation=90,
    )

    # ── Capture zones ─────────────────────────────────────────────────────
    poly_zones = [z for z in cfg.capture_zones if z.type == "polygon" and z.vertices]
    # Collect priorities from ALL zone types for consistent colour mapping
    all_priorities = [z.priority for z in cfg.capture_zones]

    legend_handles = []
    seen_labels = {}

    for zone in sorted(poly_zones, key=lambda z: z.priority):
        color = _priority_color(zone.priority, all_priorities)

        all_positions = _all_translated_vertices(zone)
        has_sweep = len(all_positions) > 1

        # ── All sweep positions: very transparent ─────────────────────────
        for verts in all_positions:
            patch = MplPolygon(
                verts, closed=True,
                facecolor=color,
                edgecolor="#888888",
                linewidth=0.6,
                linestyle="--",
                alpha=0.18 if has_sweep else 0.55,
                zorder=2,
            )
            ax.add_patch(patch)

        # ── Median position: full opacity + label ─────────────────────────
        med_verts = _median_translated_vertices(zone)
        patch_med = MplPolygon(
            med_verts, closed=True,
            facecolor=color,
            edgecolor="#444444",
            linewidth=1.4,
            alpha=0.65,
            zorder=3,
        )
        ax.add_patch(patch_med)

        cx = sum(v[0] for v in med_verts) / len(med_verts)
        cy = sum(v[1] for v in med_verts) / len(med_verts)
        label_txt = f"{zone.id}\n(p={zone.priority:.1f})"
        if has_sweep:
            label_txt += "\n[median pos.]"
        ax.text(
            cx, cy, label_txt,
            ha="center", va="center",
            fontsize=7.5, fontweight="bold", color="#222222",
            zorder=6,
            path_effects=[pe.withStroke(linewidth=2, foreground="white")],
        )

        # Legend entry (deduplicate by priority)
        p_key = round(zone.priority, 2)
        if p_key not in seen_labels:
            seen_labels[p_key] = True
            legend_handles.append(
                mpatches.Patch(facecolor=color, edgecolor="#444444",
                               alpha=0.8,
                               label=f"Zone priority {zone.priority:.1f}")
            )

    # ── Corridor / sub_zone zones ─────────────────────────────────────────
    rect_zones = [z for z in cfg.capture_zones if z.type in ("corridor", "sub_zone")]
    for zone in sorted(rect_zones, key=lambda z: z.priority):
        color = _priority_color(zone.priority, all_priorities)
        rects = _corridor_rects(zone, cfg.capture_zones)
        if not rects:
            continue
        has_sweep = len(rects) > 1
        mid_idx = len(rects) // 2

        for i, (x0, y0, w, h) in enumerate(rects):
            is_median = (i == mid_idx)
            rect_patch = mpatches.Rectangle(
                (x0, y0), w, h,
                facecolor=color,
                edgecolor="#444444" if is_median else "#888888",
                linewidth=1.4 if is_median else 0.6,
                linestyle="-" if is_median else "--",
                alpha=0.65 if is_median else 0.18,
                zorder=3 if is_median else 2,
            )
            ax.add_patch(rect_patch)

            if is_median:
                cx, cy = x0 + w / 2, y0 + h / 2
                lbl = f"{zone.id}\n(p={zone.priority:.1f})"
                if has_sweep:
                    lbl += "\n[median pos.]"
                ax.text(cx, cy, lbl,
                        ha="center", va="center",
                        fontsize=7.5, fontweight="bold", color="#222222",
                        zorder=6,
                        path_effects=[pe.withStroke(linewidth=2, foreground="white")])

        p_key = round(zone.priority, 2)
        if p_key not in seen_labels:
            seen_labels[p_key] = True
            legend_handles.append(
                mpatches.Patch(facecolor=color, edgecolor="#444444",
                               alpha=0.8,
                               label=f"Zone priority {zone.priority:.1f} ({zone.type})")
            )

    # ── Point zones ───────────────────────────────────────────────────────
    point_zones = [z for z in cfg.capture_zones if z.type == "point"]
    for zone in point_zones:
        color = _priority_color(zone.priority, all_priorities)
        circles = _point_circles(zone, cfg.capture_zones)
        if not circles:
            continue
        mid_idx = len(circles) // 2

        for i, (cx, cy, r) in enumerate(circles):
            is_median = (i == mid_idx)
            circle = plt.Circle(
                (cx, cy), r,
                facecolor=color,
                edgecolor="#444444" if is_median else "#888888",
                linewidth=1.4 if is_median else 0.6,
                linestyle="-" if is_median else "--",
                alpha=0.65 if is_median else 0.18,
                zorder=3 if is_median else 2,
            )
            ax.add_patch(circle)
            if is_median:
                lbl = f"{zone.id}\n(p={zone.priority:.1f})"
                ax.text(cx, cy, lbl,
                        ha="center", va="center",
                        fontsize=7.5, fontweight="bold", color="#222222",
                        zorder=6,
                        path_effects=[pe.withStroke(linewidth=2, foreground="white")])

        p_key = round(zone.priority, 2)
        if p_key not in seen_labels:
            seen_labels[p_key] = True
            legend_handles.append(
                mpatches.Patch(facecolor=color, edgecolor="#444444",
                               alpha=0.8,
                               label=f"Zone priority {zone.priority:.1f} (point)")
            )

    # ── Obstacles ─────────────────────────────────────────────────────────
    for obs in cfg.obstacles:
        h = obs.get("height", 0)
        verts_obs = obs.get("vertices", [])
        if not verts_obs:
            continue

        is_full = h >= cfg.ROOM_HEIGHT
        can_mount = obs.get("can_mount_camera", False)
        face_c     = "#888888" if is_full else "#bbbbbb"
        edge_c     = "#1a6bbd" if can_mount else "#333333"
        edge_w     = 2.5      if can_mount else 1.5
        hatch      = None     if is_full   else "//"

        patch_obs = MplPolygon(
            verts_obs, closed=True,
            facecolor=face_c,
            edgecolor=edge_c,
            linewidth=edge_w,
            hatch=hatch,
            alpha=0.85,
            zorder=4,
        )
        ax.add_patch(patch_obs)

        cx_o = sum(v[0] for v in verts_obs) / len(verts_obs)
        cy_o = sum(v[1] for v in verts_obs) / len(verts_obs)
        lbl = obs.get("label", "")
        suffix = " [cam]" if can_mount else ""
        if lbl:
            ax.text(
                cx_o, cy_o, lbl + suffix,
                ha="center", va="center", fontsize=7,
                color="white" if is_full else "#333333",
                fontweight="bold", zorder=7,
                path_effects=[pe.withStroke(linewidth=2,
                              foreground="#555555" if is_full else "white")],
            )

    # Legend for obstacles
    legend_handles.append(
        mpatches.Patch(facecolor="#888888", edgecolor="#333333",
                       label="Obstacle — full height")
    )
    legend_handles.append(
        mpatches.Patch(facecolor="#bbbbbb", edgecolor="#333333",
                       hatch="//", label="Obstacle — partial height")
    )
    legend_handles.append(
        mpatches.Patch(facecolor="#888888", edgecolor="#1a6bbd",
                       linewidth=2.0,
                       label="Obstacle — camera mountable [cam]")
    )
    if any(len(_all_translated_vertices(z)) > 1
           for z in poly_zones):
        legend_handles.append(
            mpatches.Patch(facecolor="#cccccc", edgecolor="#888888",
                           alpha=0.4, linestyle="--",
                           label="Zone sweep positions (all offsets)")
        )

    # ── Grid ──────────────────────────────────────────────────────────────
    x_ticks = np.arange(math.floor(min(xs)), math.ceil(max(xs)) + 1, 1.0)
    y_ticks = np.arange(math.floor(min(ys)), math.ceil(max(ys)) + 1, 1.0)
    ax.set_xticks(x_ticks)
    ax.set_yticks(y_ticks)
    ax.grid(color="#cccccc", linewidth=0.5, linestyle="--", zorder=0)
    ax.set_axisbelow(True)

    # ── Title and labels ──────────────────────────────────────────────────
    ax.set_xlabel("X (m)", fontsize=10)
    ax.set_ylabel("Y (m)", fontsize=10)
    config_name = os.path.basename(getattr(cfg, "_yaml_path", "config"))
    ax.set_title(
        f"Room preview — {config_name}\n"
        f"Room {room_w:.1f} m × {room_h_m:.1f} m  |  Height {cfg.ROOM_HEIGHT:.1f} m",
        fontsize=11, fontweight="bold",
    )

    margin = max(room_w, room_h_m) * 0.06
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)
    ax.set_aspect("equal")

    # ── Legend ────────────────────────────────────────────────────────────
    if legend_handles:
        ax.legend(
            handles=legend_handles,
            loc="upper left",
            bbox_to_anchor=(1.01, 1.0),
            fontsize=8,
            framealpha=0.9,
            borderpad=0.8,
        )

    # ── Compass ───────────────────────────────────────────────────────────
    ax.annotate(
        "N", xy=(max(xs) - margin * 0.3, max(ys) - margin * 0.3),
        fontsize=13, fontweight="bold", color="#333333",
        ha="center", va="center",
    )

    plt.tight_layout()

    if save_path:
        out_dir = os.path.dirname(os.path.abspath(save_path))
        os.makedirs(out_dir, exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches="tight", facecolor="white")
        print(f"Saved: {os.path.abspath(save_path)}")

    if show:
        plt.show()
    else:
        plt.close()


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Top-down preview of a lab config room layout."
    )
    parser.add_argument("--config", "-c", required=True,
                        help="Path to the YAML config file")
    parser.add_argument("--save", "-s", default=None,
                        help="Save path for the PNG (optional)")
    parser.add_argument("--no-show", action="store_true",
                        help="Do not open the interactive window")
    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg._yaml_path = args.config

    # Default save path: outputs/preview_room/<config_name>.png
    save_path = args.save
    if save_path is None:
        base = os.path.splitext(os.path.basename(args.config))[0]
        save_path = os.path.join("outputs", "preview_room", f"{base}.png")

    draw_room(cfg, save_path=save_path, show=not args.no_show)


if __name__ == "__main__":
    main()

