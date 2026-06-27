"""
consensus.py
Consensus / robustness analysis across the top-K scoring configurations.

When several configurations score nearly the same but place cameras very
differently, the objective is flat there and the single best config is not
obviously the right choice. Aggregating the top-K answers:

  - Is it really a tie?            -> score spread across the top-K.
  - Which camera POSITIONS are robust (used by almost every good config) vs
    flexible?                       -> per-camera position agreement.
  - Which DIRECTION should each camera aim, and do the good configs agree on
    that aim?                       -> per-camera aim angle + angular spread.
  - Which single config is the most representative?  -> the medoid.

Placements are compared per role (wall vs tripod) via optimal assignment, so a
wall camera is never matched against a tripod.
"""

import math
import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
    _HAVE_SCIPY = True
except Exception:                                   # pragma: no cover
    _HAVE_SCIPY = False


UNMATCHED_PENALTY = 5.0   # metres-equivalent cost per unmatched camera


# ─────────────────────────────────────────────────────────────────────────────
# Circular statistics (for aim-angle agreement)
# ─────────────────────────────────────────────────────────────────────────────

def _circ_mean(angles_deg):
    s = sum(math.sin(math.radians(a)) for a in angles_deg)
    c = sum(math.cos(math.radians(a)) for a in angles_deg)
    return math.degrees(math.atan2(s, c))


def _circ_std(angles_deg):
    """Circular standard deviation in degrees (0 = all identical)."""
    n = len(angles_deg)
    if n == 0:
        return 0.0
    s = sum(math.sin(math.radians(a)) for a in angles_deg) / n
    c = sum(math.cos(math.radians(a)) for a in angles_deg) / n
    R = min(1.0, max(1e-9, math.hypot(s, c)))
    return math.degrees(math.sqrt(-2.0 * math.log(R)))


# ─────────────────────────────────────────────────────────────────────────────
# Placement distance & medoid
# ─────────────────────────────────────────────────────────────────────────────

def _role_cams(entry, role):
    cams = entry["cam_A"] if role == 'A' else entry["cam_B"]
    return [(c[0], c[1]) for c in cams]


def _assign_cost(p, q):
    if not p and not q:
        return 0.0, 0
    if not p or not q:
        n = max(len(p), len(q))
        return UNMATCHED_PENALTY * n, n
    C = np.zeros((len(p), len(q)))
    for i, (px, py) in enumerate(p):
        for j, (qx, qy) in enumerate(q):
            C[i, j] = math.hypot(px - qx, py - qy)
    if _HAVE_SCIPY:
        ri, cj = linear_sum_assignment(C)
        matched = float(sum(C[i, j] for i, j in zip(ri, cj)))
        k = len(ri)
    else:                                           # greedy fallback
        used, matched, k = set(), 0.0, 0
        for i in range(len(p)):
            best_j, best_d = None, 1e18
            for j in range(len(q)):
                if j not in used and C[i, j] < best_d:
                    best_d, best_j = C[i, j], j
            if best_j is not None:
                used.add(best_j); matched += best_d; k += 1
    nm = abs(len(p) - len(q))
    return matched + UNMATCHED_PENALTY * nm, k + nm


def placement_distance(entry1, entry2):
    """Mean per-camera displacement between two configs (metres), per-role assignment."""
    total, n = 0.0, 0
    for role in ('A', 'B'):
        c, k = _assign_cost(_role_cams(entry1, role), _role_cams(entry2, role))
        total += c
        n += k
    return total / n if n else 0.0


def medoid_index(entries):
    """Index of the config minimising summed placement distance to all others."""
    m = len(entries)
    if m <= 1:
        return 0
    D = np.zeros((m, m))
    for i in range(m):
        for j in range(i + 1, m):
            d = placement_distance(entries[i], entries[j])
            D[i, j] = D[j, i] = d
    return int(np.argmin(D.sum(axis=1)))


# ─────────────────────────────────────────────────────────────────────────────
# Per-camera consensus (position + aim direction)
# ─────────────────────────────────────────────────────────────────────────────

def camera_consensus(entries, ref_idx, radius=0.8):
    """
    For each camera of the reference (medoid) config, measure how the top-K agree:
      pos_frac     : fraction of configs with a same-role camera within `radius`
      angle        : the medoid camera's aim (absolute degrees, 0=East)
      angle_std    : circular std (deg) of the matching cameras' aims (0 = agree)
      orient_agree : fraction of matches sharing the medoid's L/P orientation
      orient/height: medoid camera's sensor orientation and mount height
    Returns a list of dicts (wall cameras then tripod cameras).
    """
    ref = entries[ref_idx]
    out = []
    for role in ('A', 'B'):
        ref_cams = ref["cam_A"] if role == 'A' else ref["cam_B"]
        for c in ref_cams:
            cx, cy, cang, corient, ch = c[0], c[1], c[2], c[3], c[4]
            hits, angles, orients = 0, [], []
            for e in entries:
                cams = e["cam_A"] if role == 'A' else e["cam_B"]
                near = [o for o in cams if math.hypot(cx - o[0], cy - o[1]) <= radius]
                if near:
                    hits += 1
                    best = min(near, key=lambda o: math.hypot(cx - o[0], cy - o[1]))
                    angles.append(best[2])
                    orients.append(best[3])
            out.append({
                "x": cx, "y": cy, "role": role,
                "angle": cang, "orient": corient, "height": ch,
                "pos_frac": hits / len(entries),
                "angle_std": _circ_std(angles) if angles else 0.0,
                "orient_agree": (orients.count(corient) / len(orients)) if orients else 1.0,
            })
    return out


def summarise(entries, radius=0.8):
    """Full consensus summary over the (top-K, score-sorted) entries."""
    scores = [e["score"] for e in entries]
    med = medoid_index(entries)
    cams = camera_consensus(entries, med, radius=radius)
    stable = [c for c in cams if c["pos_frac"] >= 0.7]
    return {
        "n": len(entries),
        "score_max": max(scores),
        "score_min": min(scores),
        "score_median": float(np.median(scores)),
        "score_spread_pct": (max(scores) - min(scores)) / max(scores) * 100
                            if max(scores) else 0.0,
        "medoid_index": med,
        "medoid_score": entries[med]["score"],
        "cameras": cams,
        "n_stable": len(stable),
        "n_cams": len(cams),
        "radius": radius,
    }


# ─────────────────────────────────────────────────────────────────────────────
# Plot
# ─────────────────────────────────────────────────────────────────────────────

def plot_consensus(entries, summary, cfg, save_path, walk_y=None, human_height=1.9):
    """
    Consensus figure: density of camera positions across the top-K, with the
    medoid config overlaid. Each camera shows:
      - a colour: green if its POSITION is stable (>=70% agree), else orange;
      - an ARROW in its principal aim direction;
      - a label with sensor orientation (P/L), mount height, and the downward
        tilt in degrees (computed toward the capture axis walk_y).
    """
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    corners = cfg.ROOM_CORNERS
    fig, ax = plt.subplots(figsize=(17, 9))

    loop = corners + [corners[0]]
    ax.plot([c[0] for c in loop], [c[1] for c in loop], 'k-', lw=2, zorder=2)

    xs = [c[0] for e in entries for c in (e["cam_A"] + e["cam_B"])]
    ys = [c[1] for e in entries for c in (e["cam_A"] + e["cam_B"])]
    if xs:
        ax.hexbin(xs, ys, gridsize=30, cmap="YlOrRd", mincnt=1, alpha=0.55, zorder=1)

    arrow_len = 1.4
    for cam in summary["cameras"]:
        cx, cy = cam["x"], cam["y"]
        stable = cam["pos_frac"] >= 0.7
        col = "#2ca02c" if stable else "#ff7f0e"

        # aim arrow (principal viewing direction)
        a = math.radians(cam["angle"])
        ax.annotate("", xy=(cx + arrow_len * math.cos(a), cy + arrow_len * math.sin(a)),
                    xytext=(cx, cy),
                    arrowprops=dict(arrowstyle="-|>", color=col, lw=2.4,
                                    shrinkA=0, shrinkB=0), zorder=6)

        # camera marker
        ax.plot(cx, cy, 'o', color=col, ms=15, markeredgecolor='black',
                markeredgewidth=1.5, zorder=7)

        # downward tilt toward the capture axis
        if walk_y is not None:
            d_perp = max(abs(cy - walk_y), 0.3)
            tilt = abs(math.degrees(math.atan2(human_height / 2.0 - cam["height"], d_perp)))
        else:
            tilt = 0.0

        orient_lbl = "Portrait" if cam["orient"] == "P" else "Landscape"
        label = (f"{orient_lbl}\n{cam['height']:.1f}m  ↓{tilt:.0f}°\n"
                 f"pos {cam['pos_frac']*100:.0f}%  aim±{cam['angle_std']:.0f}°")
        # offset label away from the room centre-ish to reduce overlap
        ax.text(cx, cy - 0.45, label, ha='center', va='top', fontsize=8.5,
                color='black', fontweight='bold', zorder=8,
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                          edgecolor=col, alpha=0.85))

    ax.set_aspect('equal')
    xs_r = [c[0] for c in corners]; ys_r = [c[1] for c in corners]
    ax.set_xlim(min(xs_r) - 0.8, max(xs_r) + 0.8)
    ax.set_ylim(min(ys_r) - 1.2, max(ys_r) + 0.8)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")

    s = summary
    ax.set_title(
        f"Consensus over top-{s['n']} configs  |  "
        f"score {s['score_min']:.1f}-{s['score_max']:.1f} "
        f"(spread {s['score_spread_pct']:.1f}%)  |  medoid={s['medoid_score']:.1f}  |  "
        f"{s['n_stable']}/{s['n_cams']} positions stable (green)  |  "
        f"arrow = aim, ↓ = tilt down",
        fontsize=12, fontweight='bold')

    if os.path.dirname(save_path):
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
    plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
