"""
reporting.py
Functions for logging and printing results.
"""

import os
import datetime
import math
from .room import wall_normal_at

class Logger:
    def __init__(self):
        self._file = None

    def init(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True) if os.path.dirname(path) else None
        self._file = open(path, 'w', encoding='utf-8')
        self.log(f"=== LOG START  —  "
                 f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===")

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


def print_results(cam_A_list, cam_B_list, score, bilateral_scores, cfg, state):
    """Prints the final formatted results of a configuration."""
    south_s, north_s = bilateral_scores
    total_s = max(south_s + north_s, 1e-6)
    bal     = min(south_s, north_s) / max(south_s, north_s, 1e-6)

    cam_A = next(c for c in cfg.camera_sets if c.mounting == "wall")
    cam_B = next((c for c in cfg.camera_sets if c.mounting == "tripod"), None)
    walk_y = state["walk_y"]
    wall_segs = state["wall_segments"]

    LOG.log("\n" + "="*65)
    LOG.log("  CONFIGURATION RESULT")
    LOG.log("="*65)
    LOG.log(f"Score: {score:.2f}")
    LOG.log(f"Bilateral:  SOUTH={south_s:.1f} ({100*south_s/total_s:.0f}%)  "
            f"NORTH={north_s:.1f} ({100*north_s/total_s:.0f}%)  "
            f"balance={bal*100:.0f}% "
            f"{'OK' if bal > 0.6 else 'UNBALANCED'}\n")

    LOG.log(f"Wall cameras ({cam_A.name}):")
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
        LOG.log(f"\nTripod cameras ({cam_B.name}):")
        for i, (x, y, angle, orient, ih) in enumerate(cam_B_list):
            d_perp   = max(abs(y - walk_y), 0.3)
            tilt_deg = math.degrees(math.atan2(cfg.HUMAN_HEIGHT / 2.0 - ih, d_perp))
            LOG.log(f"  B{i+1} [{orient}]  pos=({x:.2f}m, {y:.2f}m)  h={ih:.1f}m  "
                    f"angle={angle:.0f}deg  tilt={abs(tilt_deg):.1f}deg")

    LOG.log("="*65)