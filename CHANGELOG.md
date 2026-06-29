# Changelog

All notable changes to this project will be documented here.  
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [1.1.2] — 2026-06-27 — Correct displayed tilt; document consensus

### Fixed
- **Downward tilt** shown for each camera (in `print_results`, the optimisation
  log, the consensus report/plot) was computed from the *perpendicular* distance
  to the capture axis, which **overestimated** it for diagonally-aimed cameras
  (e.g. a wall camera reported at 57° was really ~8°). It now uses
  `cam_fixed_tilt` — the same aim-direction tilt the scorer actually assumes.
  The optimisation itself was unaffected (it always used `cam_fixed_tilt`); only
  the displayed numbers were wrong.
- The coverage visualisations (`count_cameras_3d` heatmap, XZ side view, coverage
  bar chart) used the same perpendicular approximation and are now aligned to
  `cam_fixed_tilt`, so the displayed coverage matches the score.

### Docs
- README and ALGORITHM.md now document the consensus / robustness analysis
  (`consensus_topk`).

---

## [1.1.1] — 2026-06-27 — Consensus analysis, STS fine search, plot fix

### Added
- **Consensus / robustness analysis** (`core/consensus.py`, opt-in via
  `optimization.consensus_topk`). Keeps the top-K scoring configs of the
  **winning zone** and reports:
  - the score spread (is the result really a tie?),
  - per-camera **position stability** (fraction of good configs that agree) and
    **aim-angle agreement** (circular spread of the viewing direction),
  - the **medoid** (most representative) configuration.
  Outputs `CONSENSUS.png` — camera-position density + the medoid config with an
  **aim arrow**, sensor orientation (Portrait/Landscape), mount height and
  **downward tilt** per camera — plus `consensus_topk.json` for offline
  re-analysis without re-running.
- **Auto-optimised STS location** restored as a post-placement fine search over
  the analysis zone, based on the full chosen configuration
  (`scoring.find_best_sts_position`).

### Changed
- The consensus runs on the configs of the best zone only, so agreement
  reflects placement uncertainty at a fixed zone rather than differences between
  zone positions.

### Fixed
- Bottom-right "Quality Score" axis is no longer hard-capped at 2.2 — it
  auto-scales to the data, so the curve is no longer clipped.

---

## [1.1.0] — 2026-06-24 — Unified placement engine, camera budget, performance

### Added
- **Unified placement engine.** Tripod cameras now go through the *same*
  zone-wide optimisation as wall cameras (farthest-point diverse init → 1-opt
  with a coverage-ratio penalty → angle reorientation). The old tripod path,
  which placed cameras by gating them on a single STS point, is gone — tripods
  no longer cluster locally and are spread across the capture zone.
- **Camera budget / free allocation.** New per-set `count_min` / `count_max`
  (back-compatible alias: `max_count`) plus an optional
  `optimization.total_cameras`:
  - no `total_cameras` → **fixed mode**: each set places exactly its `count_max`
    (legacy behaviour; covers wall-only, tripod-only, and "exactly N wall + M
    tripod").
  - `total_cameras: N` → **free mode**: the optimiser allocates `N` cameras
    across the sets within their `[count_min, count_max]` bounds (marginal greedy
    over the union of candidate pools + combined 1-opt with type-changing swaps).
- **Per-set `score_factor`** (default `1.0`): replaces the hard-wired ×0.5
  tripod down-weighting so free allocation is not biased toward wall cameras.
- **Auto-optimised STS location** is now a post-placement fine search over the
  analysis zone, based on the full chosen configuration.

### Changed
- **STS is fully optional.** With no `point` capture zone, no STS ring is added
  to the sample grid and no STS marker is drawn — tripod-only / zone-wide configs
  are no longer polluted by a phantom high-weight point.
- Tripod candidates now aim at the capture zone (weighted sample-point centroid)
  instead of a single STS point; `reorient` extended to tripods.

### Performance
- **Precomputed coverage table** (`scoring.precompute_coverage` /
  `score_indexed`): per-candidate per-point contributions are computed once per
  combo and the whole search runs on candidate indices via table aggregation,
  instead of recomputing cones and line-of-sight on every score evaluation.
  Verified to match the geometry path exactly (equivalence test).
- `cam_d_min` memoised; search scores on a subsample of evaluation points
  (the decisive per-restart score still uses the full set).
- Net effect on representative cases: ~10-15× faster (a free-mode case went from
  a >180 s timeout to ~13 s).

### Fixed
- **Partial-height obstacle occlusion** (`room.has_line_of_sight`): now tests the
  ray's *minimum* height across the whole footprint crossing (both ends) instead
  of just the midpoint, so a ray that clears an obstacle mid-span but dips below
  it at an edge is correctly reported as blocked.
- `early_stop` now actually triggers (the no-improvement counter was never
  incremented).
- Logger no longer crashes on Windows consoles that can't encode unicode.

### Tests
- Added `tests/` suite (config loader, room geometry/line-of-sight, candidates,
  scoring) including a `score_indexed` ↔ `score_configuration` equivalence test
  and partial-occlusion regression tests.

---

## [1.0.3] — 2026-03-09 — Fix configs bundling in pip wheel

### Fixed
- Configs were missing from the installed wheel (1.0.2).
  They now live in `lab_camera_optimizer/configs/` — a proper Python package
  directory that setuptools reliably includes in the wheel.
- `init_project.py` now finds bundled configs via `importlib.util.find_spec`
  regardless of the install method.

---

## [1.0.2] — 2026-03-09 — pip install improvements

### Added
- `lab-camera-init` command: copies example configs and creates `outputs/`
  folder in the current directory after `pip install`
- `init_project.py` entry point bundled in the package

---

## [1.0.1] — 2026-03-09 — PyPI fix

### Fixed
- `pyproject.toml`: updated deprecated `license` table format to SPDX string
- `pyproject.toml`: removed deprecated license classifier

---

## [1.0.0] — 2026-03-09 — Initial public release

### Added
- **Core optimiser** (`optimize.py`) — greedy + greedy_1opt algorithms with
  random restarts, early stopping and bilateral coverage constraint
- **Room preview** (`preview_room.py`) — standalone top-down visualisation of
  room layout, obstacles and capture zones before running optimisation
- **YAML configuration** — fully config-driven, no code editing required
- **Room geometry** — arbitrary polygon floor plans (L, T, U, rectangular…)
- **Obstacles** — full-height (line-of-sight blocking) and partial-height,
  with optional camera mounting support
- **Camera sets** — wall-mounted and tripod cameras, landscape/portrait
  orientations, multiple height options per camera
- **Capture zones** — corridor, sub_zone, point and polygon types;
  sweep optimisation over multiple zone positions
- **Scoring** — 3D vertical body coverage (v²), bilateral factor,
  angular diversity with proximity+angle redundancy penalty,
  diminishing-returns multi-camera reward
- **Orientation logic** — wall-aperture clamping, adjacent-wall penalty,
  butt-against-wall strategy for wide-FOV cameras near corners
- **Output** — 4-panel figures (top view, heatmap, side view, bar chart),
  per-attempt graphs, final best result, full text log
- **3 example configs** — simple rectangle, real-world L-shaped lab,
  T-shaped direction-change zone
- MIT License, CITATION.cff for academic use

---

## [Unreleased]

- Unit tests
- GUI / web interface (planned)

