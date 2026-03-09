# Changelog

All notable changes to this project will be documented here.  
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

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
- `pyproject.toml` for `pip install .` support
- GUI / web interface (planned)

