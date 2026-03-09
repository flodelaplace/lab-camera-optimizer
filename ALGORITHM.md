# How the optimiser works

This document explains the internal logic of **Lab Camera Optimizer** —
the scoring function, the optimisation algorithm, the combo sweep, and
how to tune every parameter.

---

## Table of contents

1. [High-level overview](#1-high-level-overview)
2. [Evaluation points — the sample grid](#2-evaluation-points--the-sample-grid)
3. [Scoring a camera configuration](#3-scoring-a-camera-configuration)
   - 3.1 [Vertical body coverage `v`](#31-vertical-body-coverage-v)
   - 3.2 [Per-camera score `score_v`](#32-per-camera-score-score_v)
   - 3.3 [Effective camera count (angular diversity)](#33-effective-camera-count-angular-diversity)
   - 3.4 [Sum with diminishing returns](#34-sum-with-diminishing-returns)
   - 3.5 [Bilateral factor](#35-bilateral-factor)
   - 3.6 [Final point score](#36-final-point-score)
4. [Candidate generation](#4-candidate-generation)
5. [Optimisation algorithm](#5-optimisation-algorithm)
   - 5.1 [Pure greedy (`greedy`)](#51-pure-greedy-greedy)
   - 5.2 [Greedy + 1-opt (`greedy_1opt`)](#52-greedy--1-opt-greedy_1opt)
   - 5.3 [Diverse initialisation](#53-diverse-initialisation)
   - 5.4 [Coverage-ratio penalty](#54-coverage-ratio-penalty)
   - 5.5 [Random restarts and early stopping](#55-random-restarts-and-early-stopping)
6. [Zone sweep — the combo grid](#6-zone-sweep--the-combo-grid)
   - 6.1 [Corridor-based zones](#61-corridor-based-zones)
   - 6.2 [Polygon zones](#62-polygon-zones)
   - 6.3 [Bilateral axis `walk_y`](#63-bilateral-axis-walk_y)
7. [Camera orientation logic](#7-camera-orientation-logic)
8. [Tuning guide — all parameters](#8-tuning-guide--all-parameters)

---

## 1. High-level overview

```
YAML config
    │
    ├─ Room geometry + obstacles
    ├─ Camera set(s) specs
    └─ Capture zone(s) definition
           │
           ▼
  ┌─────────────────────────────────────┐
  │  COMBO SWEEP                        │
  │  For every (zone position) combo:   │
  │    1. Build sample point grid       │
  │    2. Generate candidate placements │
  │    3. Run greedy optimisation       │
  │       (N restarts)                  │
  │    4. Save best config + graph      │
  └─────────────────────────────────────┘
           │
           ▼
  Global best config  →  FINAL_RESULT.png
```

The tool explores a **grid of zone positions** (the "combo sweep").
For each position of the capture zone, it runs a greedy optimisation
that places cameras one by one to maximise a coverage score.
The globally best configuration across all combos is kept.

---

## 2. Evaluation points — the sample grid

Coverage is measured at a set of **evaluation points** `(px, py, weight)`
distributed across the capture zone.

| Zone type | How points are placed | Default weight |
|---|---|---|
| `corridor` | Regular grid along the corridor axis | `0.5` |
| `sub_zone` | Denser grid inside the priority sub-zone | `1.0` |
| `point` | Concentric rings around the target point | `2.0` |
| `polygon` | Regular grid inside the polygon | `zone.priority` |

Points outside the room or inside obstacles are automatically excluded.

Higher-weight points contribute more to the total score — place higher
priorities on zones where full-body visibility is most critical.

---

## 3. Scoring a camera configuration

The score of a configuration is the **sum of per-point scores** over all
evaluation points.

### 3.1 Vertical body coverage `v`

For each camera that sees a point `(px, py)` in its horizontal cone,
the tool computes `v` — the **fraction of the body (0 → subject_height)**
that falls inside the camera's vertical FOV:

```
v = visible body height / total body height    ∈ [0, 1]
```

If `v < vertical_coverage_threshold` (default `0.9`), the camera is
considered to **not usefully cover** that point and is ignored.

This threshold is the most important quality gate — a camera that only
sees the legs or only the upper body does not count.

### 3.2 Per-camera score `score_v`

```
score_v = v² × dist_quality(d) × camera_weight

dist_quality(d) = 1 / (1 + k × d²)
```

- `v²` penalises partial coverage quadratically — 90% coverage gives
  `0.81`, while 100% gives `1.0`.
- `dist_quality` slightly reduces the score for far cameras (tunable
  via `distance_quality_factor k`). With the default `k = 0.001` the
  effect is small: a camera at 8 m gives `dist_quality ≈ 0.94`.
- `camera_weight` (`score_weight` in the YAML) lets you weight one
  camera set more than another (e.g. high-resolution vs wide-angle).

### 3.3 Effective camera count (angular diversity)

Simply counting cameras that see a point would reward stacking many
cameras at the same angle. Instead, the tool counts **angularly distinct
views**:

```
effective_n starts at 1.0 for the first camera, then for each next camera:

  angular difference < 20°              → +0.30  (near-duplicate view)
  close in position (<2m) AND same dir  → +0.30 to +0.95  (partial redundancy)
  otherwise                             → +1.00  (genuinely new viewpoint)
```

Two cameras close to each other AND pointing in the same direction are
nearly redundant (they see the same occlusions, same angle). Two cameras
close to each other but pointing in *opposite* directions provide two
genuinely different views — they both get full credit.

### 3.4 Sum with diminishing returns

Instead of averaging scores (`avg_v`), the tool sums them with a
**square-root saturation**:

```
sorted_scores = [s1 ≥ s2 ≥ s3 ≥ …]
sum_v = s1/√1 + s2/√2 + s3/√3 + …

sum_v_norm = sum_v / √target_coverage
base = min(effective_n, target_coverage) / target_coverage × sum_v_norm
```

This means:
- 1 perfect camera → `base ≈ 1/target`
- 3 cameras → meaningfully better than 1
- 10 cameras on the same point → not 10× better than 3

### 3.5 Bilateral factor

For biomechanics, it is essential to have cameras on **both sides** of
the subject. The bilateral factor rewards this:

```
south_v = Σ score_i/√i  for cameras south of walk_y   (saturated sum)
north_v = Σ score_i/√i  for cameras north of walk_y

bilateral_pt = √(south_v × north_v) / max(south_v, north_v)
```

`bilateral_pt` = 1.0 if both sides are equally covered, → 0 if only
one side covers the point.

Unlike the previous `max()` per side, this **sum with saturation**
rewards having multiple cameras on each side, not just one good one.

### 3.6 Final point score

```
point_score = weight × [
    (1 - bilateral_weight) × (base + angular_bonus)
  + bilateral_weight       × (base + angular_bonus) × bilateral_pt
]
```

- `bilateral_weight = 0.0` → pure coverage, no bilateral constraint
- `bilateral_weight = 1.0` → score = 0 if any point is not covered
  from both sides
- Recommended: `0.7 – 0.85` for biomechanics labs

`angular_bonus` adds up to +0.30 if cameras viewing a point are
well-spread angularly (gaps > 60° between consecutive view angles,
maximum gap < 120°).

**Total score** = Σ `point_score` over all evaluation points.

---

## 4. Candidate generation

Before optimising, the tool generates all **valid camera placements**
it will consider.

For **wall-mounted cameras**, a candidate is a tuple
`(x, y, angle, orientation, height)`:

- `(x, y)` : sampled every `wall_step` metres along every wall segment
  and every obstacle wall marked `can_mount_camera: true`
- `angle`  : the best angle for this position (see
  [Camera orientation logic](#7-camera-orientation-logic))
- `orientation` : `"L"` (landscape) or `"P"` (portrait)
- `height` : one of `height_options`

The number of candidates is typically `wall_perimeter / wall_step × 2 orientations × N heights`.
For a 13 m × 4.7 m lab with `wall_step = 0.35` and 2 heights: ~500 candidates.

For **tripod cameras**, candidates are placed on a regular grid
(`tripod_grid_step`) over the floor, keeping only positions with
line-of-sight to the target point.

---

## 5. Optimisation algorithm

### 5.1 Pure greedy (`greedy`)

The simplest algorithm. Cameras are added **one at a time**:

```
for i in 1 → max_count:
    pick the candidate that maximises score(current_cameras + candidate)
    add it to the configuration
```

Fast but can get stuck in local optima — a bad first choice propagates.

### 5.2 Greedy + 1-opt (`greedy_1opt`) — recommended

A two-phase algorithm:

**Phase 1 — Diverse initialisation** (see §5.3)
Place all cameras using farthest-point sampling to ensure good spatial
spread before any score-based refinement.

**Phase 2 — 1-opt local search**
For each camera in the configuration (in random order):
```
for each candidate in pool_1opt (80 random candidates):
    compute score(other_cameras + candidate)
    × coverage_ratio(candidate)   ← penalise tiny footprint
    if score improves → replace
repeat until no pass improves (max 2 passes)
```

This is repeated for each restart with a **different random pool**,
so different restarts explore genuinely different parts of the search
space rather than re-running the same 1-opt on the same candidates.

### 5.3 Diverse initialisation

Instead of placing the first camera greedily (which biases toward
wherever the zone is densest), `greedy_1opt` uses **farthest-point
sampling**:

1. Pick one random visible candidate as the first camera.
2. Each subsequent camera is the candidate **farthest from all already-placed
   cameras** that still satisfies `min_spacing`.

This guarantees cameras are spread around the room before score
refinement, preventing all cameras from clustering on one side.

### 5.4 Coverage-ratio penalty

When evaluating a candidate during greedy/1-opt, its score is multiplied
by the **fraction of sample points it actually sees**:

```
adjusted_score = score_configuration(...) × coverage_ratio(candidate)
coverage_ratio = points_seen / total_sample_points
```

This prevents a camera very close to a zone from getting a high score
even though it only covers a tiny slice of it — closeness boosts
`dist_quality` but `coverage_ratio` brings the score back down.

### 5.5 Random restarts and early stopping

The full greedy process is repeated `restarts_per_combo` times, each
time with a different random pool of candidates. The best result across
all restarts is kept.

**Early stopping**: if no improvement is found after `early_stop`
consecutive restarts, the combo is terminated early.

```
early_stop: 5   # stop after 5 restarts with no improvement
                # 0 = auto (n_restarts // 3)
```

---

## 6. Zone sweep — the combo grid

The **combo sweep** is what makes the tool explore not just camera
positions but also the optimal **zone placement** in the room.

### 6.1 Corridor-based zones

For `corridor`/`sub_zone` configs, three parameters sweep independently:

| Parameter | YAML key | What it varies |
|---|---|---|
| Corridor X start | `x_start_options` | Where the walking path begins along X |
| Zone Y position | `y_options` | Transverse position of the corridor in the room |
| Sub-zone offset | `offset_options` | How far into the corridor the analysis zone starts (run-up distance) |

Total combos = `|x_start_options| × |y_options| × |offset_options|`

Example: `[1.0, 2.0] × [1.0, 1.4, 1.8] × [1.0, 2.0, 3.0]` = **18 combos**.

Each combo is a separate optimisation run. The globally best result
across all 18 is the final answer.

### 6.2 Polygon zones

For `polygon` zones (L, T, cross, etc.), each zone has:

```yaml
placement:
  x_offsets: [0.0, 0.5, 1.0]   # translate the whole polygon along X
  y_offsets: [2.5, 3.0]         # translate the whole polygon along Y
```

Zones that share the **same** `x_offsets` and `y_offsets` lists are
**grouped** and move together (e.g. all parts of a T-shape).
Zones with different offset grids vary independently (cartesian product).

Total combos = `|x_offsets × y_offsets|` for each group, multiplied
across groups.

**Why sweep zone positions?**
A researcher may not know exactly where the subject will walk in the
room. The sweep finds the zone position that simultaneously maximises
camera coverage — i.e. where to place the walking path to be best seen.

### 6.3 Bilateral axis `walk_y`

The bilateral constraint requires knowing where "south" and "north" are.
`walk_y` is the Y coordinate of the capture axis:

- **Corridor mode**: `walk_y` = the Y centre of the corridor for this combo
- **Polygon mode**: `walk_y` = centroid Y of the **highest-priority** polygon
  zone (e.g. the intersection of a T-shape)

This means the bilateral axis correctly tracks the zone as it moves
across combos.

---

## 7. Camera orientation logic

Each wall-mounted camera's angle is not swept exhaustively — it is
**computed analytically** per candidate position to point optimally
toward the capture zone.

**Step 1 — Wall aperture**
`wall_angular_limits()` computes the angular range `[lim_left, lim_right]`
within which the camera can point without entering its own wall.
For a corner, this is the diagonal bisector of the two walls.

**Step 2 — Candidate angles**
The tool sweeps offsets within the aperture (every 5°). For each angle:
```
q = aim_quality(angle, zone_targets) × adjacent_wall_penalty(angle)
```
- `aim_quality`: fraction of the zone's sample points inside the cone
- `adjacent_wall_penalty`: penalises cones pointing toward a nearby
  adjacent wall, weighted by `1 / distance_to_wall`

**Step 3 — Wide-FOV handling (butt-against-wall)**
When the camera's horizontal FOV is wider than the available aperture
(common in corners with wide-angle cameras), some overflow is unavoidable.
The tool tests two "butt positions":
- Push the cone against the **left** edge of the aperture
- Push the cone against the **right** edge of the aperture

The position with the best `aim_quality × adjacent_wall_penalty` is kept.
This ensures overflow always goes toward the **far adjacent wall**
rather than splitting equally between both.

---

## 8. Tuning guide — all parameters

### Room and subject

| YAML key | Effect |
|---|---|
| `room.corners` | Defines the floor plan polygon. Any convex or concave shape. |
| `room.height` | Used to determine floor-to-ceiling obstacles and camera mount height limits. |
| `subject.height` | The full body height used for vertical coverage calculation. |
| `subject.foot_z` | Height of feet above floor (almost always 0). |

### Camera sets

| YAML key | Effect |
|---|---|
| `fov_h_landscape / fov_v_landscape` | Horizontal and vertical FOV in landscape orientation. |
| `fov_h_portrait / fov_v_portrait` | FOV in portrait orientation (camera rotated 90°). |
| `height_options` | List of mounting heights to test. Each height is a separate candidate. More values = more candidates = slower. |
| `max_count` | Maximum cameras in the final solution. |
| `min_spacing` | Minimum distance between two cameras of the same set (metres). Prevents trivial clustering. |
| `max_range` | Maximum useful range of the camera. Points beyond this distance are not seen. |
| `min_range` | Minimum range (blind zone directly in front). |
| `score_weight` | Multiplier on `score_v` for this camera set. Use to prioritise one set over another. |

### Capture zones

| YAML key | Effect |
|---|---|
| `priority` | Weight of this zone's points in the total score. Higher = more important. |
| `grid_step` | Spacing between evaluation points (metres). Smaller = more precise but slower. |
| `placement.x_offsets / y_offsets` | List of translations to test for polygon zones. More values = more combos = slower. |
| `placement.x_start_options` | List of corridor X start positions to test. |
| `placement.y_options` | List of corridor Y centre positions to test. |
| `offset_options` | Run-up distances to test for sub_zone (metres from corridor start). |

### Optimisation

| YAML key | Default | Effect |
|---|---|---|
| `target_coverage` | `4` | Target number of cameras per point. Score saturates above this. Typically 3–6 for biomechanics. |
| `bilateral_weight` | `0.8` | Weight of the bilateral constraint. `0` = disabled, `1` = fully enforced. |
| `vertical_coverage_threshold` | `0.9` | Minimum fraction of body height a camera must see to count. `0.9` = must see at least 90% of the body. |
| `algo` | `greedy_1opt` | `greedy` = fast, lower quality. `greedy_1opt` = slower, better. |
| `restarts_per_combo` | `15` | Number of random restarts per combo. More = better quality, longer runtime. |
| `early_stop` | `5` | Stop a combo after this many consecutive restarts with no improvement. `0` = run all restarts. |
| `wall_step` | `0.35` | Spacing between candidate positions along walls (metres). Smaller = more candidates = slower. |
| `angle_steps` | `24` | (Legacy parameter, now computed analytically.) |
| `tripod_grid_step` | `0.70` | Grid spacing for tripod camera candidates (metres). |
| `distance_quality_factor` | `0.001` | Controls how much distance penalises the score. `0` = no penalty. `0.01` = noticeable penalty at 10 m. |
| `graph_mode` | `best_per_combo` | `all` = save a graph for every attempt. `records_only` = only when a new record is set. `best_per_combo` = one graph per combo at the end. |

### Runtime vs quality trade-offs

| Goal | Recommended settings |
|---|---|
| Quick exploration | `algo: greedy`, `restarts_per_combo: 5`, `wall_step: 0.5`, fewer `x_offsets/y_offsets` |
| Best quality | `algo: greedy_1opt`, `restarts_per_combo: 20`, `wall_step: 0.25`, `early_stop: 0` |
| Reduce combos | Reduce `x_offsets`, `y_offsets`, `x_start_options`, `y_options` to fewer values |
| Better bilateral | Increase `bilateral_weight` (0.85–0.95), increase `target_coverage` |
| Better corner coverage | Reduce `wall_step` near corners, increase `restarts_per_combo` |

