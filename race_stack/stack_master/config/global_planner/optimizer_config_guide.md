# Global Planner Optimizer Config — Change Guide

## Why Small Track Size Is Fundamentally a Problem

The global planner pipeline has two sequential stages that both break on small tracks:

### Stage 1 — Centerline extraction and smoothing

The raw centerline is extracted by skeletonizing the binary occupancy grid map. On a small home track (~30–50m perimeter at 0.05m/px map resolution), this produces only **~600–1000 skeleton pixels**, compared to thousands on competition tracks.

The smoothed centerline is then fed into the IQP (Iterative Quadratic Program) minimum-curvature optimizer. The IQP builds curvature matrices from point-to-point finite differences. If the smoothed centerline has **kinks, oscillations, or non-monotonic curvature**, the resulting matrices are ill-conditioned and the QP solver diverges — this is the **solver explosion**. It manifests as `RuntimeError` from the IQP, NaN values in the trajectory, or the optimizer looping forever.

**Why CCMA fails on small tracks:**

CCMA (Curvature-Corrected Moving Average) uses centripetal parametrization and a moving-average window to smooth while preserving corner curvature. It is recommended for competition tracks because it prevents inward bending at hairpins. But on a small track:

- The skeleton has few points per corner (a 0.5m-radius hairpin at 0.05m/px = only ~30 pixels around the bend)
- CCMA's window size for `n < 500` is `w_ma=5, w_cc=3` — the smallest setting, barely larger than the corner itself
- The curvature-correction term fights the smoothing term in a regime (few points, high curvature) it wasn't designed for
- Result: the "smoothed" output has oscillatory artifacts right at the corners — exactly where the IQP is most sensitive

**Why savgol works on small tracks:**

Savitzky-Golay fits a local degree-3 polynomial over a window of points (window size = 21 for `n < 500`). It has no curvature constraint — it just fits a smooth polynomial regardless of corner radius. This produces a stable, artifact-free centerline even on tight corners with few supporting points. The trade-off (noted in the code docstring) is that savgol can overshoot slightly at hairpin→straight transitions on large competition tracks — which is why CCMA is preferred there.

### Stage 2 — Trajectory optimization resolution

After centerline extraction, the IQP optimizer resamples the track to `stepsize_reg` meter intervals. On a 40m home track:

- Default `stepsize_reg=0.2m` → 200 track points → acceptable
- Default `curvlim=1.0 rad/m` → max turn radius = 1m → too gentle for a track with 0.3–0.5m hairpins → optimizer clips corners, hits track bounds, goes infeasible

So even if the centerline extraction succeeds, the optimizer still fails without the smaller stepsizes and higher curvlim.

---

## Why `centerline_smoother: 'savgol'` in global_planner_params.yaml (not CCMA)

The code default (`smooth_centerline(..., smoother='ccma')`) and the comparison plot title say `CCMA (recommended)`. Despite this, `global_planner_params.yaml` is set to `savgol`. Three reasons:

1. **`ccma` package not installed on the car** — the code silently falls back to savgol anyway if `import ccma` fails, so setting it to `ccma` without the package is a silent no-op
2. **Home track topology** — as explained above, savgol is genuinely better for the home track; CCMA would cause solver explosions
3. **Stability over accuracy** — for home testing we need reliable planning, not competition-grade corner optimization

If running on porto or a large track and the `ccma` package is installed (`pip install ccma`), change `centerline_smoother` to `'ccma'` for better corner accuracy.

---

## Background: ini Files

The global trajectory optimizer uses a vehicle params file (`.ini`) that controls trajectory resolution and vehicle constraints. The default repo ships two such files:

- `racecar_f110.ini` — F1TENTH-scale vehicle params, used by `trajectory_optimizer.py` as the active config
- `racecar.ini` — base/fallback config inside the optimizer package (`global_racetrajectory_optimization/params/`)

Both were originally tuned for **competition-size tracks** (porto, larger circuits). Running them on a **small home track** (home_v4 — tight corners, short straights) caused problems:

- Coarse stepsizes → few trajectory points → poor resolution on tight corners
- Low `curvlim` → optimizer can't generate tight-turn trajectories → cuts corners or goes infeasible

## What We Changed (Home Track Adaptation)

### Phase 1 — Direct edits to defaults (now reverted)

| File | Parameter | Default → Home Track |
|------|-----------|----------------------|
| `racecar_f110.ini` | `stepsize_reg` | `0.2` → `0.05` |
| `racecar_f110.ini` | `stepsize_interp_after_opt` | `0.1` → `0.05` |
| `racecar_f110.ini` | `curvlim` | `1.0` → `3.0` |
| `racecar.ini` | `stepsize_prep` | `1.0` → `0.1` |
| `racecar.ini` | `stepsize_reg` | `3.0` → `0.2` |
| `racecar.ini` | `stepsize_interp_after_opt` | `2.0` → `0.1` |

**Why reverted:** Editing the defaults makes it impossible to switch back cleanly for competition maps. Instead, a separate home track config was created.

### Phase 2 — Proper solution (current state)

**New file created:** `racecar_f110_home.ini`
- Copy of `racecar_f110.ini` with home track values: `stepsize_reg=0.05`, `stepsize_interp_after_opt=0.05`, `curvlim=3.0`
- `racecar_f110.ini` reverted to competition defaults

**`optimizer_config` param added** — allows selecting which `.ini` to use per deployment without touching code:

| File | Change | Reason |
|------|--------|--------|
| `global_planner_params.yaml` | Added `optimizer_config: 'racecar_f110.ini'` | ROS2 param to select ini file at launch |
| `global_planner_node.py` | Reads `optimizer_config`, passes to `GlobalPlannerLogic` | Node wires param into logic |
| `global_planner_logic.py` | `__init__` accepts `optimizer_config_`, stores as `self.optimizer_config`, passes to all 3 `trajectory_optimizer` calls | Logic passes selected ini through to optimizer |
| `trajectory_optimizer.py` | Added `veh_params_file` arg to signature, replaced hardcoded `"racecar_f110.ini"` | Optimizer uses whichever ini is passed in |

## Stepsize Parameters — What They Mean

| Parameter | Effect | Small track | Large track |
|-----------|--------|-------------|-------------|
| `stepsize_prep` | Initial interpolation before spline fit | 0.05–0.1 m | 0.5–1.0 m |
| `stepsize_reg` | Points per meter during optimization | 0.05 m = dense | 0.2–3.0 m = sparse |
| `stepsize_interp_after_opt` | Final trajectory resolution | 0.05 m | 0.1–2.0 m |
| `curvlim` | Max curvature [rad/m] the optimizer allows | 3.0 (tight turns OK) | 1.0 (gentle turns) |

Rule of thumb: `stepsize_reg` should be ~2–5% of the shortest straight on the track.

## How to Switch Configs

**For home track (home_v4):**
```yaml
# global_planner_params.yaml
optimizer_config: 'racecar_f110_home.ini'
```

**For competition/porto:**
```yaml
# global_planner_params.yaml
optimizer_config: 'racecar_f110.ini'
```

**To add a new track config** (e.g. for a new venue):
1. Copy `racecar_f110.ini` → `racecar_f110_<venue>.ini` in `stack_master/config/global_planner/`
2. Tune stepsizes and curvlim for that venue
3. Set `optimizer_config: 'racecar_f110_<venue>.ini'` in `global_planner_params.yaml`

No code changes needed.

## Files Summary

```
race_stack/stack_master/config/global_planner/
  racecar_f110.ini                  # competition defaults (do not edit)
  racecar_f110_home.ini             # home track (home_v4)
  global_planner_params.yaml        # set optimizer_config here
  optimizer_config_guide.md         # this file

race_stack/planner/global_planner/global_planner/
  global_planner_node.py            # reads optimizer_config param
  global_planner_logic.py           # stores + passes to trajectory_optimizer
  global_racetrajectory_optimization/.../
    trajectory_optimizer.py         # accepts veh_params_file arg
    params/racecar.ini              # base fallback (competition defaults, do not edit)
```
