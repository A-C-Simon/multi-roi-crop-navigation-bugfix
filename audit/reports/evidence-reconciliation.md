# Audit supplement: reconciling my findings with the reviewer's reproductions

This supplement stays within the same read-only scope. Here is the split of responsibility:

- **The coordinating reviewer ran** the reproductions described below.
- **I did not run anything.** I re-read the relevant code to check each claim and its reachability.
- **Nobody has tested** any ROS, Gazebo or physical-robot consequence.

I have not seen the earlier audit and have not tried to guess its contents.

## 1. What changed

- **A-01 and A-02 are real and new.** I traced both. I missed both in the first pass. Each is a separate root cause from my earlier findings.
- **F-04 moves from Medium to High.** The reproduction confirms it in every configuration, including the adapter that actually drives the simulated rover.
- **F-07 keeps its severity but gains a deterministic case.** With fewer than 7 configured strips, a large change can never be committed.
- **Every other severity is unchanged.** The reproductions confirmed the mechanism but did not show impact beyond what I first claimed.
- **Result:** 4 High, 5 Medium, 11 Low, 0 Critical. Section 4 explains why the count would differ under other severity definitions.

**Two kinds of evidence, kept separate:**

- *Intrinsic corruption* means the stack's numbers or commands are wrong. The reviewer reproduced this for F-01, F-02, F-03, F-04, F-07, F-09, F-10, A-01 and A-02, and for the F-05 interface gap and the F-06 clock dependence.
- *Closed-loop physical consequence* means what the rover actually does as a result. This has not been tested for any finding.

## 2. Final ranked table

| Rank | ID | Revised severity (was) | Evidence state | Justification |
| --- | --- | --- | --- | --- |
| 1 | A-01 | **High** (new); Critical if severity is judged per supported mode | Reproduced (line mode, healthy corridor). Sparse spline fallback statically reachable, not reproduced. Spline envelope path platform-dependent, requires experiment | In the supported straight-line mode, an ideal centred corridor is encoded on the wrong side of the image (X ≈ -617 px), so the command is a full-rate turn (w = 0.6) at full speed (0.2 m/s). |
| 2 | A-02 | **High** (new) | Reproduced | When updates arrive faster than every 16.7 ms, the smoother produces zero steering forever, whatever the error, in every smoothed path (all temporal and lookahead runs and the simulation). |
| 3 | F-01 | High (unchanged) | Reproduced | With lookahead on, the heading feature flips sign (+7.8 vs -8.0 degrees), so the heading term of w points the wrong way (-0.138 vs +0.161). Default for video and camera runs and for `MultiROINavigationStack`. |
| 4 | F-04 | **High** (was Medium) | Reproduced | With no crops visible, the stack reports `has_line=True` and keeps driving at about 0.10 m/s toward a made-up centred line. There is no stop path in any configuration, including the simulation. |
| 5 | F-07 | Medium (unchanged; deterministic case added) | Reproduced, filter only | A genuine large change is held for 200+ frames when fewer than 7 strips are two-sided. With `n_strips < 7` it can never be committed. |
| 6 | F-02 | Medium (unchanged) | Reproduced | The lookahead path is biased by the crop offset (+1 px vs -11 px, offset 12 px). The bias is bounded and constant. |
| 7 | F-03 | Medium (unchanged) | Reproduced | A strip-count/bin-count mismatch crashes video mode on the second frame. The trigger is a non-default flag. |
| 8 | F-06 | Medium (unchanged) | Reproduced via clock injection | The command depends on host timing (-0.061 vs -0.148 for identical inputs). This is also the main way A-02's short-interval condition gets reached by default. |
| 9 | F-05 | Medium (unchanged) | Interface reproduced; simulation consequence untested | `info` never carries `has_line`, so shuttle and fishtail rear-camera logic falls back to odometry or timers. Simulation-only, non-default turn modes. |
| 10 | F-09 | Low (unchanged) | Reproduced | Interleaved low-evidence frames and gaps do not reset the pending candidate, so a stale candidate can commit (+35 px) on one frame. Bounded, and the gap route is unreachable from the shipped pipelines (F-04). |
| 11 | F-10 | Low (unchanged) | Reproduced (2 of 8 points dropped from `keep`, spline identical) | The spline's outlier rejection does nothing. The safety envelope limits the effect. |
| 12 | F-08 | Low | Static / likely | The simulation node publishes nothing when perception fails, so the last command persists. |
| 13 | F-11 | Low | Likely | The lookahead's bottom measurement uses strip 1, which the navigation fit deliberately excludes. |
| 14 | F-13 | Low | Static | The controller's normalisation width is not synced to the actual frame size. |
| 15 | F-14 | Low | Likely | A rover reset does not reset the navigation node's state. |
| 16 | F-15 | Low | Static | Midpoint y is the strip top, not its centre: about 2 to 3 px bias on slanted corridors. |
| 17 | F-16 | Low | Static | Off-by-one strip index in `pre_corridor_clean.strip_of`. |
| 18 | F-12 | Low | Requires experiment | Bottom bins are not refreshed after a `map_switch`. |
| 19 | F-17 | Low | Requires experiment | `_stop_robot` runs a second time on a destroyed node. |
| 20 | F-18 | Low | Static | The detector CLI summary counts each scrub delta twice. Cosmetic. |

How I avoided counting a root cause twice:

- A-01 and F-04 are separate: one is how a vertical line is encoded, the other is what happens when no crops are found. They combine in one case (section 3.1) but are not counted twice.
- F-06 is about which clock is used. A-02 is about the order of operations inside the smoother. They are separate.
- A-01's effect on the spline envelope is part of A-01, not F-10.

## 3. Detailed assessments

### 3.1 A-01: Vertical line fit puts the corridor on the wrong side of the image when the slope sign is negative

**Where:** `test_multi_roi.py:916-921`.

**The defect, statically demonstrated:**

```python
if abs(dx) < 1e-9:
    w_out, b_out = 1e6 * (1.0 if dy >= 0 else -1.0), -1e6 * c[0]
```

For `y = w*x + b`, a vertical line at `x = c0` needs `b = -w*c0`. The code always uses `-1e6*c0`. When `w = -1e6`, that gives `x(y) = (y + 1e6*c0)/(-1e6) = -c0 - y*1e-6`: the line sits at about `-c0`. The non-vertical branch (`:920-921`) is correct.

**When the vertical branch is taken:**

- `_fit_line_tls` hits it only when every *accepted* point has exactly the same x value. Rejected outliers do not count, because the line is fitted on `acc` only (`:902-905`).
- Midpoints are averages of cluster centres that sit on half-pixel steps, so they land on quarter-pixel values (`:520-521`). On straight synthetic or rendered corridors, identical values across strips are common. On real field images they are less likely.
- The F-04 fallback always produces identical x values (all midpoints equal the initial guess).

**Whether the result is wrong depends on point order and platform:**

- The sign of `vt[0]` from SVD is not specified by LAPACK. On the reviewer's platform, points with y descending gave `w = -1e6` (wrong); y ascending gave `+1e6` (correct).
- Another BLAS/LAPACK build (MKL, Accelerate, a different OpenBLAS) may choose the opposite sign. That would change which code paths are affected, not whether the defect exists.
- The fix `b_out = -w_out * c[0]` is correct whichever sign SVD returns.

**Which paths reach it:**

| Path | Point order | Reachability |
| --- | --- | --- |
| Line mode: `nav_curve=False` via CLI `--line` (`run_mr_navigation.py:777`), simulation `line_fit:=true` / `run_sim.sh --line` (`pipeline.py:164-178`), library `MultiROIDetector(nav_curve=False)` | `nav_pts` in strip order, y descending (`:603-623`) | **Reproduced:** 10 two-sided strips, lateral error -617 px, v 0.2, w 0.6. |
| Spline mode with fewer than 4 accepted points (`:637`, so `nav_curve=None`), after which `mr_vs.py:252-258` uses `nav_line` | y descending | **Statically reachable, not reproduced.** Needs 2 or 3 accepted midpoints with exactly equal x, as on a sparse but straight corridor. With 5 or more points, total-least-squares stops rejecting at 4 (`:909`), so this sparse case needs `nav_pts <= 3`. |
| Line mode on a blank or covered frame (F-04 fallback, all x identical) | y descending | **Derived, not reproduced:** on the reviewer's platform this would give the same saturated turn as the healthy corridor. |
| Spline safety envelope reference fit (`:846`), where `ref_x = (yy - b_r)/w_r` | y **ascending** (`:794`) | Correct on the reviewer's platform. On a platform with the opposite SVD sign, the envelope would pull the default-mode curve to about `-c0 + env_cap`. **Requires experiment on other BLAS backends.** |
| Detection lines (`:669`), drawing, and the opt-in `--post_scrub` anchors (`isolate_forest_clean`) | y descending | Rows drawn on the wrong side. If scrubbing is enabled, the anchor distances, and therefore which blobs get protected or erased, are wrong. |

**What the temporal filter does about it:** nothing reliable. The first frame is accepted directly (`temporal_filter.py:286-301`). If the defect is present on every frame, the innovation is zero and the wrong value is accepted. If it appears intermittently with at least 7 two-sided strips, the spike guard commits it after 7 frames.

**Conditions for the reproduced failure:**
- line mode;
- every accepted midpoint has exactly the same x;
- a platform whose SVD returns a negative sign for y-descending input.

**Practical consequence:**
- *Intrinsic, reproduced:* a maximum-rate turn command at full speed on the nominal ideal input.
- *Physical:* untested.

**Narrowest correction (proposal):** `b_out = -w_out * c[0]` at `:918`, plus a regression test that runs both point orders.

### 3.2 A-02: Smoother applies the rate limit before the deadband and stores the deadbanded value, so steering can lock at zero

**Where:** `mr_vs.py:435-446`.

**The order of operations:**
1. low-pass;
2. rate limit to `max_w_rate*dt`;
3. clamp;
4. deadband: if `|w_lpf| < w_deadband`, set it to 0;
5. store the result: `self._prev_w = w_out`.

**Why it locks:** starting from zero, the first step is at most `min(w_alpha*|w_clamped|, max_w_rate*dt)`. If that step is below the deadband, the output is set to 0 and that 0 is stored. The next call starts from 0 again, so the output never leaves zero.

**Condition 1: fast updates.** If `max_w_rate*dt < w_deadband`, the output stays at zero for **any** error once it is at zero.
- With defaults (1.2 rad/s², 0.02 rad/s) the threshold is dt < 0.0167 s, i.e. updates faster than 60 Hz.
- This matches the reproduction: zero output at dt = 0.01 and 0.005, while the raw command was -0.3125.
- Once steering is running at these rates, any return through the deadband (settling, or a change of turn direction) locks it again.
- The integrator (`ki`) cannot unlock it, because it only raises `w_clamped`, not the step size.

**Condition 2: small demands, any rate (secondary).** From zero, a steady demand below `w_deadband/w_alpha ≈ 0.057 rad/s` (rather than 0.02) is also zeroed forever.
- Arithmetic: dt = 0.05, raw -0.05, low-pass step -0.0175, which is inside the deadband, so 0.
- The integrator eventually pushes the demand past this at normal rates, by my hand estimate after several seconds for a 10 px error.

**Bypasses that exist:**
- `smooth=False`. This is used only in the legacy path with neither temporal filter nor lookahead (`run_mr_navigation.py:334`, `mr_navigation_stack.py:332`); folder mode uses it by default.
- The CLI flags `--w-deadband 0` or a larger `--max-w-rate` (`run_mr_navigation.py:750-751`).
- The simulation adapter exposes no override for either parameter.
- The same order of operations exists in `temporal_filter.CommandSmoother.smooth` (`:546-577`). That class is not used by any runner, so it is latent only.

**Where updates faster than every 16.7 ms come from:**
- **Python API:** callers pass `dt` as the source frame interval (`docs/python-api.md:71`). Any source at 60 fps or more triggers the lock deterministically.
- **Host-clock paths (F-06):** `run_video`, `MultiROINavigationStack`, and the simulation (`dt=None`) all use the wall-clock time between calls. This is below 16.7 ms whenever processing is fast or frames arrive back to back.
  - Simulation at 20 Hz: nominally about 0.05 s, so locking is likely only transient on queued frames.
  - Offline video: depends on processing speed and needs measurement.

**Practical consequence:**
- *Intrinsic, reproduced:* zero steering while forward speed stays at the confidence-scaled value, so the robot drives straight regardless of where the corridor is.
- *Physical:* untested.

**Narrowest correction (proposal):** keep the rate-limited value as the state (`self._prev_w = w_lpf` before the deadband) and apply the deadband only to the published output, or apply the deadband before rate limiting. Mirror the change in `CommandSmoother` or remove that class.

### 3.3 F-04 raised to High

**The reproduction:**
- 0 two-sided strips, 10 accepted points;
- `has_line=True`;
- status `pending`, confidence 0.115;
- **v = 0.103 m/s**, no stop.

**Why High:**
- It applies in every configuration, including the simulation adapter that actuates the simulated rover.
- The trigger is common: row ends, bare patches, a covered or black camera.
- Nothing in the stack turns loss of perception into a stop.
- The documented `held_no_line` contract (technical documentation section 7, lines 319-321) is unreachable.

**Counterpoint:** `docs/python-api.md:43-44` warns that a returned line is not sufficient evidence, and confidence does drop to about 0.11, so an external supervisor could act on that. If you confirm such a supervisor is required in deployment, Medium is defensible.

**Interaction with A-01:** in line mode, on the reviewer's platform, this fallback becomes a saturated turn (derived in 3.1, not reproduced).

### 3.4 F-07 keeps Medium, with a deterministic case

`commit_min_two_sided = 7` is fixed (`temporal_filter.py:124`) and is not scaled to `n_strips`. With `--n_strips 5` (or anything below 7), `enough_evidence` at `:399-401` can never be true. A large deviation is then never committed: the reproduction shows a 200-frame hold at 5 configured strips.

It stays Medium because:
- small and gradual changes are still tracked by the moving average;
- the default 10-strip configuration needs sparse evidence to trigger the hold;
- the closed-loop result of steering on frozen geometry is unmeasured.

### 3.5 Findings whose severity I did not raise

- **F-01:** the sign flip is reproduced. Its only default-on paths are the analysis CLI and `MultiROINavigationStack`, whose own simulator is purely kinematic. The lateral term still corrects, and stability depends on gains. So it stays High, not Critical.
- **F-06:** stays Medium. Its worst effect is enabling A-02, and I have rated A-02 on its own.
- **F-09:** stays Low. The reproduced "after 100 missing frames then one observation" route goes through `has_line=False`, which F-04 makes unreachable from the shipped pipelines. The interleaving route is reachable, but it commits at most 35% of the deviation, which then washes out.

## 4. Why a previous audit could report Critical findings without any code difference

My definitions treat **Critical** as requiring all three of:
1. the documented **default** configuration;
2. a **commonly met** input;
3. no mitigation in the stack.

Measured against that:

| Finding | Default config | Common input | No mitigation |
| --- | --- | --- | --- |
| A-01 | No (line mode, or a sparse/platform-dependent spline case) | Yes | Yes |
| A-02 | Yes (smoothed paths) | No (needs updates faster than 60 Hz) | Yes |
| F-01 | Yes, but only on paths that do not drive a robot | Yes | Yes |
| F-04 | Yes | Yes | Mostly (it degrades to about half speed with low confidence, which a supervisor could act on) |

None meets all three on evidence, and F-04 is the closest.

Another audit could reasonably reach Critical without seeing different code if it:

- **scored by worst-case physical outcome, assuming the stack drives a robot.** On that basis, F-01 (wrong-sign heading), A-01 (full-rate turn on an ideal corridor), A-02 (steering disabled) and F-04 (no stop) could each be called Critical;
- **scored each supported mode on its own.** A-01 is Critical within line mode under my own definitions;
- **counted symptoms or entry points separately**, for example F-01 in the CLI and in the stack class, or F-06 and A-02 as one combined timing finding;
- **assumed an actuating deployment** with no external supervisor.

None of this means the code changed between audits. The runtime hashes match.

**To compare the two audits, I would need from the previous report:**
- its severity definitions, and how it weighed likelihood against impact;
- the deployment context it assumed (analysis only, simulation, or hardware actuation, and whether a supervisor exists);
- each finding's file, lines, trigger and claimed consequence, so findings can be matched by root cause rather than by title;
- whether each finding was executed or argued from the code, and on what platform (relevant to the SVD sign in A-01);
- whether it counted per root cause, per entry point, or per configuration.

## 5. Decisions for you before any fixes

1. **Deployment intent.** Will any path command real hardware, and will there be an independent supervisor or e-stop? This decides whether F-01, F-04, A-01 and A-02 move to Critical.
2. **Status of line mode.** Is `--line` / `line_fit:=true` a primary supported mode? If so, A-01 is Critical within it.
3. **Loss-of-perception policy (F-04, F-07).** Should no evidence, or a hold that never resolves, lead to a stop, a timeout, or only a confidence signal? The commit floor should probably scale with `n_strips`.
4. **Update rate (A-02, F-06).** What camera and processing rates matter? Should the source interval (frame timestamps or simulation time) become mandatory instead of the host clock?
5. **Lookahead default (F-01, F-02, F-03, F-11, F-12).** Keep it enabled by default for video and camera runs before it is corrected and validated in closed loop, or disable it by default until then?
6. **Platform coverage for A-01.** Should the regression tests run on more than one BLAS backend?
7. **Order of work.** I propose writing failing regression tests first for A-01, A-02, F-01, F-02, F-03 and F-04, then making the narrow fixes listed in each finding, then running Gazebo to measure the closed-loop consequences that nobody has tested yet. This is a proposal only; no changes or commits have been made.