# Implementation and validation report

| Item | Value |
| --- | --- |
| Investigation checkout | Bugfix implementation based on `cedff9a`; see this repository's commit history |
| Source baseline | `d5c68594e8e633b4569a4a59f8c60f33ac785cf0` (official repository, unchanged) |
| Scope | Synthetic and local-sequence tests, CLI examples, headless Gazebo simulation |
| Not done | No physical robot test |

The design intent stays authoritative. The corrections preserve the identity
of the driven furrow: strict holds against unsupported sharp changes,
lookahead memory guiding the search windows, and command smoothing. No gate
was removed and no evidence threshold was lowered for the default
configuration. No acceptance happens because time has passed. The official
checkout `/home/ac/multi-roi-crop-navigation` was not modified.

## Corrections by root cause

| ID | Root cause | Correction | Files |
| --- | --- | --- | --- |
| A-01 | Vertical TLS line intercept ignored the slope sign | `b = -w*x` for either SVD sign | `test_multi_roi.py` |
| A-02 | Deadbanded zero stored as smoother state, so steering locked at zero above ~60 Hz | State keeps the rate-limited value. The deadband is applied to the output, which is rate-limited again, so the published step is never larger than `max_w_rate*dt`. Low-pass `w_alpha` is defined per 0.05 s and rescaled to the actual interval. Same fix in the unused `CommandSmoother`. | `mr_vs.py`, `temporal_filter.py` |
| F-01 | Map heading had the opposite sign to the controller | Map uses the `mr_vs` convention | `lookahead_corridor_map.py` |
| F-02, P-09, P-11 | Map mixed cropped and full-image coordinates and used its own strip grid; overlays used neither | Map works in the detector frame with the detector's strip rows. Outputs are full-image values referred to the image bottom row. Overlays use the detector rows plus the crop offset. | map, `run_mr_navigation.py`, `mr_navigation_stack.py` |
| F-04 | A fabricated fallback line was reported as an observation, pulling held tracks toward the image centre | No line without at least 2 measured midpoints. `nav_source` reports observed, prior or none, and counts are honest. Map exposes `track_valid` and `track_source`. Filter replaces a placeholder state with the first real observation. Without an established track, commands are zero. | detector, map, filter, runner |
| F-04b | Map hold features were passed to the filter as fresh evidence when other strips were observed | Filter input is observed only when the map output is observed. Remembered frames never start, advance or reset a pending candidate. | runner, filter |
| F-05 | ROS locks read a key that never existed | `info.has_observation`, `has_track`, `has_line` (= `has_track`) and `track_source`. Locks use fresh observation. | runner, `nav_node.py` |
| F-06 | Control interval came from host processing time | Simulation image-stamp dt goes through pipeline and node (front and rear). Video uses file timestamps. Folder runs use a nominal 0.05 s. One dt drives low-pass, rate limit and integrator. | runner, stack, pipeline, node, `mr_vs.py` |
| F-07 | Fixed 7-strip commit floor was unreachable with ignored or fewer strips | Floor is `ceil(0.7 * (n_strips - ignore_initial))` (7 at the defaults). An explicit unreachable floor is rejected. Ignored strips never count as evidence. | filter, detector |
| F-08 | Perception errors skipped publishing and every run-end check | A decode or pipeline failure publishes zero and still runs the time, lane and lap checks. Optional camera watchdog `camera_timeout` (default 0 = off). | `nav_node.py`, launch, `run_sim.sh` |
| F-09 | Contradicting observations did not break a pending streak | A fresh observation near the held state ends the candidate. Missing frames keep it (intended). | filter |
| F-10 | Spline robust refit ignored its retained subset | Refit on the retained points; extension, tangent and envelope use the subset | detector |
| F-03, P-10 | Strip/bin mismatch crashed on frame 2; one strip crashed | Explicit `ValueError` and CLI `parser.error`. `--lookahead-bins` defaults to `--n_strips`. | map, detector, CLIs |
| P-07 | Interpolation pulled an observed bin toward an unobserved placeholder | The more confident bin is used alone | map |
| P-08a/b | Proxy bottom consistency read from the missing strip; new-geometry support counted against an ignored strip | One reference strip for consistency, prediction, holds and support; the reference itself is excluded from support | map |
| F-13 | Controller normalization ignored the frame size | `geometry_from_frame` when `--width/--height` are not given. Explicit values are kept, with a warning on mismatch. | `mr_vs.py`, runner, stack |
| F-14 | `/reset_rover` left navigation state in place | Node resets perception, map, lap and row-change state | `nav_node.py`, `pipeline.py` |
| F-15 | Midpoints were tagged at the strip top, not the measured rows | Strip centre row; `accepted_nav` matching updated. The Eq. (10) spacing is unchanged. | detector |
| F-16, F-18 | Off-by-one `strip_of`; duplicate summary append | Fixed | detector |
| F-17 | Stop destroyed the node inside its callback; spin never returned | Idempotent stop, then `NavRunFinished` ends spin; `main` cleans up | `nav_node.py` |
| Sim | No way to run the complete design in simulation | `lookahead:=true` / `--lookahead` (default off, keeping the historical adapter configuration) | pipeline, launch, `run_sim.sh` |

### Held-heading compensation in this variant

`held_yaw_gain = 1.0` (temporal filter): on frames without a fresh
observation, the held heading is rotated by the rover's own commanded yaw.
This was added because Gazebo showed a row-end failure. The last noisy view
gave a -11 degree heading. The pure hold, now reachable after the F-04
correction, then commanded a constant 0.20 rad/s for 11 s, and the rover
drifted 0.74 m.

The rule reuses the filter's existing motion-prediction term and applies only
to unobserved frames, so gates on observed data are unchanged. No translation
model was added. Setting the gain to 0 restores a pure hold.

The review retains this correction in the bugfix variant because the local
regressions and post-correction simulation runs exercise it. It uses commanded
yaw as an approximation of actual motion; this does not validate a complete
motion model or hardware behaviour.

## Validation

| Check | Command | Result |
| --- | --- | --- |
| Unit and regression tests | `env -u PYTHONPATH -u PYTHONHOME python3 -m pytest -q` | 108 passed. This is 8 inherited, 4 independent-review (`tests/unit/test_independent_review.py`) and 96 bug-fix tests. Local pytest 6 prints a `pythonpath` option warning; `tests/unit/conftest.py` handles imports. |
| Snapshot and manifest | `python3 scripts/verify_snapshot.py` | Exit 0: 35 snapshot files unchanged, 10 approved changes and 10 added files match `change-manifest.json`. `--strict` exits 1 and lists the changed files, as intended. 13 of the 46 audited baseline files differ (listed in the manifest). |
| Historical defect harness | `audit/reproductions/runtime_checks.py` | 14 of 17 cases no longer reproduce (exit 1 is expected post-fix). Still flagged: F06 (calls the controller without dt by construction), P06 (policy) and P09 (measures parameter arithmetic, not the run-time map grid). See `validation/runtime-checks-postfix.json`. |
| Design sequences | `audit/reproductions/design_intent_checks.py` | Exit 0. Holds and gradual tracking preserved. Steering reaches -0.3125 rad/s at dt 0.05, 0.02 and 0.01. The occlusion estimate stays at 281 px with and without memory. |
| CLI examples | `examples/generate_sample.py`, the three CLIs, `scripts/check_example_outputs.py` | Pass. Lookahead video lateral error mean +0.55 px (baseline about -12 px). |
| Compile | `python3 -m compileall -q tests/sim_ros2/src/mrsim *.py scripts tests/unit` | Pass |
| Distribution | `python3 -m pip wheel . --no-deps --no-build-isolation` | Wheel built. `build` is not installed and was not added. |
| PDF | `python3 scripts/build_technical_documentation.py` | Rebuilt |

Baseline evidence in `audit/evidence/`, `audit/baseline-files.json` and
`docs/source-snapshot.json` is unchanged. Current hashes are in
`change-manifest.json`, which `verify_snapshot.py` checks; `--strict` runs the
original import check.

## Gazebo simulation

Environment: ROS 2 Humble, headless gzserver 11, `ROS_DOMAIN_ID=87`,
`GAZEBO_MASTER_URI=http://localhost:11387`. Only processes started by these
scripts were stopped. Scripts are in `simulation/` and summaries in
`validation/simulation-{pre,post}correction.json`. Raw CSV files and logs stay
under `/tmp/mr_sim`.

Cross-track error is measured against the plant-pose centreline of each
world. The "Pre" runs were made before the independent-review corrections and
before held-yaw rotation. "Post" runs use the final source. Each navigation run reached its own stop condition and its launch returned
zero, with no perception errors except in the injected fault run. The
auxiliary reset-node shutdown exception is recorded below.

| Run | Length m | Pre RMS / max m | Post RMS / max m | Post in-row max (x<=7) m | Observed frames | End |
| --- | --- | --- | --- | --- | --- | --- |
| straight, memory off | 17.0 | 0.054 / 0.222 | 0.028 / 0.070 | 0.069 | 86% | lane end |
| straight, memory on | 17.0 | 0.026 / 0.082 | not rerun | (pre 0.044) | 88% | lane end |
| curve, memory off | 16.4 | 0.079 / 0.180 | 0.088 / 0.212 | 0.212 | 94% | 110 s limit |
| curve, memory on | 17.0 | 0.098 / 0.217 | 0.103 / 0.227 | 0.227 | 87% | lane end |
| crop gaps, memory off | 16.7 | 0.119 / 0.735 | 0.040 / 0.156 | 0.064 | 91% | 110 s limit |
| crop gaps, memory on | 16.2 | 0.014 / 0.038 | 0.016 / 0.047 | 0.036 | 96% | 110 s limit |
| line mode, straight | 17.0 | 0.017 / 0.060 | not rerun | (pre 0.035) | 88% | lane end |

The gap world removes a 1.5 m stretch of one row, 1.5 m of the other, and
0.8 m of both. The forward camera still saw plants through each gap, so these
runs test sparse evidence, not full blindness. Full occlusion is covered by
the local sequences.

**Fault and watchdog runs (pre and post, both pass).** The run used
`camera_timeout:=1.0`. Frames with an unsupported encoding (49 pre, 53 post) each gave
`perception failure ... commanding stop` and a CSV stop row. Pausing physics
triggered `camera stall: no frame for 1.07s, commanding stop`, and `/cmd_vel`
read zero during the pause. Frames resumed, and the run ended at
`max_seconds=45` with a clean launch exit (58 s wall).

**Flagged results:**

- Curved-field error is about 0.2 m in both modes and memory does not reduce
  it. This is consistent with inside-cutting on the S-bend and the fixed
  14 px/frame map shift with no yaw term. It is not resolved.
- Row-end drift remains in the gap run (0.16 m over the last 1 m). The final
  noisy views before the rows leave the image set the held lateral and heading
  terms; no translation model exists.
- `rover_reset.py` exits with `ExternalShutdownException` during launch
  shutdown. This happened before these changes; the file was not modified.
- Measured steering acceleration from the 3-decimal CSV reaches 1.22 rad/s²,
  which is within rounding of the 1.2 limit.

## Open decisions (deliberate policy, not changed)

1. Should simulation default to the complete design (`lookahead:=true`)?
   The default stays off.
2. Lost-track policy after long memory age. Short gaps travel on the held
   track. Commands are zero only before any track exists.
3. Motion model. The map uses a fixed image shift and ignores `dt`/`last_w`.
   The filter has only held-yaw rotation. Validation needs recorded or
   simulated turning sequences.
4. Camera watchdog default (off) and the fail-open ToF guard on stale sensors
   (unchanged).
5. CLI and library gain presets (P-06, unchanged).
6. Reacquisition rule when started on a wrong corridor (not implemented).
7. F-11 (map uses strip 1) and F-12 (bottom bins after a switch) were kept as
   conservative behaviour.
