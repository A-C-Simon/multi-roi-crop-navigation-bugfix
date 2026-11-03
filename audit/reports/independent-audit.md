# Independent technical audit: MultiROI crop-row detection and visual navigation

Baseline: `audit/independent-review` branch, HEAD `d5c6859` (matches the requested baseline d5c68594e8e6). Working tree clean.

## 1. Summary

### What I inspected

I read these files in full and traced their call paths:

| Area | Files |
| --- | --- |
| Runtime detector (the one the navigation stack actually imports) | `test_multi_roi.py` (all 1702 lines) |
| Reference detector | `multi_roi.py` |
| Feature conversion and controller | `mr_vs.py` |
| Temporal filter | `temporal_filter.py` |
| Lookahead map | `lookahead_corridor_map.py` |
| Runners | `run_mr_navigation.py`, `mr_navigation_stack.py` |
| Tests, packaging, CI | `test_temporal_filter.py`, `pyproject.toml`, `requirements.txt`, `constraints-tested.txt`, `.github/workflows/checks.yml`, `scripts/check_example_outputs.py`, `examples/generate_sample.py` |
| ROS 2 / Gazebo adapter | `tests/sim_ros2/src/mrsim/mrsim/{pipeline.py, nav_node.py, tof_guard.py}`, `rover_reset.py` (the respawn path), `launch/farm.launch.py`, `run_sim.sh`, `setup.py`, the diff-drive and camera blocks of `urdf/rover.urdf` |
| Documentation (only to find the stated contracts) | `docs/technical-documentation.md` sections 3 to 10, `docs/python-api.md`, `docs/usage.md`, `docs/validation.md`, `docs/architecture.md`, `docs/simulation.md` |

I did not open `audit/README.md` or `.pytest_cache/`. The first looks like it may be an earlier audit, so I left it alone to keep this review independent.

### Limits of this pass

- **Nothing was run.** No tests, snippets, CLI commands or simulation. Every finding is labelled:
  - **statically demonstrated**: the code path and arithmetic prove it;
  - **likely**: the path is reachable and the consequence follows, but depends on runtime conditions;
  - **requires experiment**.
- The Python snippets below are proposals for the coordinating reviewer to run from the repository root. The expected numbers come from hand calculation, not from execution.
- I did not check the behaviour of external ROS packages (for example, whether `gazebo_ros_diff_drive` on Humble holds the last command). Where a finding depends on that, I say so.

### Important context about configurations

The closed-loop Gazebo adapter (`pipeline.py`) runs **without** the lookahead map and with `morph="paper"`. The offline video and camera CLI and `MultiROINavigationStack` run **with** the lookahead map by default. The docs record this difference (`docs/architecture.md:51-54`, technical documentation section 10.1).

As a result, the most serious control-path defects (F-01, F-02) affect the default offline video/camera path and the public stack class, **not** the closed-loop simulation. Results validated in simulation therefore say nothing about the lookahead path.

## 2. Severity definitions

Severity reflects impact if the trigger occurs. Likelihood is shown separately in the trigger and "default path" fields.

| Severity | Definition |
| --- | --- |
| Critical | In the documented default configuration, a commonly met input produces unsafe or grossly wrong motion commands, or silent data corruption, with no mitigation in the stack. |
| High | A documented default path produces a wrong-direction or materially wrong control output, or the defect invalidates a primary documented capability. Physical harm needs the path to be used for actuation. |
| Medium | Wrong output, crash, or lost safety/robustness behaviour that needs a non-default but reachable configuration, a specific scene condition, or the simulation-only adapter. Or a bounded systematic bias in a default path. |
| Low | Small bounded numerical bias, dead or misleading robustness logic with limited effect, diagnostics or cosmetic errors, or lifecycle issues with a narrow trigger. |

## 3. Findings table

| ID | Sev. | Confidence | Location | Trigger | Practical consequence |
| --- | --- | --- | --- | --- | --- |
| F-01 | High | Statically demonstrated | `lookahead_corridor_map.py` `LookaheadCorridorMap.update` (heading fit); consumed by `run_mr_navigation.process_image`, `MultiROINavigationStack.step` | Lookahead enabled (default for `--video`, camera, `MultiROINavigationStack`), corridor not vertical in the image | The heading fed to the filter and controller has the **opposite sign** to the `mr_vs` convention, so the heading term of `w` steers the wrong way. The filtered red line is drawn leaning the wrong way. |
| F-02 | Medium | Statically demonstrated | `lookahead_corridor_map.update` (`corrected_X`, fallbacks) with `test_multi_roi._run_core` `strip_profile` | Lookahead enabled, default `border_frac=0.02` | Lateral feature mixes cropped and full-image frames: a constant bias of `-dx` px (-12 px at 640 wide). Some fallbacks emit full-frame `w/2`, causing a frame jump. |
| F-03 | Medium | Statically demonstrated | `lookahead_corridor_map._predict_bins`, `update`; CLI `--n_strips` / `--lookahead-bins` | Video mode with `--n_strips` smaller than `--lookahead-bins` (for example `--n_strips 8`) | `IndexError` on the second frame; the video/camera run aborts. |
| F-04 | Medium | Statically demonstrated (closed-loop effect requires experiment) | `test_multi_roi._run_core` fallback; `mr_vs.compute_control`; `temporal_filter.update` | No vegetation, a covered or black camera, or bare soil | The detector always returns a line. "No line" is never reported, the documented `held_no_line` path is unreachable, and no path commands `v=0`. The stack keeps driving at about 50% speed toward a synthetic centred corridor. |
| F-05 | Medium | Statically demonstrated | `nav_node.py` `_pipe_locked`, `_rear_locked`, `_primary_empty` | Simulation `turn_mode:=shuttle` or `fishtail` | `info["has_line"]` never exists, so the rear camera never "locks". Shuttle backward legs end at about 70% of the leg. Fishtail turns never use rear vision and always wait out an 8 s odometry fallback. |
| F-06 | Medium | Statically demonstrated (magnitude requires experiment) | `nav_node._on_image`, `pipeline.process`, `mr_vs.compute_control`, `temporal_filter.update`, `run_video` | Simulation (always); offline video (always) | The controller and filter time base is host wall-clock processing time (integrator fixed at 0.05 s), not frame or sim time. Commands depend on CPU speed and Gazebo's real-time factor. The sim-time `dt` the node computes is thrown away. |
| F-07 | Medium | Requires experiment | `temporal_filter.update` spike-guard branch | Any large position or heading innovation while fewer than 7 strips are two-sided | The state is held indefinitely (no timeout), so the controller steers on frozen geometry. A code comment claims a timeout that does not exist. |
| F-08 | Low | Likely | `nav_node._on_image`, `_decode_bgr`; `rover.urdf` diff-drive | Pipeline exception, unsupported encoding, camera stall | No command is published, so the last `/cmd_vel` keeps running (fail-open). There is no watchdog. |
| F-09 | Low | Statically demonstrated | `temporal_filter.update` low-evidence branch | Large deviations interleaved with low-evidence frames | The documented "consecutive coherent frames" requirement is not enforced, so a spike can be committed early. |
| F-10 | Low | Statically demonstrated | `test_multi_roi.MultiROIDetector._fit_nav_curve` | Default spline mode | The "robust refit" never changes the spline: `keep` is ignored. Outlier dots still shape the curve and the base tangent, within the envelope. |
| F-11 | Low | Likely | `lookahead_corridor_map.update` bottom selection vs `_run_core` nav gating | Lookahead enabled, default `init_window_frac=1.0` | The lookahead path's primary lateral measurement is strip 1, which the detector deliberately excludes from navigation as unreliable. |
| F-12 | Low | Requires experiment | `lookahead_corridor_map.update` bin refresh after `map_switch` | A real lane shift with lookahead on | The bottom bins keep the old geometry after a switch, contrary to the comment, so convergence is slower than designed. |
| F-13 | Low | Statically demonstrated | `run_video`, `MultiROINavigationStack.step` vs `MRVSParams.width` | Frame width differs from `--width` (default 640) | The filter and map auto-resize but the controller does not, so the effective lateral gain and heading gate scale with resolution. |
| F-14 | Low | Likely | `rover_reset.py`, `nav_node.py` | `/reset_rover` or `r` key in auto mode | The rover respawns but the nav node keeps its filter, integrator, lap angle and row-change state. |
| F-15 | Low | Statically demonstrated | `_run_core` Q-point y vs chain/strip-profile y | Slanted corridor | Midpoints are tagged at the strip top, not the strip centre: a lateral bias of `slope*dh/2` at the base (about 2 to 3 px at 8 degrees). |
| F-16 | Low | Statically demonstrated | `pre_corridor_clean.strip_of` | Default `morph="struct"` | Off-by-one strip index, so the band-tolerance window is shifted down one strip. |
| F-17 | Low | Requires experiment | `nav_node._stop_robot`, `main` | Run ends by stop reason | `_stop_robot` is invoked again on a destroyed node during shutdown. The process may raise, or may not leave `spin`. |
| F-18 | Low | Statically demonstrated | `test_multi_roi.main` | `--post_scrub` with scrub applied | The `deltas` list gets each entry twice, so the summary count is doubled. Cosmetic. |

## 4. Detailed findings

A shared helper used by several snippets (run from the repository root):

```python
import math, time, cv2, numpy as np
from test_multi_roi import MultiROIDetector
from mr_vs import MultiROIVS, MRVSParams
from temporal_filter import TemporalNavigationFilter, TemporalFilterParams
from lookahead_corridor_map import LookaheadCorridorMap, LookaheadParams
from run_mr_navigation import process_image

def rows(lean_deg=0.0, h=480, w=640, xl=250, xr=390):
    """Two green rows on soil. lean_deg > 0: rows lean RIGHT toward the image top."""
    img = np.full((h, w, 3), (60, 65, 120), np.uint8)
    d = int(round(h * math.tan(math.radians(lean_deg))))
    for xb in (xl, xr):
        xt = xb + d
        poly = np.array([[xb-5, h-1], [xb+5, h-1], [xt+5, 0], [xt-5, 0]], np.int32)
        cv2.fillPoly(img, [poly], (30, 200, 30))
    return img
```

---

### F-01 (High): Lookahead heading has the opposite sign to the controller's Theta convention

**Status:** statically demonstrated.

**Code path**

- `mr_vs.py:292-295` defines the controller heading feature: `Y = P.y - Q.y`, `X = Q.x - P.x`, `Theta = pi/2 - atan2(Y, X)`. A corridor leaning **right** toward the top (`Q.x > P.x`) gives **Theta > 0**. The technical documentation (section 8.1, lines 339-345) states this convention.
- `lookahead_corridor_map.py:366-374` computes `raw_theta` by fitting `x = a*y + b` over strip centres in image coordinates (y down), then takes `raw_theta = atan(a)`. For the same right-leaning corridor, x grows as y shrinks, so `a < 0` and **raw_theta < 0**. `pred_theta` (`:474-486`) uses the same formula. `corrected_theta` (`:490-497`) is one of these two or a blend of them, so it always has the inverted sign.
- `run_mr_navigation.py:196-203` and `mr_navigation_stack.py:240-246` put `map_out["corrected_theta"]` into `raw_theta` for the temporal filter. `temporal_filter._build_output` (`:479`) passes it into `filt_F[2]`, and `mr_vs.compute_control` (`:410`) computes `w_raw = -(lambda_x*e + lambda_theta*Theta*gate) ...`.

Worked example: bottom point (300, 400), top point (340, 200).
- `mr_vs`: `Theta = pi/2 - atan2(200, 40) = +0.197` rad.
- Lookahead: `a = (340-300)/(200-400) = -0.2`, so `atan(a) = -0.197` rad.

**Expected vs actual**

- Expected: the same geometry gives the same sign through both paths, so the heading term in `w` turns the robot toward the corridor's heading.
- Actual: with lookahead on, the heading term has the opposite sign. In the overlay, the filtered red line (drawn from `filt_theta`, `temporal_filter.py:485-492`) leans opposite to the raw cyan line on slanted corridors.

**Closed-loop consequence (requires experiment).** In a linearised model with `X ≈ e + L*psi` and `Theta ≈ psi`, the damping coefficient becomes `kx*L - ktheta` instead of `kx*L + ktheta`. That reduces damping and can destabilise if `ktheta > kx*L`.

**Default path reached?**
- Yes: `multiroi-nav --video` (file or camera, `run_mr_navigation.py:822-829`), `MultiROINavigationStack(...)` (default `use_lookahead=True`, `mr_navigation_stack.py:113`), and `demo_offline`.
- No: the Gazebo adapter (`pipeline.py:220`, `lookahead_map=None`).
- The CI video is symmetric (heading close to zero, `examples/generate_sample.py:18-20`), so CI cannot reveal this.

**Repro**

```python
bgr = rows(8.0)
for use_map in (False, True):
    det = MultiROIDetector(morph="struct")
    vs = MultiROIVS(MRVSParams(width=640, height=480))
    tf = TemporalNavigationFilter(TemporalFilterParams(image_width=640, image_height=480, n_strips=det.n))
    lm = LookaheadCorridorMap(LookaheadParams(image_width=640, image_height=480, n_bins=det.n)) if use_map else None
    out = process_image(bgr, det, vs, draw=False, t_filter=tf, dt=0.05, last_w=0.0, lookahead_map=lm)
    print(use_map, round(out["info"]["filt_err_theta_deg"], 2), round(out["info"]["w_raw"], 3))
# Hand-derived expectation: without map Theta ~ +8 deg, w_raw ~ -0.14;
# with map Theta ~ -8 deg, w_raw ~ +0.17 (sign flip in both)
```

**Narrowest correction (proposal):** in `lookahead_corridor_map.update`, negate both fitted angles (`raw_theta = -atan(a)`, `pred_theta = -atan(a)`), or compute them with the same `P`/`Q` formula as `mr_vs`. Add a test that asserts the sign matches between the two paths on a slanted synthetic image.

---

### F-02 (Medium): Lookahead lateral output mixes cropped-image and full-image coordinates

**Status:** statically demonstrated.

**Code path**

- `strip_profile[*]["center_x"]` (`test_multi_roi.py:719-722`) comes from cluster centres in the **cropped** binary frame (`:454-455`; `cluster_feature_columns` adds the cropped-frame `x_lo`).
- `LookaheadCorridorMap.update` receives the **full** image shape (`run_mr_navigation.py:193`) and computes `corrected_X = corrected_bottom_x - w/2.0` with the full width (`lookahead_corridor_map.py:605`). The crop offset is never added.
- Its fallbacks use full-frame `w/2.0` (`:362`, `:422`). So when strip 1 and its proxies lack two-sided evidence, the output jumps by `dx` between frames.
- By contrast, `mr_vs.nav_line_to_feature` adds `dx` (`mr_vs.py:248-249`, `:280`). The documented contract (technical documentation 3.1, lines 121-122) says feature conversion adds the crop offset.
- A secondary definitional mismatch: the map's "bottom x" is the strip-1 centre at about `h - dh/2` in cropped coordinates, while `mr_vs`'s `P` is at the image bottom. These differ by `slope*(dh/2 + dy)`.

**Expected vs actual (640 px wide, dx = 12)**

- Expected: a corridor centred in the image gives `X ≈ 0`.
- Actual: the lookahead path gives `X ≈ -12 px`. With CLI `--lambdax 10`, that is `w ≈ +0.19 rad/s` of spurious left turn on a perfectly centred corridor, before smoothing, and the integrator (`ki=0.3`) winds toward it.
- In closed loop this would become a steady-state offset of about 12 px at strip-1 height, roughly 9% of a 140 px corridor.

**Default path reached?** Same as F-01. It should be visible in the existing CI artifact `outputs/video/corridor_nav.csv`: `filt_err_x_px` should sit near -12 while `raw_err_x_px` sits near 0 (±6 from the synthetic shift).

**Repro**

```python
for lean in (0.0, 8.0):
    bgr = rows(lean); det = MultiROIDetector(morph="struct")
    vs = MultiROIVS(MRVSParams(width=640, height=480))
    lm = LookaheadCorridorMap(LookaheadParams(image_width=640, image_height=480, n_bins=det.n))
    res = det.detect(bgr)
    F, _ = vs.nav_line_to_feature(res["nav_line"], res["nav_curve"], res["crop_offset"],
                                  bgr.shape[:2], vertical_coverage=res["vertical_coverage"])
    m = lm.update(res["strip_profile"], res, bgr.shape[:2])
    print(lean, round(F[0], 1), round(m["corrected_X"], 1), res["crop_offset"])
# Expected (hand-derived), lean 0: raw X ~ 0, map X ~ -12
```

**Narrowest correction (proposal):** pass `crop_offset` into `LookaheadCorridorMap.update` and add `dx` to every x value it emits (`corrected_bottom_x`, left/right, `pred_bottom_x`), or emit cropped values and subtract the cropped width/2. The fallbacks must use the same frame. Separately, decide whether the map should extrapolate to the image bottom like `mr_vs`.

---

### F-03 (Medium): Lookahead bin count different from strip count crashes on frame 2

**Status:** statically demonstrated.

**Code path**

- The CLI sets the detector strip count from `--n_strips` (`run_mr_navigation.py:737`, `:775`) and the map bin count independently from `--lookahead-bins` (`:757`, `:834`).
- `update` rebuilds `self.bins` from `strip_profile`, which has `detector.n` entries (`lookahead_corridor_map.py:501-592`).
- On the next frame, `_predict_bins` loops over `range(p.n_bins)` and reads `self.bins[lo]` and `self.bins[hi]` (`:143-166`). If `n_bins > len(self.bins)`, that raises `IndexError`.
- `get_prediction` is wrapped in `try/except` in `process_image` (`run_mr_navigation.py:136-141`), so the first failure there is silently swallowed. But `lookahead_map.update` (`:193`) is not wrapped, and `run_video` only catches `KeyboardInterrupt` (`:712`).

**Expected vs actual:** expected either validation or tolerance; actual is a crash on the second frame.

**Default path reached?** No. It needs a non-default `--n_strips` (smaller than 10) with the default `--lookahead-bins 10`, or the reverse with `n_bins > n_strips`. `MultiROINavigationStack` ties them together and is safe.

**Repro**

```python
det = MultiROIDetector(n_strips=8); lm = LookaheadCorridorMap(LookaheadParams(n_bins=10))
bgr = rows(0.0)
for i in range(2):
    res = det.detect(bgr); lm.update(res["strip_profile"], res, bgr.shape[:2])  # IndexError at i == 1
```

CLI equivalent: `multiroi-nav --input examples/data/corridor.mp4 --video --output /tmp/f03 --n_strips 8`

**Narrowest correction (proposal):** in `main`, set `n_bins=detector.n` (or reject a mismatch at argument parsing). Optionally assert `len(strip_profile) == p.n_bins` in `update`.

---

### F-04 (Medium): Loss of perception becomes a synthetic centred corridor; "no line" is unreachable and nothing stops the robot

**Status:** detector behaviour statically demonstrated. Closed-loop effect requires experiment.

**Code path**

1. `_run_core` (`test_multi_roi.py:613-623`): if fewer than 2 evidence points qualify, `nav_pts` falls back to all strip midpoints `q`. That always has `n >= 2` entries, so `_fit_line_tls` always returns a line. For `n_strips >= 2`, `nav_line` is never `None`. `docs/python-api.md:35` says `nav_line` may be "None".
2. On a frame with no vegetation, every `mo_x` stays at the initial `w/2` (or the lookahead seed). The total-least-squares fit gives a vertical line, `q_accepted` holds all 10 points (zero residuals, no rejections), `n_q_accepted = 10`, and the spline is fitted as well.
3. `_build_raw_dict` (`run_mr_navigation.py:72`) therefore always has `has_line=True`. The filter's `not has_line` path (`temporal_filter.py:308-328`), the "first frame without a line" initialisation (`:272-285`), and `compute_control`'s `F is None` branch (`mr_vs.py:334-360`) are all unreachable from the shipped pipelines. The technical documentation (section 7, lines 319-321) documents the `held_no_line` behaviour.
4. Confidence arithmetic for a blank frame: `n_two=0`, `n_q=10`, `fail_rate=1.0`, so `raw_conf = (0.35 + 0.10) * (1 - 0.45) ≈ 0.25`. That is below 0.35, so the frame takes the low-evidence path, which slow-follows (8% per frame) toward the centred vertical line. The state confidence converges to about 0.12, so `v ≈ vf*(0.45 + 0.55*0.12) ≈ 0.51*vf`. No path produces `v = 0`; even the unreachable `F is None` branch keeps `v >= 0.45*vf`.

**Expected vs actual**

- Expected (per the documented contract): no evidence leads to `held_no_line` and decaying confidence, with a caller-visible "no line" state.
- Actual: the detector fabricates a centred line, `has_line` reads true in the CSV, and the stack keeps moving at about half speed, steering toward image centre and vertical.

**Default path reached?** Yes, in all configurations, including the simulation adapter.

**Repro**

```python
bgr = np.full((480, 640, 3), (60, 65, 120), np.uint8)       # bare soil
det = MultiROIDetector(morph="struct"); vs = MultiROIVS(MRVSParams())
tf = TemporalNavigationFilter(TemporalFilterParams(n_strips=10))
res = det.detect(bgr); print(res["nav_line"], res["n_two_sided"], res["n_q_accepted"])
for _ in range(40):
    out = process_image(bgr, det, vs, draw=False, t_filter=tf, dt=0.05, last_w=0.0)
print(out["info"]["status"], round(out["v"], 3), round(out["info"]["confidence"], 2))
# Expected (hand-derived): steep line near x=308 (cropped), 0, 10; status "pending", v ~ 0.10, conf ~ 0.12
```

**Narrowest correction (proposal):** in `_run_core`, return a flag such as `res["evidence_fallback"] = fallback`, or `nav_line=None` when `measured` is empty. In `_build_raw_dict` and in the stack, set `has_line = F is not None and not res.get("evidence_fallback")`. Whether to add a stop or a timeout is a policy decision for the reviewer.

---

### F-05 (Medium): Simulation rear-camera "lock" is never detected because `info` has no `has_line` key

**Status:** statically demonstrated.

**Code path**

- `nav_node._pipe_locked` (`:477-489`) and `_rear_locked` (`:736-748`) return `None` unless `info.get("has_line", False)` is true.
- The `info` dict from `process_image` never contains `has_line`. The temporal path builds it from `compute_control` (`mr_vs.py:453-465`) plus the keys added at `run_mr_navigation.py:239-258`. Neither the lookahead-only path nor the raw path adds it either. The key exists only in `filt_out`, not in `info`.

**Consequences**

- **Shuttle, backward legs:** `_primary_empty` (`:518-525`) sees `locked is None` and returns `True` on every tick. So `_shuttle_end_triggered` (`:532-556`) fires 10 frames after passing 70% of the leg, whatever the rear camera actually sees. The backward lane is not covered past about 70%.
- **Fishtail, reverse phase:** `info` is always `None`, so the code always uses the odometry crab (`:785-796`). `rear_ok` only becomes true after the 8 s no-lock grace period (`:810-819`). Rear-vision steering (`:776-784`, "sign verified in sim") never runs.

**Expected vs actual:** the docstrings say vision decides the row end and rear-guided reversing is used. Actually, odometry and timers decide.

**Default path reached?** No. `turn_mode` defaults to `bulb`. `run_sim.sh` auto-enables row change on fields with 2+ furrows, but still with bulb turns unless `--turn-mode` is passed.

**Repro**

```python
det = MultiROIDetector(); vs = MultiROIVS(MRVSParams())
tf = TemporalNavigationFilter(TemporalFilterParams(n_strips=10))
out = process_image(rows(0.0), det, vs, draw=False, t_filter=tf, dt=0.05, last_w=0.0)
print("has_line" in out["info"])     # False, so _pipe_locked/_rear_locked always return None
```

Simulation confirmation: `run_sim.sh --straight 5 --turn-mode fishtail` should log "fishtail: no rear lock for 8s" on every turn. With `--turn-mode shuttle`, backward legs should end near 70% of the leg distance.

**Narrowest correction (proposal):** in `_pipe_locked` and `_rear_locked`, read `pipe_out["filt_out"]["has_line"]` (or `pipe_out["res"]["nav_line"] is not None`), or add `info["has_line"] = has_line` in `process_image`. Given F-04, `has_line` alone is a weak lock criterion, so pairing it with `n_two_sided` or `confidence` should be considered.

---

### F-06 (Medium): Time base is host wall-clock processing time, not frame or sim time

**Status:** statically demonstrated. Magnitude requires experiment.

**Code path**

- **Simulation:** `nav_node._on_image` computes a sim-time `dt` (`:942-949`) that is never used afterwards. `pipeline.process` calls `process_image` without `dt` (`pipeline.py:218-222`). So:
  - `TemporalNavigationFilter.update` uses `perf_counter` differences (`temporal_filter.py:250-255`);
  - the `compute_control` smoother and rate limiter use their own `perf_counter` (`mr_vs.py:425-433`);
  - the integrator uses a fixed `dt_i = 0.05` whenever `dt is None` (`mr_vs.py:391-393`), whatever the true frame interval.
- **Offline video:** `run_video` (`run_mr_navigation.py:649-652`) uses the wall-clock time between loop iterations, which is detector processing time, not the frame interval from `CAP_PROP_FPS`.

**Expected vs actual**

- Expected: rate limits (`max_w_rate` in rad/s²) and the integrator use the source-frame or sim-time interval, as `docs/python-api.md:71` states for `dt`.
- Actual:
  - The per-frame angular change allowance scales with host speed. At a Gazebo real-time factor of 0.5, the limiter allows twice the documented rate in sim time.
  - The integrator's gain per sim-second scales with frame rate.
  - Offline CSV output is not reproducible across machines.

**Default path reached?** Yes, in both the simulation and the offline video path.

**Repro (offline, shows host-timing dependence)**

```python
F = np.array([100.0, 0.0, 0.0])
a = MultiROIVS(MRVSParams()); a.compute_control(F, confidence=1.0); time.sleep(0.4)
_, w_slow, _ = a.compute_control(F, confidence=1.0)
b = MultiROIVS(MRVSParams()); b.compute_control(F, confidence=1.0)
_, w_fast, _ = b.compute_control(F, confidence=1.0)
print(w_slow, w_fast)   # same inputs; hand-derived about -0.15 vs about -0.06
```

**Narrowest correction (proposal):**
- Simulation: pass the node's `dt` through `pipeline.process(bgr, dt)` into `process_image`.
- `run_video`: use `1/fps` or `CAP_PROP_POS_MSEC` deltas.
- `compute_control`: when `dt` is supplied, use it for both the smoother and the integrator.

---

### F-07 (Medium): Spike-guard commit floor with no timeout can freeze the navigation state indefinitely

**Status:** requires experiment for the closed-loop effect. The hold itself is statically demonstrated.

**Code path.** `temporal_filter.py:360-434`. A large innovation (more than about 64 px or 12 degrees) enters the pending path. A commit needs `pending_count >= 7` **and** `n_two_sided >= 7` (`:399-401`). Until then the state is held exactly at the predicted value (`:426-429`). Nothing bounds this. The comment at `:321-322` ("pending will timeout eventually") describes a timeout that does not exist. While held, `filt_F` stays fixed, so the controller issues a constant `w` on stale geometry while `v` decays only to the confidence floor.

**Expected vs actual**

- Expected (`temporal_filter.py:37-41` docstring): "Genuine persistent changes ... are still tracked."
- Actual: this holds only if at least 7 strips are two-sided. With 6 or fewer (sparse rows, coverage 0.75, perspective), a genuine change is never followed.

**Counterpoint.** The evidence floor is a deliberate and documented tradeoff (technical documentation section 7, lines 312-315). The open question is whether an unbounded hold is acceptable, not whether the floor exists.

**Repro**

```python
tf = TemporalNavigationFilter(TemporalFilterParams(n_strips=10))
def feed(x, n2): return tf.update({"raw_bottom_x": x, "raw_theta": 0.0, "raw_width": 140.0,
    "has_line": True, "n_two_sided": n2, "n_q_accepted": n2, "n_q_rejected": 0, "weed_pressure": 0.0}, dt=0.05)
feed(320, 9)
for _ in range(200): o = feed(420, 6)       # genuine +100 px shift, 6 two-sided strips
print(o["status"], o["filt_bottom_x"], o["pending_count"])   # expected: pending 320.0 200
```

**Narrowest correction (proposal):** add a bounded hold, such as a maximum pending duration after which the filter either commits with reduced confidence or reports a hard "lost" state that the stack maps to a stop. Fix the misleading comment either way.

---

### F-08 (Low): Simulation node fails open on pipeline errors and camera stalls

**Status:** the node side is statically demonstrated. That the plugin holds the last command is likely and should be confirmed.

**Code path**

- `nav_node._on_image` returns without publishing when decoding fails (`:955-962`) or `pipeline.process` raises (`:964-968`).
- There is no timer or watchdog for missing frames.
- The diff-drive plugin block (`rover.urdf:291-316`) has no command timeout. On Humble's `gazebo_ros_diff_drive` I expect the last command to persist; please confirm.
- Reachable exception sources exist and are uncaught, for example `LSQUnivariateSpline` in `_fit_nav_curve` (`test_multi_roi.py:811`). I did not prove one is triggerable on realistic frames.

**Expected vs actual:** expected a zero command or a decaying command when perception fails. Actually, the last `(v, w)` persists.

**Experiment:** monkeypatch `pipeline.process` to raise after N frames, or pause the camera plugin, and watch `/cmd_vel` and odometry.

**Narrowest correction (proposal):** publish a zero `Twist` in the exception and `None` branches, and add a timer that zeros the command when no frame has been processed for more than about 0.3 s.

---

### F-09 (Low): The "consecutive frames" requirement of the spike guard is not enforced

**Status:** statically demonstrated.

**Code path.** The low-evidence and width-only branch (`temporal_filter.py:435-450`) does not reset or advance `_pending_raw` or `_pending_count`. The `held_no_line` branch keeps them as well (`:321`). So a large candidate interrupted by small-innovation, low-evidence frames keeps its count.

**Repro**

```python
tf = TemporalNavigationFilter(TemporalFilterParams(n_strips=10))
def feed(x, n2): return tf.update({"raw_bottom_x": x, "raw_theta": 0.0, "raw_width": 140.0,
    "has_line": True, "n_two_sided": n2, "n_q_accepted": n2, "n_q_rejected": 0, "weed_pressure": 0.0}, dt=0.05)
feed(320, 9)
for k in range(7):
    o = feed(420, 9); print(k, o["status"], o["pending_count"])
    if k < 6: feed(322, 0)      # low-evidence, small innovation: does not break the streak
# Expected per docstring: never "pending_accepted"; hand-derived actual: pending_accepted at k=6
```

**Narrowest correction (proposal):** reset `_pending_raw` and `_pending_count` in the low-evidence branch, or document that the count is cumulative.

---

### F-10 (Low): Spline "robust refit" has no effect

**Status:** statically demonstrated.

**Code path.** In `_fit_nav_curve`, `spline_at` closes over the full `ys` and `xs` arrays (`test_multi_roi.py:807-812`). The loop at `:815-824` pops indices from `keep` but never refits on them. The sampled curve (`:826-827`) and the base tangent (`:857-858`) use all points. Only the envelope reference line (`:846`) uses `keep`.

**Consequence:** outliers that survive the total-least-squares stage still shape the curve and its bottom tangent (the steering Theta). The envelope (15% of the median span) bounds the deviation.

**Repro (output equals the plain all-points spline)**

```python
from scipy.interpolate import LSQUnivariateSpline
det = MultiROIDetector()
ys = np.arange(60, 380, 40.0); xs = np.full_like(ys, 300.0); xs[4] = 380.0
samples, _ = det._fit_nav_curve(list(zip(xs, ys)), 462, env_cap=1e9)
n_int = max(0, min(3, (len(ys) - 3) // 2)); t = np.linspace(ys[0], ys[-1], n_int + 2)[1:-1]
full = LSQUnivariateSpline(ys, xs, t, k=3)
yy = np.arange(0, 462, 4.0); m = (yy >= ys[0]) & (yy <= ys[-1])
print(np.max(np.abs(np.array([s[0] for s in samples])[m] - full(yy[m]))))   # expected 0.0
```

**Narrowest correction (proposal):** make `spline_at` fit `ys[keep]` and `xs[keep]` (recomputing `n_int` from `len(keep)`).

---

### F-11 (Low): Lookahead uses strip 1 as its primary lateral measurement, the strip the detector excludes

**Status:** likely.

**Code path**

- The navigation gate excludes strip 1 because "its full-width view predates any corridor knowledge" (`test_multi_roi.py:594-610`, `:243-248`).
- The map's bottom measurement is `strip_profile[eff_bottom_idx=0]` (`lookahead_corridor_map.py:299-301`, `:327-356`), and in `no_map` or `map_accept` it becomes `corrected_bottom_x`.
- With the default `init_window_frac=1.0`, a strip-1 pick anchored on outer rows (the documented photo_6 class) becomes the control `X` directly whenever the map has no confident prior, which includes every stream start.

**Experiment:** run `--video` on a repeated photo_6-class frame (or a synthetic image with four rows offset from centre) and compare the `raw_err_x` and `filt_err_x` CSV columns.

**Narrowest correction (proposal):** use the lowest accepted navigation strip (or the extrapolated raw `P`) as the bottom measurement, or default `ignore_initial` to 1 for the map.

---

### F-12 (Low): After `map_switch`, the bottom bins are not refreshed

**Status:** requires experiment.

**Code path**

- In `map_switch`, the effective-bottom bins fall to the `else` branch at `lookahead_corridor_map.py:543-590`. `supports_new_here` is true, but `(i+1) >= upper_start` is false. `cons` is false because the bottom conflicted.
- `pred.confidence > 0.4`, so the old predicted geometry is kept (`:579-584`).
- `_switch_frames` resets (`:455-456`), so the next frame needs a full persistence cycle again. The comment at `:454` ("map will be updated to new") does not hold for the bottom bins.

**Experiment:** feed a +60 px synthetic lane shift through `process_image` with lookahead and count frames until `corrected_bottom_x` reaches the new centre, compared with a variant that also updates the bottom bins on a switch.

**Narrowest correction (proposal):** on `map_switch`, update the bottom bins toward the raw value with `map_alpha`.

---

### F-13 (Low): Controller normalisation width is not synchronised with the frame width

**Status:** statically demonstrated.

**Code path.** `run_video` and `MultiROINavigationStack.step` set the filter and map `image_width` and `image_height` to the real frame size on frame 0 (`run_mr_navigation.py:655-662`, `mr_navigation_stack.py:195-200`), but `MRVSParams.width` is never updated. `compute_control` normalises `err_x / p.width` (`mr_vs.py:367`), which also sets the heading-gate width.

**Consequence:** a 1280-wide video doubles the effective lateral gain and halves the gate. `docs/usage.md:71` tells users to pass `--width`, so the contract is documented; the inconsistency is that the filter and map auto-correct while the controller does not.

**Narrowest correction (proposal):** set `vs.params.width` and `vs.params.height` from the first frame alongside the filter and map, or warn when they differ.

---

### F-14 (Low): Rover reset does not reset navigation-node state

**Status:** likely.

**Code path.** `rover_reset.Respawn.respawn` deletes and respawns the model and publishes a single zero `Twist` (`rover_reset.py:142-170`). `nav_node.py` has no subscription to `/reset_rover`. The following persist across a respawn:
- the temporal filter, the `vs` integrator and smoother;
- `_lap_angle` and `_lap_prev`;
- `lane_index`, `drive_dir`, `phase`, `lanes_done`.

The docs and docstrings (`rover_reset.py:9-12`, `farm.launch.py:263-266`) say reset works "in every mode, including autonomous".

**Consequence:** after a mid-run reset, the filter treats the jump as a large innovation (F-07 hold applies), and row-change or lap bookkeeping is wrong.

**Narrowest correction (proposal):** subscribe `nav_node` to `/reset_rover` and call `_reset_perception()` plus a reset of the lap and row-change state.

---

### F-15 (Low): Navigation midpoints are tagged at the strip top, not the strip centre

**Status:** statically demonstrated.

**Code path.** `q.append((mo_x, my))` with `my = h - mu*dh = y1`, the top row of strip mu (`test_multi_roi.py:547`, `:558`). The measurement comes from rows `[y1, y1+dh)`. The detection chains use `cy = y1 + dh/2` (`:573`) and `strip_profile` uses `y_center` (`:716`).

**Consequence:** the navigation line and curve are shifted up by `dh/2` relative to the evidence. At the base this gives a lateral bias of `(dx/dy)*dh/2`, about 2.4 px for an 8 degree lean with `dh = 34`. The paper's Eq. 10 may define it this way, so this could be intentional fidelity to the paper.

**Narrowest correction (proposal):** use `y1 + dh/2` for `my`, and update the `accepted_nav` comparison at `:733` to match.

---

### F-16 (Low): Off-by-one in `pre_corridor_clean.strip_of`

**Status:** statically demonstrated.

**Code path.** `strip_of(y) = min(n, max(1, (h - int(y)) // dh))` (`test_multi_roi.py:1057-1058`). For y inside strip mu (with mu ≥ 2), `(h - y)` lies in `((mu-1)*dh, mu*dh]`, which floors to `mu - 1`.

Example: `h=462`, `dh=46`, `y = h - dh - 1 = 415` (strip 2) gives `47 // 46 = 1`.

**Consequence:** the `{mu-1, mu, mu+1}` tolerance set becomes `{actual-2, actual-1, actual}`. The component's own strip is still included, so the effect on cleaning is small and asymmetric.

**Narrowest correction (proposal):** `(h - int(y) - 1) // dh + 1`.

---

### F-17 (Low): Double `_stop_robot` and spinning after `destroy_node`

**Status:** requires experiment.

**Code path.** `_stop_robot` destroys the node inside a callback (`nav_node.py:859-867`). `rclpy.spin` keeps running on a destroyed node, and `main`'s `finally` calls `_stop_robot` again (`:1085`), publishing on a destroyed publisher. Whether the process exits (which the launch file relies on to shut down via `OnProcessExit`) is not determined statically.

**Narrowest correction (proposal):** set a `_stopped` flag, make `_stop_robot` idempotent, and call `rclpy.shutdown()` or raise `SystemExit` to leave `spin`.

---

### F-18 (Low): Duplicated append in the detector CLI summary

**Status:** statically demonstrated.

`test_multi_roi.py:1669-1672` appends to `deltas` twice when the scrub is applied, so the printed count is doubled. The mean is unchanged.

**Correction (proposal):** delete the duplicate block.

## 5. Test coverage gaps and proposed order of investigation

### Coverage gaps (from reading the tests)

- `pyproject.toml:49` collects only `test_temporal_filter.py`.
  - None of the tests construct `LookaheadCorridorMap`, call `process_image` with a map, or use `MultiROINavigationStack`.
  - So F-01, F-02, F-03, F-11 and F-12 have no test exposure.
- No test checks the **sign** of steering.
  - `test_curved` (`test_temporal_filter.py:361-384`) asserts only `|w| > 0.5 deg/s` and low jitter.
  - `test_width_expansion` (`:213`) asserts a status in `("pending", "pending_accepted", "accepted")`, which is nearly vacuous.
  - `test_missing_plants` (`:133-139`) chooses its assertion conditionally.
- Coordinate-frame consistency (cropped vs full) is untested apart from drawing clips.
- No test covers a no-vegetation or black frame (F-04), an `n_strips`/`n_bins` mismatch (F-03), or the `_fit_nav_curve` outlier handling (F-10).
- The tests use `MultiROIDetector()` with its default `morph="paper"`, while the navigation CLI uses `struct`.
- **CI** checks only artifact counts (`scripts/check_example_outputs.py`). Its video corridor is symmetric (heading close to zero), so heading-sign and crop-offset errors cannot show up.
- **Simulation logic** has no offline tests. `nav_node.py` imports `rclpy` at module level, so the pure helpers (`_pipe_locked`, `_primary_empty`, `_base_err_from_res`, `_shuttle_end_triggered`) cannot be unit-tested without ROS.
- **Timing:** every test passes `dt=0.05` explicitly, so the wall-clock paths (F-06) are never exercised.

### Proposed order of investigation (proposals only)

1. **F-01 and F-02:** run the two snippets, and check `filt_err_x_px` against `raw_err_x_px` in the existing CI `corridor_nav.csv`. Cheap, and decisive for the default video path.
2. **F-03:** run the CLI command with `--n_strips 8`. One command.
3. **F-05:** run the `"has_line" in info` snippet, then one short Gazebo run with `--turn-mode shuttle` to confirm the 70% leg end.
4. **F-04:** run the blank-frame snippet. Then decide the policy question (Q3 below).
5. **F-06:** run the timing snippet. Then a Gazebo run at a reduced real-time factor, comparing `w` traces.
6. **F-07 and F-09:** run the filter-only snippets.
7. **F-10 and F-15:** run the snippets. Low priority.
8. **F-08, F-14, F-17:** simulation experiments with injected faults and resets.
9. **F-11 and F-12:** targeted video experiments, only if the lookahead path is kept.

Proposed tests to add once fixes are authorised:
- a sign-agreement test between the raw and lookahead paths on a slanted synthetic image;
- a frame-consistency test (centred corridor gives |X| < 2 px in both paths);
- a test that a blank frame yields `has_line=False`;
- an `n_bins`/`n_strips` mismatch test;
- extracting the `nav_node` decision helpers into a ROS-free module so they can be tested.

## 6. Questions for the coordinating reviewer

1. **F-01 severity.** `docs/usage.md:55-56` says the offline CLI does "not publish ROS commands or drive hardware", and the simulation adapter does not use the lookahead map. A reasonable counterargument is that F-01's present impact is limited to wrong telemetry and analysis, which would make it Medium. I kept it at High because it is the documented default for video and camera input, it is the default of the public `MultiROINavigationStack` class, and the commented ROS wrapper in `run_mr_navigation.py:862-890` invites integration. Do you want severity tied to current actuation paths only?
2. **F-01 interpretation.** Is there any intended "x-per-y" heading convention in the lookahead map that a downstream gain sign was meant to compensate? I found no compensation: the same `compute_control` serves both paths. If anyone tuned the lookahead mode empirically, it would have been tuned against an inverted heading term, which would also invalidate earlier video-mode comparisons.
3. **F-04 policy.** `docs/python-api.md:43-44` acknowledges that "a returned line ... is not sufficient to establish valid crop evidence", and `v_min_scale=0.45` looks deliberately chosen to avoid stalls. Is a no-evidence stop (or timeout) intended to live outside this stack, for example in a hardware supervisor? If so, F-04 narrows to "documented `held_no_line` path unreachable; `has_line` misreported", which is Low to Medium.
4. **F-05 history.** The shuttle and fishtail comments say rear steering was "verified in sim". Do the recorded simulation results (for example `docs/assets/navigation-demo.mp4`) predate the current `info` dict layout? If so, those claims do not apply to the current code.
5. **F-07 tradeoff.** The evidence floor of 7 is documented. Do you consider an unbounded hold acceptable by design, or should the contract include a bounded hold with explicit degradation?
6. **Configuration divergence.** The simulation (paper morphology, no lookahead, `lambda_x` 2.0, `w_max` 0.6) and the video CLI (struct morphology, lookahead on, `lambda_x` 10.0, `w_max` 0.5) are documented as different. I did not list this as a defect. Should the "validated" simulation gains be treated as evidence for the CLI path at all, given F-01 and F-02 only exist there?
7. **F-15** may be faithful to the paper's Eq. 10 (`My(mu) = My(mu-1) - dh`). Do you want paper fidelity or internal consistency with the detection-chain y convention?
8. **Repository contents.** There is an `audit/README.md` in the tree that I did not open, to keep this review independent. You may want to reconcile it with these findings separately.