# Addendum: reassessment against the owner's control-design intent

**Scope.** Baseline runtime `d5c6859`, unchanged. This is a read-only re-reading of the code; I ran nothing. Every reproduced result cited below was produced by the coordinating reviewer. The earlier reports in `audit/reports/` are treated as revisable.

**Classification labels used below:**
- **I**: intentional behaviour that serves the stated goal.
- **E**: implementation error, either proven from the code or reproduced.
- **P**: missing policy or requirement that the owner needs to decide.
- **U**: untested architectural concern that needs sequence or simulation evidence.

## 1. The failure loop being prevented, and what the code does about it

**The loop the owner described:** missing plants shift a strip centre → the fitted line moves → the steering command changes → the rover turns → the next frame's search regions are placed from the turned view → the detector locks onto a neighbouring corridor → the error reinforces itself.

The intended fix is **corridor identity continuity**: commit only to physically plausible change, and never let one noisy view decide which corridor the rover follows.

### Comments that already state this intent

| Location | What it says |
| --- | --- |
| `temporal_filter.py:9-11` | "Trust the past enough to reject transient anomalies, but allow persistent, coherent, multi-frame evidence to override the past." |
| `temporal_filter.py:30-46`, `:101-124` | The spike guard holds strictly, with no drift toward the input. The two-sided evidence floor exists specifically so a remembered map slope cannot be committed through an occlusion. |
| `lookahead_corridor_map.py:5-7`, `:17-18` | Bottom-strip conflicts with a confident remembered corridor are held, and are only accepted with support from upper strips. |
| `test_multi_roi.py:460-468` | A single (one-sided) cluster must not move the midpoint or the search window. |
| `test_multi_roi.py:394-397`, `:470-473` | Predicted bins seed the first strip's search window. A sudden jump against the prediction is treated as a gap in the plants, and the window and midpoint are held. |
| `mr_vs.py:83-90` | Rate limiting and low-pass filtering of the turn command exist "to prevent chatter". |

### What each layer actually implements

| Layer | Mechanism | Where it runs |
| --- | --- | --- |
| Within one frame | Only strips that see both flanking rows renew the window. Also: a cluster-mass gate, trimming of merged clusters, a corridor-width gate (at most 1.6 × median), rejection of up to 3 outlier points in the line fit, and a curve-deviation envelope. | All paths |
| Across frames: search-window memory | First strip's window and midpoint seeded from predicted bin 0. Strips that disagree with the prediction are suppressed: their window and midpoint are held, and they are left out of `measured` and `n_two` (`:501-505`, `:594`, `:704`). | **Only when the lookahead map is passed in.** Video/camera command-line mode and `MultiROINavigationStack` do this; the simulation pipeline passes `lookahead_map=None` (`pipeline.py:220`). |
| Across frames: corridor memory | Image-space bins, shifted down by a fixed 14 px per frame. Gates on centre and width. Counts of upper strips supporting the old geometry (`support_old`) and the new geometry (`support_new`). A switch needs 4 frames of persistence. | Map path only |
| Across frames: command-state memory | Innovation gates. Strict hold on large changes. Commit only after 7 coherent frames **and** at least 7 two-sided strips. Exponential smoothing otherwise. Slow following when evidence is weak. | All temporal paths, including simulation |
| Actuation | Low-pass (0.35), rate limit (1.2 rad/s²), deadband, speed scaled by confidence. | All smoothed paths |

### Observations

- **Without the map, nothing carries the search windows from one frame to the next.** Every frame starts again from the image centre with a full-width first strip (`:364-370`). The simulation therefore tests the command-hold and actuation layers, but **not** the window-memory or corridor-memory layers. I cannot tell whether that is an intended comparison baseline or a mismatch with the target design. See question Q1.
- **Gradual curvature passes by design.** The filter's prediction holds position constant. At a hypothetical image shift of 7.5 px per frame (about 0.3 rad/s at 20 Hz with roughly 500 px focal length; my estimate), the smoothing lag is about 14 px, well inside the 64 px gate.
- **Gradual hijack also passes.** A drift slow enough to stay inside the gate every frame is accepted. No layer checks whether a slow drift is consistent with the rover's own motion.
- **Nothing uses the rover's own motion.** The map accepts `dt` and `last_w` but never uses them (`lookahead_corridor_map.py:209-210`). It stores `age` but never uses it in a decision. The 14 px shift is fixed, while the stack itself varies forward speed between 0.45 and 1.0 of the target. The filter's motion gains default to 0 (`temporal_filter.py:139-140`). Whether this is adequate for a moving, turning rover is **U**: it needs recorded or simulated sequences, and is not a confirmed failure.

## 2. Reassessed findings

### A-01: vertical line written on the wrong side of the image (`test_multi_roi.py:917-918`)

| | |
| --- | --- |
| Class | **E** |
| Proven | An arithmetic error: the intercept ignores the sign of the slope. Reproduced: -617 px for a centred straight corridor in line mode. |
| Still needs evidence | Whether the sparse-curve and curve-envelope paths trigger it on other linear-algebra backends. |
| Relation to intent | The anti-jitter goal does not cover this. Worse, the identical-x fallback (F-04 case c) feeds exactly this vertical branch. |

### A-02: steering locks at zero (`mr_vs.py:435-446`)

| | |
| --- | --- |
| Class | Primary lock: **E**. Secondary from-rest threshold: **I/P**. |
| Proven | When `max_w_rate*dt < w_deadband` (dt < 16.7 ms), the stored deadbanded zero keeps the output at zero for **every** demand. Reproduced at dt = 0.01 and 0.005. |
| Relation to intent | This is a deadlock, not a rejection of sharp turns: a steady, physically plausible turn is never issued at all. The secondary effect, where demands below about 0.057 rad/s are zeroed from rest at any rate, may be acceptable as an anti-chatter measure; that is a tuning decision. |

### F-01: heading sign inverted in the map path

| | |
| --- | --- |
| Class | **E** |
| Proven | The map's heading is the opposite sign to the controller's convention. Reproduced: +7.8 vs -8.0 degrees, and the heading term of `w` flips. The map's internal gating stays self-consistent, because the raw and predicted headings share the inverted convention; only its output to the controller is wrong. |
| Relation to intent | This is the component that implements memory, so the intended stabiliser outputs a heading pointing the wrong way. The controller's own convention (`mr_vs.py:292-295`, `:410`) is correct. |
| Still needs evidence | My earlier damping argument for what this does in closed loop is a hypothesis (**U**). |

### F-02: 12 px coordinate-frame bias

| | |
| --- | --- |
| Class | **E**, bounded |
| Proven | Map outputs are in cropped-image coordinates but are compared against the full-image centre. Reproduced: +1 px without the map vs -11 px with it. |
| Relation to intent | Not related. Memory continuity is unaffected; only the absolute reference is offset. |

### F-04: what happens when no crops are seen

There are three cases, and they need to be kept apart:

| Case | When it happens | What the code does | Class |
| --- | --- | --- | --- |
| (a) Fresh observation | At least 2 strips seen on both sides, not suppressed, and within the width gate | The line is fitted to real evidence. | **I**, correct |
| (b) Predicted track | Map path, with bin 0 confidence ≥ 0.40 | The midpoint is seeded from the predicted centre `pc`. The fallback copies every strip midpoint (`q`) from that seed, so the line sits on the remembered position but is forced **vertical**, losing the remembered heading. In this path the controller uses the map's corrected values, not this line. | Intent served. **E** in labelling only: it is reported as `has_line=True` with 10 accepted points, so confidence gains about 0.35 credit (`temporal_filter.py:204-206`) from points nobody observed. |
| (c) No-prior synthetic fallback | Simulation, folder mode, `--no-lookahead`, or a low-confidence map | Every midpoint is the image centre. The filter treats this as a weak measurement (raw confidence about 0.25) and follows it at 8% per frame (`temporal_filter.py:435-450`). | **E**: invalid evidence provenance. |

**Why case (c) works against the stated intent:** during a full temporary occlusion, the held corridor is pulled toward the image centre (about 81% of the way after 20 frames) instead of being held. The filter path that *would* hold it, the "no line" branch at `:308-328`, cannot be reached because the detector always reports a line.

**On the very first frame**, case (c) is accepted directly as the initial state (`:286-301`). The real corridor that appears afterwards is then gated as a large innovation (see F-07).

**What is not a defect:** keeping the rover moving through a short gap on a remembered track (**I**). Whether to stop or slow down after a lost-track period is a policy question for the owner (**P**).

### F-07: hold on large changes

| Sub-issue | Class | Evidence |
| --- | --- | --- |
| Holding against an abrupt +100 px / +20° input | **I** | This is exactly the hijack the owner describes. My earlier test called a single +100 px step a "genuine shift"; it is not a physically plausible single-frame change. |
| Commit made impossible by evidence counting | **E** | Commit needs `n_two_sided ≥ 7` (`:124`, `:399-401`). The detector can never reach 7 when `n_strips - ignore_initial < 7`: reproduced at `ignore_initial=4` (at most 6) and at `n_strips=5`. In those supported configurations, the documented recovery condition (`pending_accepted`) cannot be reached. |
| Map suppression lowers the evidence count | **U** | Suppressed strips are excluded from `n_two`. With the map on, strips showing a changed corridor reduce the very count the filter needs to commit. The two persistence stages run in series (map 4 frames, filter 7 frames); the combined latency and lockout have not been measured. |
| No recovery that accounts for the rover's own motion | **U/P** | Prediction holds position constant, and `last_w` is ignored. An image shift caused by the rover's own steering looks the same as a hijack. A held image-space error then drives the controller with no feedback. |
| Initial wrong identity | **U** | Case (c) on the first frame, plus a sparse field, could make reacquisition unreachable. Not reproduced end to end. |

**Withdrawn:** my earlier suggestion that a held change should eventually commit with reduced confidence after a time limit. That would recreate the owner's failure. Any recovery has to come from evidence that is physically consistent with motion (gradual, motion-compensated, or supported by several strips), or from an explicit "lost" state. Which one is a policy choice for the owner.

### F-08: no stop on failure in simulation

| | |
| --- | --- |
| Class | Missing fail-safe behaviour: **P**. Termination checks being skipped: **E**. |
| Proven | The early returns at `nav_node.py:955-968` skip both publishing and the `max_seconds` and lane-end checks (`:1046-1069`). |
| Still needs evidence | Whether Gazebo's diff-drive plugin keeps applying the last command has not been tested. |
| Relation to intent | Not related to anti-jitter. |

### P-07: interpolation with an empty neighbour (`lookahead_corridor_map.py:168-183`)

| | |
| --- | --- |
| Class | **E** |
| Proven | The code contradicts its own comment ("only interpolate if both have reasonable confidence"). A bin with no confidence at the default centre pulls the prediction for its neighbour 39% of the way toward it. Reproduced: predicted 246.7 for an observed 200, with repeated rejection. |
| Relation to intent | This works against continuity: at the edge of what the map remembers, the memory rejects the real corridor because its prediction is polluted. |
| Still needs evidence | The effect on the turn command near the end of a row needs a sequence. |

### F-09: pending evidence retained

| | |
| --- | --- |
| Class | Retention across frames with no line: **I**. The comment at `:321` explicitly keeps it, and retention can no longer be reached anyway (F-04). Not resetting the candidate on frames that contradict it: **E**, minor. No age limit on a candidate: **P**. |
| Proven | Frames that agree with the held state are evidence *against* the pending candidate, yet they do not reset it. The docstring says "consecutive coherent frames". Reproduced: interleaved frames still commit. |
| Reworded | I now classify committing a 100-frame-old candidate after a single observation as a missing age requirement, not a bug. |

### F-10: curve outlier refit does nothing (`test_multi_roi.py:807-824`)

| | |
| --- | --- |
| Class | **E** (dead logic) |
| Proven | Reproduced: 2 of 8 points were removed from `keep`, yet the curve was identical to the all-points fit. |
| Relation to intent | Directly relevant: it was meant to stop a single midpoint shifted by missing plants from bending the path. The line fit's own 3-point rejection and the deviation envelope limit the effect. |

### P-08: substituting the effective bottom strip

| | |
| --- | --- |
| Class | P-08a: **E**, conservative in effect. P-08b: **E**, invalid evidence provenance. |
| P-08a proven | The proxy strip replaces the missing bottom strip (comment at `:346`), but its consistency flag is still read from the missing strip (`:328`). A valid proxy is treated as a conflict and the prediction is held. Reproduced. The output stays within the gate, but the bottom bins stop taking in fresh evidence. |
| P-08b proven | `support_new` is measured against strip 0 even when `ignore_initial` has declared strip 0 unreliable (`:299-301`, `:316-324`). The later bin update uses the effective bottom (`:546-551`), so the function contradicts itself. Reproduced: `support_new=0` while 7 strips matched. The path to accepting a new geometry is blocked. |
| Still needs evidence | How long the slow bin-by-bin convergence takes. |

## 3. Acceptance scenarios

### Observables to log per frame

**Ground truth and estimates:**
- **True** corridor centre and heading, from the synthetic generator or the simulation world.
- **Measured** midpoints, each tagged with its provenance: two-sided, suppressed, one-sided, or fallback.
- **Predicted**: the map's predicted bins and the filter state.
- **Output**: the corrected values passed to the controller.

**Errors and command behaviour:**
- Lateral drift: true minus output in px; in simulation, also the rover's distance from the furrow centreline in metres.
- Steering acceleration (|Δw|/Δt) against the 1.2 rad/s² limit.
- Command latency.

**Identity and memory:**
- Search-window continuity: change in the first strip's window centre, and a corridor identity index (which furrow is being tracked).
- Recovery time.
- Memory age: frames since the last real observation, per bin and for the filter.

### Scenarios

**L** = provable with local unit sequences. **S** = needs simulation or recorded field sequences.

| # | Scenario | Pass condition (proposed, open to discussion) | Proof |
| --- | --- | --- | --- |
| 1 | Clean travel, straight and gently curving | Output within a few px of truth. Steering acceleration within limit. No identity change. | L for numbers; S for drift |
| 2 | One frame with the window falsely displaced | Output and window centre unchanged beyond noise. The false geometry leaves no trace. | L |
| 3a | Persistent false displacement, e.g. a weed patch, over many frames | No commit while the displacement is not supported across strips. Identity unchanged. | L |
| 3b | Persistent real, gradual shift consistent with motion | Tracked within a stated lag. Commit reachable in every allowed configuration. | L for reachability; S for lag under motion |
| 4 | Plants missing on one side | Window and midpoint held. Bounded drift. Provenance shows one-sided. | L |
| 5 | Full temporary occlusion with a prior track | Output stays near the predicted track, not the image centre. Memory age grows. Output marked as predicted, not observed. When the crops reappear, resume without the identity jumping. | L for hold and labelling; S for travel distance |
| 6 | First frame blank | No initial state created from synthetic points. The first real observation initialises the track. | L |
| 7 | Rover's own motion: lateral offset and yaw while driving | The predicted image position follows the motion, so steady turns are not flagged as conflicts. | S (fixed 14 px shift and no yaw compensation, **U**) |
| 8 | Non-default strip or ignore counts | Every documented recovery state is reachable, or the configuration is rejected at start-up. | L |
| 9 | Steady commanded turn at 20, 60 and 100 Hz | Output reaches the demand within the time implied by the rate limit, at every update rate. Acceleration within limit. Never stuck at zero. | L |
| 10 | Started on the wrong corridor, then reacquiring | A defined, reachable rule for regaining the correct corridor, with no cross-field drift. | L for the state machine; S for whether the decision is correct in closed loop |

**What local sequences can prove:** sign and coordinate-frame conventions, provenance labels, reachability of each state, independence from update rate, hold behaviour, and interpolation.

**What needs simulation or field sequences:** whether the motion model is adequate, closed-loop drift through gaps and turns, reacquisition, F-08 persistence, and timing under real processing load.

## 4. Changes to the earlier audit

### Withdrawn or reworded

- **F-07:** the hold on abrupt change is intended. The time-limited commit proposal is withdrawn. What remains is the configurations where commit is impossible (**E**) and the absence of motion-aware recovery (**U/P**).
- **F-04:** travelling a short gap on a remembered track is intended. What remains is the provenance and labelling problem, and the drift toward the image centre when there is no prior. The stop/slow-down question becomes a policy question for the owner.
- **F-09:** retention across frames with no line is reclassified as intentional. Only the failure to reset on contradicting frames remains, plus a policy question about candidate age.
- **A-02:** the from-rest threshold is reworded as a tuning interaction. The lock at fast update rates remains a defect.
- **F-12** (bottom bins not refreshed after a map switch): conservative and plausibly intended. It is kept only as a contradiction with the code comment at `:454`.
- **F-11** (map using the first strip as its bottom measurement): mitigated by the memory gates. Now a **U** concern only.
- **P-06** (gains differ between command-line and library defaults): unequal presets are not a defect. Whether gain 10 aggravates jitter is an open question.

### Definite defects regardless of the anti-jitter intent

| ID | Defect |
| --- | --- |
| A-01 | Intercept arithmetic for vertical lines |
| A-02 | Steering deadlock at fast update rates |
| F-01 | Heading sign inverted in the map path |
| F-02 | Coordinate frames mixed in map output |
| F-03 | Crash on strip/bin count mismatch |
| F-04 (a/b/c) | Synthetic fallback reported as observed evidence |
| F-07 | Commit impossible in supported configurations |
| P-07 | Interpolation contradicts its own comment |
| P-08a, P-08b | Inconsistent effective-bottom evidence |
| F-10 | Dead outlier-refit logic |
| F-06 | Simulation time step discarded, so the rate limit is not applied in physical time |
| F-08 | Termination checks skipped after a failure |
| F-05 | Missing interface field |
| P-10 | Crash with a single strip |

### Questions for the owner

| # | Question |
| --- | --- |
| Q1 | Is the simulation's map-disabled configuration an intended baseline, or should simulation represent the target design with window and corridor memory? |
| Q2 | What should happen when the track is lost: hold, slow down, or stop? And after how much memory age? |
| Q3 | Which evidence is allowed to reacquire the corridor after a hold: motion-consistent gradual change, agreement across several strips, or an explicit operator or re-initialisation event? |
| Q4 | Should the rover's own motion (its commanded turn rate and speed, `w` and `v`) drive the map shift and the filter prediction? |
| Q5 | What update rates must be supported? |

No corrections are proposed for implementation until these are discussed.