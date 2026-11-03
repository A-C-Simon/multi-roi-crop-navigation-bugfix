# Addendum: comparison with yesterday's audit

I re-read only the code each newly raised item depends on: `lookahead_corridor_map.py:136-205` and `:296-356`, `mr_navigation_stack.py:373-390`, `test_multi_roi.py:594-643`, `nav_node.py:940-1069`, and `tof_guard.py:112-150` and `:257-286`.

The coordinating reviewer ran every reproduction cited here. I ran nothing. Nobody has run a closed-loop Gazebo test or a physical test for any item.

## 0. One severity rubric for both reports

**Categories:**
- **Defect:** the code contradicts its own code comment or documentation, or produces wrong numbers or decisions.
- **Policy:** the behaviour is intentional and documented; the open question is whether that choice is right.
- **Maintainability:** structure only, with no effect on runtime behaviour.

**Actuating path:** a path that sends commands to something that moves. Only the Gazebo adapter does that today. The offline command-line tool and `MultiROINavigationStack` produce commands but do not drive anything.

**Technical severity (applies to defects only):**

| Severity | Definition |
| --- | --- |
| Critical | All of these: a default or designated primary configuration; a commonly met input; a grossly wrong command (wrong direction, saturated, or no stop that never ends); an actuating path; and nothing in the stack that limits it. |
| High | A confirmed control error that is wrong in direction or never ends, in a default path or a documented supported mode. Or loss of stop behaviour on an actuating path. |
| Medium | A bounded or slowly self-correcting control error, a crash, or a wrong decision in a reachable non-default configuration. Or a component error whose effect on the final command still needs an experiment to measure. |
| Low | Small bounded bias, narrow triggers, display-only errors, diagnostics. |

Under this rubric no item reaches Critical. Each High falls short on at least one condition:
- A-01 needs line mode.
- A-02 needs updates faster than 60 Hz.
- F-01 is default only on paths that do not drive anything.
- F-04 slows the robot rather than sending a grossly wrong command.
- F-07 needs too few two-sided strips plus an abrupt large change.
- F-08 needs a failure trigger.

## 1. Crosswalk

| # | Yesterday's severity | Item | Maps to | Final severity | Why it differs, if it does |
| --- | --- | --- | --- | --- | --- |
| 1 | Critical | Heading reversed with lookahead | F-01 | High | Same defect, confirmed both times (+14.3 → -14.0 degrees; +7.8 → -8.0 degrees). It is default only in the command-line tool and the stack class, which do not drive anything. The Gazebo adapter does not use the lookahead map. It would be Critical if you designate the video or camera path as a control path. |
| 2 | Critical | -12 px crop-frame bias | F-02 | Medium | The bias is constant and bounded (about 2% of image width, about 9% of corridor width) and never reverses direction. In closed loop it would give a steady offset, not divergence. |
| 3 | Critical | Permanent freeze at n_two_sided=6 | F-07 | **High** (I had Medium) | New evidence: `ignore_initial=4` limits two-sided strips to 6 (`test_multi_roi.py:594`, the `measured` list skips ignored strips). So freezing is guaranteed whenever `n_strips - ignore_initial < 7`, through a documented option. The frozen state drives the controller (`filt_F` goes into `compute_control`). Not Critical: default settings need sparse evidence, the command stays bounded in size, and trajectory effects are untested. |
| 4 | High | No stop on exception or camera stall | F-08 | **High** (I had Low) | See section 2. |
| 5 | High | Simulation-time dt thrown away | F-06 | Medium | The clock dependence is confirmed. Its effect on the rate limit and integrator is bounded and depends on timing. Its most serious consequence, steering locking at zero, is rated separately as A-02, so it is not counted twice. |
| 6 | High | Command-line gains differ from library gains | **P-06** | Policy (Low if treated as a defect) | The difference is proven: lateral gain 10.0 vs 2.0 and angular limit 0.50 vs 0.60 (`run_mr_navigation.py:734-735`, `mr_vs.py:57`, `:76`). It is documented (technical documentation section 10.1). The comment "was 10, too aggressive" conflicts with the command-line default. Stability with gain 10 was not measured, and the command-line tool does not drive anything. |
| 7 | High | Interpolation pulls predictions toward a zero-confidence neighbour | **P-07** | Medium | Confirmed below. The error is in the map's prediction. Its effect on the final command has not been measured. |
| 8 | Medium | Proxy bottom strip and ignored-strip support count | **P-08a / P-08b** | P-08a Low, P-08b Medium | Two separate flaws in the same function (section 1.2). This is separate from F-11, which is about using strip 1 at all. |
| 9 | Medium | Map strip grid differs from detector grid | **P-09** | Low | Bins are matched to strips by index, so the mismatch only changes how the per-frame shift converts to strips (34 vs 36 px, about 6%) and where boxes are drawn. |
| 10 | Medium | `strip_of` off by one | F-16 | Low | The component's own strip is still in the tolerance set, so only the tolerance's asymmetry changes. No new evidence. |
| 11 | Low | Duplicate `deltas.append` | F-18 | Low | Same finding. |
| 12 | Low | `n_strips=1` crash (`UnboundLocalError`) | **P-10** | Low | Confirmed: `nav_line` is only assigned inside `if len(nav_pts) >= 2` (`test_multi_roi.py:622-623`) and is read at `:643`. A one-strip configuration is degenerate and should be rejected when parameters are read. |
| 13 | Low | Map boxes in `mr_navigation_stack` ignore coverage and crop offset | **P-11** | Low | `mr_navigation_stack.py:376` uses `dh = h_o / n_bins` (no coverage) and draws cropped-frame x values on the full image. Display only. |
| 14 | Low | Time-of-flight guard treats stale or NaN readings as clear | **P-12** | Policy | Intentional and documented as fail-open (`tof_guard.py:112-113`, `:259-262`). Related to F-08: the guard only republishes when a command arrives (`:257-286`), so it cannot stop a rover whose navigation node has gone silent. |
| 15 | Low | Main detector lives in `test_multi_roi.py` | **P-13** | Maintainability | `pyproject.toml:49` restricts pytest's `testpaths`, so default runs do not import it. The architecture document acknowledges the naming. |

**My findings that are not in yesterday's list:** A-01, A-02, F-03, F-04, F-05, F-09, F-10, F-11, F-12, F-13, F-14, F-15, F-17.

### 1.1 P-07: interpolation contradicts its own comment

`lookahead_corridor_map.py:168` says "only interpolate if both have reasonable confidence; else take higher". The code at `:169-183` only falls back when **both** bins have confidence below 0.05. Otherwise it blends the centres by position, ignoring confidence.

Bins that have never been observed, or that have decayed, hold the default centre `image_width/2` with confidence 0 (`:150-152`, `:170-172`). Note that this default is the full-image centre, which also mixes coordinate frames (the F-02 problem).

With the default shift of 14/36 ≈ 0.39 of a strip, any observed bin whose neighbour above is empty gets a prediction pulled 39% of the way toward 320 px. The reproduction showed:

- observed centre 200;
- predicted centre 246.7 with confidence 0.296;
- that confidence sits between `min_conf_to_update` (0.15) and `min_conf_for_hold` (0.40), so the per-strip gate rejects the observation;
- the bin then resets to the raw value at confidence 0.5 (`:586-590`);
- the same rejection repeats every frame.

At a row end, the top observed strip moves down toward the bottom strip. Once the bottom strip has no observed neighbour, `map_pending` would output `0.92*pred + 0.08*raw` (`:458-467`), feeding a centre-biased value into control. That last step follows from the code but has not been reproduced.

**Narrowest fix:** weight the blend by confidence, or take the confident bin when the other bin's confidence is below the floor.

### 1.2 P-08a and P-08b: the effective-bottom substitution is applied inconsistently

- **P-08a (Low).** `bottom_consistent` is read from the missing strip at `:328`, before the proxy substitution at `:340-350` clears `bottom_missing`. A proxy strip that agrees with the map is therefore handled as a conflict (`map_hold` or `map_pending`, `:430-469`).
  - The reproduction showed `map_hold` while the proxy strip's own status was `map_accept`.
  - The held prediction falls within the gate of a consistent proxy, so the output error is bounded.
  - Confidence and speed drop, and the bottom bins stop taking in fresh data (`:525-531`).
- **P-08b (Medium).** `support_new` is counted against strip 0 (`:299-301`, `:316-324`) before `bottom_raw_c` is reassigned to the effective bottom (`:347-356`). The later bin update at `:546-551` does use the effective bottom, so the function is inconsistent with itself.
  - With `ignore_initial > 0` (a documented option), the reproduction showed `support_new = 0` even though 7 upper strips matched the effective bottom. Status stayed `map_pending`, whose ambiguous branch resets `_switch_frames` (`:462-469`).
  - A real lane shift can then only be absorbed slowly through the 8% and 15% bin updates. How long that takes has not been measured.

## 2. F-08 severity correction, and what actually changed between reports

**F-08 moves from Low to High.** Two facts I under-weighted in my first pass:

1. The early returns at `nav_node.py:955-962` (decode failure) and `:964-968` (pipeline exception) skip not only publishing but also **every termination check**: `max_seconds`, lane end and the frame cap (`:1046-1069`).
2. When the camera stalls, no callback runs at all. The node has no timer, and the time-of-flight guard only acts when a command arrives (P-12).

So after a failure, the last `Twist` command keeps running. Within ROS, nothing bounds it; the only limit is `run_sim.sh`'s outer `timeout` (600 s by default), and a direct `ros2 launch` has none.

On an actuating path, losing stop behaviour with no time limit meets High.

Two conditions remain:
- That Gazebo's diff-drive plugin keeps applying the last command is likely but not verified (no live watchdog experiment yet). If it turns out to time out on its own, this drops to Medium.
- No realistic trigger that makes the pipeline throw has been shown. A camera stall is plausible; the code itself mentions CPU starvation.

**What changed between the reports:** no runtime code. All 46 baseline hashes match and the source is still `d5c6859`. The differences come from three places:

- **Evidence:**
  - the reviewer's `ignore_initial` result makes F-07 guaranteed in a documented configuration;
  - re-reading `nav_node.py` showed that the termination checks are also skipped (F-08);
  - P-07 to P-10 were reproduced.
- **Classification:**
  - this rubric scores actuating and non-actuating paths explicitly, and treats "never ends" as distinct from "bounded";
  - that explicit rubric raised F-07 and F-08 and kept F-01 and F-02 below yesterday's ratings;
  - configuration divergence (P-06) and the guard's fail-open choice (P-12) are scored as policy, not as bugs.
- **Coverage:**
  - each audit found root causes the other missed: P-07 to P-11 from yesterday's audit; A-01, A-02, F-03 to F-05, F-09 to F-15 and F-17 from mine.

## 3. Final priority list

**Confirmed malfunctions** (reproduced by the reviewer, or certain from the code), in priority order:

| Rank | ID | Severity | What is wrong |
| --- | --- | --- | --- |
| 1 | A-01 | High | Line mode encodes a perfectly centred straight corridor as a lateral error of -617 px, giving a full-rate turn. The sparse-spline fallback reaches the same code but was not reproduced. |
| 2 | A-02 | High | The smoother locks steering at zero when updates arrive faster than every 16.7 ms. |
| 3 | F-01 | High | The heading sign is reversed whenever lookahead is on. |
| 4 | F-04 | High | When no crops are visible, the stack invents a centred line and keeps driving with no stop. |
| 5 | F-07 | High | The filter freezes indefinitely when fewer than 7 strips are two-sided. This is guaranteed when `n_strips - ignore_initial < 7`. |
| 6 | F-06 | Medium | Controller timing follows the host clock. This is also the main way A-02's condition is reached in default paths. |
| 7 | F-02 | Medium | The lookahead path has a constant 12 px crop-frame bias. |
| 8 | P-07 | Medium | Map predictions are pulled toward image centre whenever a neighbouring bin is empty. |
| 9 | F-03 | Medium | Video mode crashes on frame 2 when the strip count and bin count differ. |
| 10 | P-08b | Medium | Support for a new bottom geometry is counted against an ignored strip. |
| 11 | F-09, F-10, F-15, F-16, F-13, P-08a, P-09, P-10, P-11, F-18 | Low | Bounded, display-only, or narrow-trigger issues. |

**Simulation hypotheses.** The code-level fact is confirmed, but the effect in Gazebo has not been measured:
- F-08 (High): persistent motion after a failure.
- F-05 (Medium): rear-camera logic never activates.
- F-14 (Low): navigation state survives a rover reset.
- F-17 (Low): stop routine runs twice on a destroyed node.

**Requires experiment:** F-11 (strip 1 used by lookahead, Low) and F-12 (bottom bins not refreshed after a switch, Low).

**Policy:** P-06 (command-line vs library gains), P-12 (guard fails open).
**Maintainability:** P-13 (detector module name).

**Totals:**
- 26 distinct defect root causes: 6 High, 6 Medium, 14 Low, no Critical.
- 2 policy items and 1 maintainability item.
- 29 items in total, with yesterday's 15 all included.

## 4. Decisions needed before any fixes

1. **Which paths count as control paths?** If the video/camera command-line path or `MultiROINavigationStack` is meant to steer something, F-01 meets Critical. If line mode is a primary simulation configuration, A-01 meets Critical.
2. **Fail-safe policy.** Should the simulation node have a watchdog that publishes a zero command on any failure or stall, with termination checks moved to a timer (F-08)? Should the time-of-flight guard fail closed when its sensors go stale (P-12)? Should lost evidence or an unresolved freeze end in a stop (F-04, F-07)?
3. **Commit floor for the filter (F-07).** Scale it as a fraction of the strips actually usable (`n_strips - ignore_initial`), or reject configurations where a commit is impossible.
4. **Gain defaults (P-06).** Align the command-line defaults with the values validated in simulation, or record why they differ, and measure stability at gain 10 before it is used anywhere that drives.
5. **Lookahead default.** Leave it disabled by default until F-01, F-02, P-07 and P-08 are fixed and tested in closed loop?
6. **Timing contract (F-06, A-02).** Make the source timestamp mandatory, and decide the camera rates that must be supported.
7. **Order of work.** I propose failing regression tests for the confirmed malfunctions first, then a Gazebo watchdog experiment for F-08 and a check of A-01's sign on more than one linear-algebra backend, then the narrow fixes. This is a proposal only; nothing has been changed or committed.