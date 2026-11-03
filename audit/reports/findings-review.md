# Consolidated findings for review

This is the pre-clarification assessment. The [design review](design-intent-review.md)
supersedes its general interpretation of holds, gap handling and recovery.

Review date: 3 November 2026  
Source revision: `d5c68594e8e633b4569a4a59f8c60f33ac785cf0`  
Status: findings discussion; implementation unchanged

## Review outcome

Yesterday's three Critical findings remain reproducible: reversed lookahead
heading, a 12 px crop-coordinate bias at 640 px width, and an indefinitely
pending temporal filter when support cannot reach seven strips. The reports
differ in classification and coverage. No runtime correction explains the
change in severity counts.

The independent first pass reported 1 High, 6 Medium and 11 Low findings.
After targeted reproduction and code review, the first supplement reported
4 High, 5 Medium and 11 Low findings, including two additional control defects.
Those counts describe stages of the review, not a conclusion that yesterday's
findings disappeared. The [prior-audit comparison](prior-audit-comparison.md)
records the final reconciliation: 6 High, 6 Medium and 14 Low defects, plus
two policy items and one maintainability item.

Seventeen diagnostic cases reproduced their recorded observations. These
include algorithm failures, interface failures, configuration differences,
and partial simulations of failure paths. They are not seventeen full robot
tests. The inherited eight tests also passed, demonstrating that the current
regression suite does not cover these failures.

## Recommended first priorities

The final comparison and coordinating review rate the following six root
causes High. High means a supported execution path can produce materially incorrect
control, lose steering, retain stale geometry indefinitely, or bypass failure
stopping. This recommendation does not depend on a physical collision being
demonstrated. The earlier classifications remain visible in the stage reports. Opus raised
the filter lockout and failure-stop findings after the comparison.

| ID | Failure | Reproduced observation | Scope |
| --- | --- | --- | --- |
| A-01 | Vertical TLS line intercept has the wrong sign | Centred two-row frame produces lateral error -617 px, `v=0.20 m/s`, `w=0.60 rad/s` | Supported line mode; depends on the SVD direction sign. Sparse spline fallback is statically reachable |
| A-02 | Rate limiter and deadband lock steering at zero | 100 updates at `dt=0.01` or `0.005 s` remain at zero while raw demand is `-0.3125 rad/s` | Smoothed controller paths, starting from zero, with default rate/deadband parameters |
| F-01 | Lookahead heading convention is reversed | About `+7.8` degrees without lookahead becomes `-8.0` degrees with it; raw steering changes sign | Default video/camera and public stack lookahead paths; ROS adapter disables lookahead |
| F-04 | No vegetation is reported as a navigation line | Zero two-sided strips, ten accepted points, `has_line=True`, forward command about `0.103 m/s` | Local pipeline execution with temporal filtering; whole-robot response not measured |
| F-07 | Evidence floor can prevent reacquisition indefinitely | After 200 persistent shifted observations, output remains at 320 px; `ignore_initial=4` yields only six two-sided strips even on the clean test frame | Filter state sequence and detector evidence limit reproduced; closed-loop trajectory not measured |
| F-08 | Perception exceptions bypass stopping | Three injected exceptions produce no new command and no stop call, despite elapsed-time and lane-end limits being exceeded | Actual callback body with stub ROS objects; command persistence in a live Gazebo plugin remains untested |

The 12 px lateral bias F-02 is confirmed and should be corrected with F-01 as
one coordinate-convention work package. The coordinating review considers the
bounded bias Medium on the current evidence, rather than retaining yesterday's
Critical label. That does not make the output correct or close the finding.

No Critical rating is assigned here without an explicit operational impact
criterion. Yesterday's Critical ratings remain recorded in the comparison.
A default-only rule is not used to lower the implementation priority of a
supported mode such as line fitting.

## Additional reproduced observations

Final ratings and the crosswalk appear in the comparison report. P-08a and
P-09 are Low; P-08b and P-07 are Medium. The grid-spacing difference is
measured, while its command impact has not been quantified.

| ID | Observation | Interpretation |
| --- | --- | --- |
| F-02 | Centred corridor: +1 px without map, -11 px with map | Cropped and full-image horizontal coordinates are mixed |
| F-03 | Eight detector strips and ten map bins raise `IndexError` on frame two | Reachable configuration crash |
| F-05 | `info` lacks `has_line` while filtered output contains it | Interface break used by rear-camera lock checks; full manoeuvre not run |
| F-06 | Identical features give different commands under injected host-clock intervals | Time-base dependence; discarded simulation `dt` is established by source tracing |
| F-09 | Pending candidate survives intervening low-evidence or missing observations | Confirmation is not strictly consecutive |
| F-10 | Two indices are removed, but the spline still equals the all-points fit | Robust refit does not use its retained subset |
| P-06 | CLI lateral gain 10, library default 2; angular caps 0.5 and 0.6 | Proven configuration divergence; stability difference and intended preset need review |
| P-07 | Observed centre 200 px is predicted at 246.7 px beside an empty bin | Zero-confidence neighbour biases interpolation and causes repeated rejection |
| P-08a | Valid proxy strip is accepted, but missing-bottom case remains inconsistent | Proxy selection does not refresh the bottom-consistency decision |
| P-08b | Seven upper strips agree with the effective bottom, but `support_new=0` | Support is measured against ignored strip zero |
| P-09 | Detector strip spacing 34 px, map spacing 36 px | Cropped detector and full-image map grids differ |
| P-10 | One detector strip raises `UnboundLocalError` for `nav_line` | Unvalidated boundary configuration |

P-08a and P-08b share one diagnostic case, P08. JSON case keys omit hyphens,
for example F01 corresponds to report finding F-01.

## Source findings and policy questions

The independent report also records lookahead use of the excluded bottom
strip (F-11), bottom-bin refresh after switching (F-12), frame-width
normalisation (F-13), reset state persistence (F-14), midpoint height (F-15),
strip-index arithmetic (F-16), shutdown lifecycle (F-17), and duplicated
summary counts (F-18). Their evidence states remain source analysis or
unexecuted hypotheses as specified in the reports.

Yesterday's overlay-coordinate concern is P-11. The stale/NaN range policy
is P-12; it is an intentional policy that needs an operational decision,
not a newly demonstrated arithmetic defect. Detector placement in
`test_multi_roi.py` is P-13, a packaging and maintainability observation.
The comparison report accounts for each earlier item.

## Evidence and provenance

- [Independent source audit](independent-audit.md)
- [Reproduction reconciliation](evidence-reconciliation.md)
- [Comparison with yesterday's audit](prior-audit-comparison.md)
- [Reproduction instructions](../reproductions/README.md)
- [Recorded results](../evidence/runtime-results.json)
- [Environment and existing test result](../evidence/environment.json)
- [Baseline file hashes](../baseline-files.json)
- [Review execution record](../review-run.json)

The independent audit started at the official baseline on the local
`audit/independent-review` branch. While it ran, the investigation checkout
received README and baseline-record additions on `main`. All 46 recorded
implementation and configuration files still match the source baseline.
The official checkout remains at that revision and was not edited.

The reports are preserved as separate review stages. A later correction to a
rating or claim takes precedence over the earlier stage; preserving the
first pass does not imply every initial inference has been verified.

## Decisions before implementation

1. Agree on the order for the six control priorities and coordinate corrections.
2. Set the required behaviour for lost perception, stale geometry and camera stalls.
3. Choose the intended CLI/library gain presets and supported strip-count range.
4. Use source timestamps for the time-base correction and define reset behaviour.
5. Select ROS/Gazebo scenarios for the rear-camera, watchdog and shutdown findings.

No bug fixes are included in this review. Implementation follows the findings
discussion in this investigation repository.
