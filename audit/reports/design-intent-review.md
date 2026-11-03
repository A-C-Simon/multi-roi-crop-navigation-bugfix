# Review against the corridor-tracking design

Review date: 3 November 2026  
Implementation baseline: `d5c68594e8e633b4569a4a59f8c60f33ac785cf0`  
Status: findings discussion; no runtime changes

This review supersedes the general interpretation of holds, gap handling and
recovery in the earlier [findings review](findings-review.md). Earlier reports
remain part of the review history. Their aggregate severity counts are not a
substitute for assessing each behaviour against the intended control design.

## Intended behaviour

The rover must keep following the same furrow despite missing plants and noisy
strip detections. A bad midpoint can change the navigation line, induce a turn,
and change the next frame's search windows. The resulting view can capture a
neighbouring corridor and reinforce the original error.

The intended protections address different parts of that feedback loop:

1. Renew strip windows only from credible flanking-row evidence.
2. Use remembered geometry from further ahead to constrain the detector's
   search windows and resist a displaced near-field measurement.
3. Hold the established navigation state against unsupported sharp changes.
4. Smooth angular commands so a brief detection error cannot cause a sharp turn.

These mechanisms are purposeful. Existing comments in the detector, map,
filter and controller already explain much of this reasoning. Missing comments
are not the reason the initial interpretation was incomplete.

## Reassessment

| Finding | Current assessment |
| --- | --- |
| A-01: vertical TLS intercept | Definite arithmetic defect. Mirroring a centred line off-image cannot serve noise rejection |
| A-02: steering remains zero at fast updates | Definite state-update defect. A substantial steady command must eventually pass through smoothing; indefinite zero at every demand is different from damping sharp commands |
| F-01: heading convention | Definite interface error. Memory and controller must use the same heading convention |
| F-02: crop offset | Definite coordinate error. Intentional holds do not explain a constant 12 px reference bias |
| F-04: continuation through missing crops | Travelling briefly on an established prediction can be correct. The remaining defect is treating fallback points as observation support, and allowing an unobserved centre fallback to pull the track away from its prior position |
| F-07: large-change hold | Rejection of a persistent false jump is intended. A held +100 px input alone does not prove failure. The definite configuration problem is that supported strip/ignore settings can make the seven-strip commit condition unreachable |
| F-08: callback failure | Skipping the existing time and lane-end termination checks is a code defect. Behaviour on a camera stall and the required watchdog action remain operational decisions |
| F-09: pending evidence | Keeping a hypothesis through missing observations can be intentional. Contradicting observations and candidate age need separate treatment; blindly clearing every gap is not required |
| F-10: curve refit | Definite ineffective outlier rejection: the curve fit ignores its retained subset. This directly weakens the intended protection against bad midpoints |
| P-07: interpolation beside empty memory | Definite prediction error: an unobserved default centre pulls a valid remembered bin sideways |
| P-08a/b: effective bottom | Inconsistent proxy consistency and support selection remain code defects. Their control impact needs sequence evaluation |
| P-06: different tuning presets | Configuration decision, not proof of a defect or instability |

The strict-hold mechanism should not be replaced by automatic acceptance after
a timeout. A persistent false track would then regain control simply by lasting
long enough. Recovery needs credible corridor evidence, a prediction consistent
with rover motion, or a defined lost-track/reinitialisation procedure.

## Local sequence evidence

Six additional local sequences were executed against the unchanged source.
They examine both useful protection and remaining defects.

| Sequence | Observed result |
| --- | --- |
| Three-frame false displacement | Filtered centre and heading do not move; correct observations resume normally |
| Persistent unsupported false track | After 100 inputs, the original 320 px / 0 degree state is retained. This is useful rejection when the candidate is false |
| Gradual supported change | A centre moving from 320 to 370 px and heading from 0 to 10 degrees is followed to 369.07 px and 9.81 degrees |
| False local displacement with consistent lookahead | Bottom measurement moves from 200 to 300 px, but corrected output remains 200 px across all 12 frames because seven upper strips support the old corridor |
| Substantial sustained steering demand | At 20 and 50 Hz the output reaches -0.3125 rad/s. At 100 Hz it stays zero while forward output remains 0.2 m/s |
| Full occlusion after an established off-centre track | Without lookahead, filtered centre moves from 281 to 312.64 px after 20 blank frames. With lookahead, it moves from 269 to 286.39 px. Both report zero two-sided strips, ten accepted points and `has_line=True` |

The occlusion sequence used repeated stationary images followed by blank images.
It proves a change in the local estimate without new crop evidence. It does not
measure a rover trajectory, or establish how far a moving rover can travel on
memory. The map's crop-coordinate bias explains why the two runs begin at
different reported positions. The map reduces but does not eliminate the
estimate's drift in this sequence.

[Sequence results](../evidence/design-intent-results.json) and
[reproduction code](../reproductions/design_intent_checks.py) are retained.
The [Opus design reassessment](design-intent-reassessment.md) is a separate
read-only analysis; the coordinating reviewer executed these new sequences.

## Execution-path and motion questions

The historical execution path is uncertain. The project owner initially recalled
ROS/Gazebo, then clarified that they could not remember which path produced the
reported jitter. No historical run is attributed to a particular adapter here.

The current ROS adapter passes `lookahead_map=None` to `process_image`. Runs
through that adapter use temporal filtering and command smoothing, but not the
map's search-window seeding or corridor memory. Every detector frame starts
again without a map prior. Whether this is an intentional comparison mode or
should represent the complete design needs agreement before any change.

The map currently shifts image-space memory by 14 px per frame by default. It
accepts `dt` and `last_w` without using them, and the filter's motion gains are
zero by default. A remembered corridor needs to be interpreted in the new view
as the rover moves. The field rows may be smooth while their image coordinates
change because of camera motion. Adequacy of the present approximation is an
architectural question for recorded sequences or simulation, not a newly proven
closed-loop failure.

## Review and implementation direction

Preserve the anti-jitter design. First isolate arithmetic, coordinate and
state-update corrections without weakening its gates. Evaluate recovery and
memory through sequences with a known true furrow, controlled missing plants,
false local detections, and rover motion.

Acceptance should measure whether the same furrow is retained, lateral drift,
steering acceleration, prediction age and recovery time. A smooth plotted line
alone cannot establish correct tracking: an estimate can be smooth and wrong.

Decisions still needed are the intended execution path, behaviour when memory
is no longer reliable, acceptable reacquisition evidence, supported update rates,
and how motion should inform prediction. No implementation changes are included.
