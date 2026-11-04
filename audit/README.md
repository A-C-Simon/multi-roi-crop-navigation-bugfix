# Audit record

The current stage is fixes implemented and validated (local tests and headless
Gazebo; no physical robot). See the
[implementation report](implementation/REPORT.md) for the corrections, tests,
simulation evidence and remaining decisions. The reports below are the review
history; their classifications describe the unmodified baseline.

Earlier stage: the review reassessed findings against the owner's
corridor-tracking and anti-jitter design. Read the
[current design review](reports/design-intent-review.md) and
[Opus reassessment](reports/design-intent-reassessment.md) for that analysis.

## Findings and evidence

| Record | Purpose |
| --- | --- |
| [Consolidated findings](reports/findings-review.md) | Current assessment, six first priorities, evidence and implementation decisions |
| [Independent audit](reports/independent-audit.md) | First source review, before earlier findings or local results were supplied |
| [Evidence reconciliation](reports/evidence-reconciliation.md) | Review of local reproductions and two additional control defects |
| [Prior-audit comparison](reports/prior-audit-comparison.md) | Every earlier item matched to current findings, with final classifications |
| [Reproduction instructions](reproductions/README.md) | How to rerun the 17 diagnostic cases |
| [Runtime results](evidence/runtime-results.json) | Recorded observations from the local runs |
| [Environment](evidence/environment.json) | Dependency versions, baseline checks and inherited test result |
| [Execution record](review-run.json) | Requested and returned model, effort, tools and report hashes |
| [Baseline](BASELINE.md) | Official revision and repository separation |
| [Implementation report](implementation/REPORT.md) | Corrections, validation commands and results, simulation evidence, open decisions |
| [Change manifest](implementation/change-manifest.json) | Approved changes relative to the source snapshot |
| [Post-fix validation](implementation/validation/) | Machine-readable post-fix results; baseline evidence is unchanged |

The earlier comparison reported 6 High, 6 Medium and 14 Low defects, with
policy and maintainability items separate. The subsequent design review
rewords holds, gap handling and recovery, and retains those counts as historical.
It does not assign a new aggregate count to intentional behaviour and open
requirements.

## Review sequence

1. Independent read-only source audit with Claude Opus 5.5 at high effort.
2. Targeted detector, map, filter, controller and callback reproductions.
3. Evidence reconciliation with the same model and effort.
4. Comparison against the earlier audit supplied by the project owner.
5. Findings discussion, followed by a separate implementation stage.

Seventeen diagnostic cases reproduced the recorded observations. Some exercise
an interface or stubbed callback rather than a whole simulator. Each result
states its scope. No ROS/Gazebo or physical robot run was performed in this review.

At the review stage all 46 recorded implementation and configuration hashes
matched the official baseline. The implementation stage changes some of them;
the change manifest lists which. Original source-snapshot metadata and historical results remain
under `docs` as inherited project material.

Six additional [design sequences](evidence/design-intent-results.json) examine
noise rejection, gradual tracking, memory protection, sustained steering and
full temporary occlusion. See the current design review for their scope.
