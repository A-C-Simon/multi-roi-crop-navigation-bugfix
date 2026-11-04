# Multi-ROI Crop Navigation: Bug Investigation

This repository is the separate audit and bug-investigation workspace for
[Multi-ROI Crop Navigation](https://github.com/A-C-Simon/multi-roi-crop-navigation).
The official repository remains the published project. Investigation reports,
reproductions, and subsequent corrections belong here.

## Current stage

Fixes implemented and validated in this repository, not yet published.
Confirmed defects from the audit have been corrected in this repository while keeping the corridor-tracking design:
strict holds against unsupported sharp changes, lookahead memory guiding the
search windows, and command smoothing. Start with the
[implementation report](audit/implementation/REPORT.md), which lists each
root cause, the design choices, the exact validation commands and results,
the simulation evidence, and the open policy questions.

The review history remains available: the
[design review](audit/reports/design-intent-review.md), the
[Opus reassessment](audit/reports/design-intent-reassessment.md), the
[prior-audit comparison](audit/reports/prior-audit-comparison.md) and the
[reproduction instructions](audit/reproductions/README.md). Baseline evidence
under `audit/evidence` describes the unmodified baseline and is not updated.

Validation covers 108 passing tests, the CLI examples, and headless Gazebo
runs (straight, curved and crop-gap fields with memory off and on, line mode,
and injected perception failure and camera stall). The curved-field error and
row-end behaviour remain open; see the report. No physical robot has been
tested. The official repository is unchanged.

## Baseline

| Item | Reference |
| --- | --- |
| Official repository | `A-C-Simon/multi-roi-crop-navigation` |
| Starting revision | `d5c68594e8e633b4569a4a59f8c60f33ac785cf0` |
| Software version | `0.1.0` |
| Workflow | Independent audit, reproduction, findings review, then corrections |
| Corrections | Recorded in `audit/implementation/change-manifest.json` against the snapshot |

The source history and implementation files are retained from the official
project. See [the baseline record](audit/BASELINE.md) and
[the audit workspace](audit/README.md).

## Working rules

- Keep investigation work in this repository.
- Record each finding with severity, trigger, affected code, and evidence.
- Distinguish source analysis from an executed reproduction.
- Review the consolidated findings before applying corrections.
- Keep any later changes small enough to assess independently.

The official repository is configured as the fetch-only `upstream` remote in
the local investigation checkout. Its push URL is disabled. The `origin`
remote points to this repository.

## Development environment

Use a virtual environment dedicated to this checkout:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
```

The inherited detector and navigation commands remain available. See the
[official technical reference](https://github.com/A-C-Simon/multi-roi-crop-navigation/blob/main/docs/technical-documentation.md)
for the implementation baseline and interfaces.
