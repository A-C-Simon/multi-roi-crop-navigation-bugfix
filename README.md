# Multi-ROI Crop Navigation: Bug Investigation

This repository is the separate audit and bug-investigation workspace for
[Multi-ROI Crop Navigation](https://github.com/A-C-Simon/multi-roi-crop-navigation).
The official repository remains the published project. Investigation reports,
reproductions, and subsequent corrections belong here.

## Current stage

Independent source review and findings assessment. No runtime corrections
have been applied. Findings will be reviewed before implementation begins.

## Baseline

| Item | Reference |
| --- | --- |
| Official repository | `A-C-Simon/multi-roi-crop-navigation` |
| Starting revision | `d5c68594e8e633b4569a4a59f8c60f33ac785cf0` |
| Software version | `0.1.0` |
| Workflow | Independent audit, reproduction, findings review, then corrections |

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
