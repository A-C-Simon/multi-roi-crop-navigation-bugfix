# Audit workspace

This directory contains the source-review record and evidence for the separate
bug-investigation repository.

## Review sequence

1. Obtain an independent audit of the imported implementation.
2. Group findings by root cause and assess their severity and confidence.
3. Run targeted reproductions in the investigation environment.
4. Present confirmed findings and unresolved questions for discussion.
5. Apply corrections only after the findings review.

The initial review is read-only. Runtime source files remain at the recorded
baseline during the audit and reproduction stage.

## Evidence conventions

Each finding should identify the current file and function, the triggering
input or configuration, expected and observed behavior, and its verification
status. Severity describes the impact under stated conditions. A source-only
finding is not recorded as an executed test.

The [baseline record](BASELINE.md) identifies the official source revision.
Original source-snapshot metadata and historical result files remain available
under `docs` as inherited project material.
