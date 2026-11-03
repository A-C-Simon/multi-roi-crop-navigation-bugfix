# Baseline reproductions

Run from the investigation repository root using its Python environment:

```bash
env -u PYTHONPATH -u PYTHONHOME python3 audit/reproductions/runtime_checks.py \
  --repo . --output /tmp/multiroi-audit-runtime-results.json
```

This diagnostic harness checks whether the recorded defects remain reproducible
at the investigation baseline. It is separate from the inherited regression
suite. An exit status of zero means all recorded observations were reproduced;
it does not mean the implementation is correct. A failed observation needs
review before it is interpreted as a correction.

The script reads the implementation and writes only the selected JSON output.
It generates local synthetic arrays in memory to isolate geometry, timing and
state transitions. It does not replace the project's photographic examples.

## Recorded scope

| Cases | Method |
| --- | --- |
| F01, F02, F04, A01 | Synthetic image through detector and navigation pipeline |
| F03 | Detector/map strip-count mismatch over two frames |
| F05 | Pipeline result interface inspection |
| F06 | Deterministic host-clock injection into controller |
| F07, F09 | Filter state sequences, plus detector evidence-count check |
| F08 | Unchanged callback body executed with stub ROS objects and an injected pipeline exception |
| F10 | Spline output comparison and tracing of rejected point indices |
| A02 | Repeated controller updates with explicit time intervals |
| P06 | CLI parser defaults compared with library defaults |
| P07, P08 | Map profiles for partial visibility and effective-bottom handling |
| P09 | Detector and map strip-grid spacing measurement |
| P10 | Single-strip detector invocation |

The record contains 17 cases. P08 covers two related bottom-selection defects.
Evidence fields state where an interface or numerical failure was exercised
without a complete ROS or Gazebo run. No physical robot was used.

See [results](../evidence/runtime-results.json),
[environment](../evidence/environment.json), and the
[consolidated review](../reports/findings-review.md).
