# Python interface

Install the project before importing it. Modules retain their original names.
Images must be nonempty OpenCV BGR arrays. Use one detector, filter, and
controller per continuous camera stream.

## Detect rows in an image

```python
import cv2
from test_multi_roi import MultiROIDetector, make_composite

image = cv2.imread("examples/data/straight.png")
if image is None:
    raise FileNotFoundError("examples/data/straight.png")

detector = MultiROIDetector(morph="struct", vertical_coverage=0.75)
result = detector.detect(image)
cv2.imwrite("outputs/detector.png", make_composite(image, result))
print(result["n_two_sided"], result["median_width"])
```

Create `outputs` before running this snippet. The image CLI creates its output
directory automatically.

`MultiROIDetector()` defaults to `morph="paper"`. Pass `morph="struct"`
explicitly to match the extended detector CLI.

Selected result fields:

| Field | Meaning |
| --- | --- |
| `binary` | Segmented mask in cropped image coordinates |
| `crop_offset` | Pixel offset from cropped to full image coordinates |
| `nav_line` | Line represented as `y = slope*x + intercept`, or `None` without an observed line |
| `nav_source` | `observed`, `prior` (lookahead-seeded, no observed line) or `none` |
| `nav_curve` | Sampled `(x, y)` spline points, or `None` |
| `q_accepted`, `q_rejected` | Points assigned to the fit and rejected points |
| `n_two_sided` | Number of measured strips counted by the detector |
| `median_width` | Median measured flank span in pixels |
| `strip_profile` | Per-strip geometry and evidence flags |
| `time_ms` | Detector processing time in milliseconds |

A returned line or accepted point count is not sufficient to establish valid
crop evidence. Review the image and two-sided evidence as well.

## Process a navigation frame

```python
import cv2
from test_multi_roi import MultiROIDetector
from mr_vs import MultiROIVS, MRVSParams
from temporal_filter import TemporalNavigationFilter, TemporalFilterParams
from run_mr_navigation import process_image

image = cv2.imread("examples/data/straight.png")
if image is None:
    raise FileNotFoundError("examples/data/straight.png")
h, w = image.shape[:2]
detector = MultiROIDetector(morph="struct")
controller = MultiROIVS(MRVSParams(width=w, height=h))
state_filter = TemporalNavigationFilter(
    TemporalFilterParams(image_width=w, image_height=h)
)
output = process_image(
    image, detector, controller, t_filter=state_filter,
    dt=0.05, last_w=0.0, draw=True,
)
print(output["v"], output["w"], output["info"])
```

Here `dt` is the interval between source frames in seconds.
`output["info"]["has_observation"]` reports a fresh observed line;
`has_track` (also exposed as `has_line`) reports that a track is available,
including a held or remembered one during short gaps. On later frames,
pass the previous returned angular command as `last_w`. A temporal filter
is stateful. Call `state_filter.reset()` and `controller.reset_smoother()`
when a stream ends or the camera jumps to an unrelated scene.

This example omits lookahead. To inspect it, instantiate
`LookaheadCorridorMap` from `lookahead_corridor_map`, set its image geometry,
and pass it as `lookahead_map`. See the existing runner for all parameters.
The `v` and `w` values are experimental commands, not a hardware interface.
