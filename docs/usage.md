# Usage

Run these commands from the repository root with the offline environment active.
Always supply an input path. The examples assume the installed console commands;
`python test_multi_roi.py`, `python run_mr_navigation.py`, and
`python multi_roi.py` provide the same respective interfaces from a checkout.

## Image detection

```bash
multiroi-detect --input /path/to/images --results_dir outputs/detection
multiroi-detect --input '/path/to/images/*.png' --results_dir outputs/pngs
```

Quote globs so the application receives the pattern. Directory mode selects lowercase `.png` and `.jpg` files in the detector,
and also `.jpeg` in the navigation runner. For other OpenCV-readable formats
or uppercase extensions, pass an explicit glob. By default the detector limits
the longest side to 1920 pixels. Set `--max_side 0` to retain the original size.

```bash
multiroi-detect --input examples/data --results_dir outputs/full-size --max_side 0
```

The detector has a historical rotation rule for filenames beginning with
`bev5`, `bev6`, or `bev7`. Disable that rule for your own dataset:

```bash
multiroi-detect --input /path/to/images --results_dir outputs/field --rotate_names ''
```

## Navigation analysis on images

```bash
multiroi-nav --input /path/to/images --output outputs/navigation
```

Folder mode processes each image independently unless temporal or lookahead
options are explicitly enabled. Use it for independent photographs. Enabling
memory across unrelated photographs does not form a valid video sequence.

## Video and camera

```bash
multiroi-nav --input examples/data/corridor.mp4 --video --output outputs/video
multiroi-nav --input /path/to/field.mp4 --video --output outputs/field-video --show
multiroi-nav --input 0 --video --output outputs/camera --show
```

Video mode enables temporal filtering and lookahead by default. Camera inputs
use an integer device index. Press `q` in the preview or use Ctrl-C to stop.
Camera runs write CSV telemetry but do not record an output video.

Use `--loop` to repeat a video file. State is reset at each rewind.
Use `--composite-only` to write diagnostic composites without the navigation
side-by-side view. These commands write analysis files; they do not publish
ROS commands or drive hardware.

## Useful options

| Option | Purpose |
| --- | --- |
| `--line` | Straight navigation fit instead of the spline, in navigation mode |
| `--no-temporal` | Disable temporal filtering for a video comparison |
| `--no-lookahead` | Disable lookahead memory for a comparison |
| `--vertical-coverage 0.75` | Restrict strip coverage to the bottom 75% |
| `--ignore-initial N` | Exclude N bottom strips from navigation evidence |
| `--width W --height H` | Fix the controller normalization geometry (default: the actual frame size) |
| `--vf V` | Reference forward speed in metres per second |
| `--wmax W` | Angular command limit in radians per second |

Without `--width`/`--height`, controller normalization follows the processed
frame size. Explicit values are kept and a mismatch with the frame is reported.
`--lookahead-bins` must equal `--n_strips` (it defaults to it). Video files use
their frame timestamps as the control interval; the CSV records it as `dt_s`
together with `has_observation` and `has_track`.
The full list and actual command defaults are available with `--help`.
Some library defaults differ from the command-line defaults; see
[the Python interface](python-api.md).

## Detector variants and reference implementation

```bash
multiroi-reference --input examples/data --results_dir outputs/reference
multiroi-detect --input examples/data --results_dir outputs/paper-morph --morph paper
multiroi-detect --input examples/data --results_dir outputs/madz --post_scrub --scorer madz
```

The extended detector CLI selects `struct` morphology. The reference command
runs the original reimplementation, with its existing defaults. Neither is a
claim of exact numerical reproduction of the published article.

Post-scrubbing and oriented morphology are experimental options. Historical
comparisons are kept in [the experiment archive](history/README.md).
