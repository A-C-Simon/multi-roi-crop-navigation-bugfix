"""Summarize a mrsim run: ground-truth cross-track from the world's plant
poses, command statistics, provenance counts and how the run ended."""
import csv, json, re, sys
from pathlib import Path
import numpy as np

run = Path(sys.argv[1]); world = Path(sys.argv[2])
rows = list(csv.DictReader(open(run / "nav_run.csv")))
txt = world.read_text()
poses = np.array([list(map(float, m.groups())) for m in
                  re.finditer(r"<model name='pl_\d+'>\s*<static>1</static>\s*<pose>([-\d.e]+) ([-\d.e]+)", txt)])
grid = np.arange(poses[:, 0].min(), poses[:, 0].max(), 0.1)
cent = []
for gx in grid:
    ys = np.sort(poses[np.abs(poses[:, 0] - gx) < 0.35, 1])
    if len(ys) < 2:
        cent.append(np.nan); continue
    k = int(np.argmax(np.diff(ys)))
    g1, g2 = ys[:k + 1], ys[k + 1:]
    cent.append((g1.mean() + g2.mean()) / 2 if np.diff(ys)[k] > 0.6 else np.nan)
cent = np.array(cent); ok = np.isfinite(cent)
def centre(x): return float(np.interp(x, grid[ok], cent[ok]))
good = [r for r in rows if r["status"] != "perception_error"]
t = np.array([float(r["sim_t"]) for r in good]); x = np.array([float(r["odom_x"]) for r in good])
y = np.array([float(r["odom_y"]) for r in good]); w = np.array([float(r["w"]) for r in good])
ct = np.array([yy - centre(xx) for xx, yy in zip(x, y)])
dw = np.diff(w) / np.maximum(np.diff(t), 1e-3)
log = (run / "launch.log").read_text() if (run / "launch.log").exists() else ""
stop = re.findall(r"stop: ([^\n]+)", log)
out = {
    "run": run.name, "world": world.name, "frames": len(rows),
    "sim_seconds": round(float(t[-1] - t[0]), 2) if len(t) else 0,
    "x_start": round(float(x[0]), 2), "x_end": round(float(x[-1]), 2),
    "cross_track_m": {"mean_abs": round(float(np.mean(np.abs(ct))), 4),
                      "rms": round(float(np.sqrt(np.mean(ct ** 2))), 4),
                      "max_abs": round(float(np.max(np.abs(ct))), 4),
                      "final": round(float(ct[-1]), 4)},
    "in_row_x_le_7_cross_track_m": {"mean_abs": round(float(np.mean(np.abs(ct[x <= 7.0]))), 4),
                                     "max_abs": round(float(np.max(np.abs(ct[x <= 7.0]))), 4)},
    "row_end_x_gt_7_held_frames": int(sum(1 for r, xx in zip(good, x) if xx > 7.0 and r["status"] in ("held_no_line", "held"))),
    "w_radps": {"mean_abs": round(float(np.mean(np.abs(w))), 4), "max_abs": round(float(np.max(np.abs(w))), 4)},
    "steer_accel_radps2": {"p95": round(float(np.percentile(np.abs(dw), 95)), 3),
                           "max": round(float(np.max(np.abs(dw))), 3)},
    "dt_s": {"median": round(float(np.median([float(r["dt"]) for r in good if r.get("dt")])), 4)} if good and good[0].get("dt") else None,
    "status_counts": {s: sum(1 for r in rows if r["status"] == s) for s in sorted({r["status"] for r in rows})},
    "track_source_counts": {s: sum(1 for r in rows if r.get("track_source") == s) for s in sorted({r.get("track_source", "") for r in rows})},
    "map_status_counts": {s: sum(1 for r in rows if r.get("map_status") == s) for s in sorted({r.get("map_status", "") for r in rows})},
    "has_obs_fraction": round(float(np.mean([int(r.get("has_obs") or 0) for r in rows])), 3),
    "n_two_median": float(np.median([int(r["n_two"]) for r in good])),
    "stop_reasons": stop, "launch_tail": log.strip().splitlines()[-1] if log else "",
}
# per-window cross-track around configured gap spans (if any)
spans = {"left_gap[-3,-1.5]": (-3, -1.5), "right_gap[1,2.5]": (1, 2.5), "full_gap[4.5,5.3]": (4.5, 5.3)}
if "gaps" in world.name:
    out["gap_windows"] = {}
    for k, (a, b) in spans.items():
        m = (x >= a - 0.5) & (x <= b + 1.0)
        if m.any():
            out["gap_windows"][k] = {"max_abs_ct_m": round(float(np.max(np.abs(ct[m]))), 4),
                                     "max_abs_w": round(float(np.max(np.abs(w[m]))), 3),
                                     "obs_fraction": round(float(np.mean([int(r.get("has_obs") or 0) for r, mm in zip(good, m) if mm])), 3)}
print(json.dumps(out, indent=1))
