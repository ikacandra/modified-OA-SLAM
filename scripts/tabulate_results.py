#!/usr/bin/env python3
"""Tabulate the repeated runs (metrics.json of every complete run) as mean ± std per sequence.

Usage: tabulate_results.py RESULTS_DIR
Writes RESULTS_DIR/summary_all_runs.csv (one row per run), summary_mean_std.csv and
summary_tables.md (computation cost table and accuracy table).
"""

import csv
import json
import statistics
import sys
from pathlib import Path

ORDER = ["fr3_sitting_xyz", "fr3_walking_half", "fr3_walking_xyz", "fr2_desk_w_person",
         "fr3_walking_static", "fr3_walking_rpy", "crowd", "person_track", "crowd2",
         "fr1_desk2", "fr1_room", "fr2_pioneer_slam"]
LABEL = {"crowd": "bonn_crowd", "person_track": "bonn_person_track", "crowd2": "bonn_crowd2"}
DYNAMIC = set(ORDER[:9])

COST = [("init_s", "Init. [s]", 3), ("tracking_ms", "Tracking time [ms]", 2), ("mean_fps", "Mean FPS", 2),
        ("min_fps", "Min FPS", 2), ("ram_gb", "RAM [GB]", 3), ("gpu_w", "GPU [W]", 3)]
ACC = [("ate_rmse_m", "ATE RMSE [m]", 4), ("rpe_trans_rmse_m", "RPE trans RMSE [m]", 4),
       ("rpe_rot_rmse_deg", "RPE rot RMSE [deg]", 4)]
EXTRA = [("detect_ms", "Detection [ms]", 2), ("slam_ms", "TrackRGBD [ms]", 2), ("cpu_w", "CPU [W]", 3),
         ("total_w", "Total rails [W]", 3), ("gpu_load_pct", "GPU load [%]", 1),
         ("gpu_temp_c", "GPU temp [C]", 1), ("cpu_temp_c", "CPU temp [C]", 1)]


def ms(values, prec):
    values = [v for v in values if v is not None]
    if not values:
        return "n/a", None, None
    mean = statistics.fmean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    return f"{mean:.{prec}f} ± {std:.{prec}f}", mean, std


def main():
    root = Path(sys.argv[1])
    runs = {}
    for f in sorted(root.glob("*_[0-9]*/*/*/metrics.json")):
        if "_incomplete" in f.parts:
            continue
        runs.setdefault(f.parent.name, []).append(json.loads(f.read_text()))
    seqs = [s for s in ORDER if s in runs] + sorted(s for s in runs if s not in ORDER)

    keys = ["seq"] + list(next(iter(runs.values()))[0].keys()) if runs else ["seq"]
    with open(root / "summary_all_runs.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for s in seqs:
            for r in runs[s]:
                w.writerow({"seq": LABEL.get(s, s), **r})

    metrics = COST + ACC + EXTRA
    with open(root / "summary_mean_std.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["seq", "n_runs"] + [f"{k}_{p}" for k, _, _ in metrics for p in ("mean", "std")])
        for s in seqs:
            row = [LABEL.get(s, s), len(runs[s])]
            for k, _, prec in metrics:
                _, mean, std = ms([r.get(k) for r in runs[s]], prec)
                row += [f"{mean:.6f}" if mean is not None else "", f"{std:.6f}" if std is not None else ""]
            w.writerow(row)

    def table(title, cols):
        lines = [f"### {title}", "",
                 "| Sequence | n | " + " | ".join(c[1] for c in cols) + " |",
                 "|---|---|" + "---|" * len(cols)]
        groups = [("Dynamic", [s for s in seqs if s in DYNAMIC]), ("Static", [s for s in seqs if s not in DYNAMIC])]
        for gname, gseqs in groups:
            if not gseqs:
                continue
            for s in gseqs:
                cells = [ms([r.get(k) for r in runs[s]], p)[0] for k, _, p in cols]
                lines.append(f"| {LABEL.get(s, s)} | {len(runs[s])} | " + " | ".join(cells) + " |")
            # mean over sequences of the per-sequence mean (std across sequences)
            cells = []
            for k, _, p in cols:
                per_seq = [ms([r.get(k) for r in runs[s]], p)[1] for s in gseqs]
                cells.append(ms(per_seq, p)[0])
            lines.append(f"| **{gname} mean** | {len(gseqs)} seq | " + " | ".join(cells) + " |")
        return "\n".join(lines) + "\n"

    md = ["# OA-SLAM-InstanceSeg + YOLOv8n-seg TensorRT FP16 (640) — repeated runs", "",
          "Values: mean ± sample std over the complete runs of each sequence. "
          "Group rows: mean ± std of the per-sequence means.", "",
          table("Computation cost", COST), table("Accuracy (evo, SE(3) alignment; RPE delta = 1 frame)", ACC),
          table("Additional cost details", EXTRA)]
    (root / "summary_tables.md").write_text("\n".join(md))
    print("\n".join(md))


if __name__ == "__main__":
    main()
