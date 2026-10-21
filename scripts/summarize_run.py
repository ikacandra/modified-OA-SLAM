#!/usr/bin/env python3
"""Summarize one OA-SLAM run into computation_cost.txt and metrics.json.

Usage: summarize_run.py RUN_DIR NAME [RUN_LABEL]
  RUN_DIR contains statistics.txt, frame_times_<NAME>.csv, tegrastats.log and the evo outputs.

Definitions (same as statistics.txt of ORB-SLAM3 / YDM-SLAM):
  Init [s]           program start -> detector and SLAM system ready
  Tracking time [ms] mean per-frame time of detection + TrackRGBD (image loading excluded)
  Mean / Min FPS     mean / minimum of the per-frame FPS (1 / tracking time)
  RAM, GPU, ...      tegrastats samples between loop_start and loop_end
"""

import csv
import json
import re
import statistics
import sys
from datetime import datetime
from pathlib import Path


def stats(values):
    if not values:
        return None
    return {"mean": statistics.fmean(values), "std": statistics.pstdev(values),
            "median": statistics.median(values), "min": min(values), "max": max(values)}


def fmt(s, unit, prec=2):
    if s is None:
        return "n/a"
    return (f"mean {s['mean']:.{prec}f} {unit} | std {s['std']:.{prec}f} | median {s['median']:.{prec}f}"
            f" | min {s['min']:.{prec}f} | max {s['max']:.{prec}f}")


def read_statistics(path):
    out = {}
    for line in path.read_text().splitlines():
        if ":" in line:
            key, val = line.split(":", 1)
            try:
                out[key.strip()] = float(val)
            except ValueError:
                pass
    return out


def evo_rmse(path):
    """RMSE from an evo_ape / evo_rpe text output, or None."""
    if not path.exists():
        return None
    m = re.search(r"^\s*rmse\s+([\d.eE+-]+)", path.read_text(), re.M)
    return float(m.group(1)) if m else None


def parse_tegrastats(path, t_start, t_end):
    power, temp = {}, {}
    gpu_load, cpu_load, ram = [], [], []
    for line in path.read_text(errors="ignore").splitlines():
        try:
            t = datetime.strptime(line[:19], "%m-%d-%Y %H:%M:%S").timestamp()
        except ValueError:
            continue
        # tegrastats stamps have 1 s resolution: keep the seconds overlapping the loop
        if t < int(t_start) or t > t_end:
            continue
        for rail, inst, _avg in re.findall(r"\b([A-Z][A-Z0-9_]*) (\d+)mW/(\d+)mW", line):
            power.setdefault(rail, []).append(float(inst) / 1000.0)
        for sensor, c in re.findall(r"\b(\w+)@(-?[\d.]+)C\b", line):
            if float(c) > -200:  # disabled sensors report -256C
                temp.setdefault(sensor, []).append(float(c))
        m = re.search(r"GR3D_FREQ (\d+)%", line)
        if m:
            gpu_load.append(float(m.group(1)))
        m = re.search(r"CPU \[([^\]]*)\]", line)
        if m:
            cores = [float(c.split("%")[0]) for c in m.group(1).split(",") if "%" in c]
            if cores:
                cpu_load.append(statistics.fmean(cores))
        m = re.search(r"RAM (\d+)/(\d+)MB", line)
        if m:
            ram.append(float(m.group(1)) / 1024.0)
    return power, temp, gpu_load, cpu_load, ram


def main():
    run_dir, name = Path(sys.argv[1]), sys.argv[2]
    label = sys.argv[3] if len(sys.argv) > 3 else name

    st = read_statistics(run_dir / "statistics.txt")
    rows = list(csv.DictReader(open(run_dir / f"frame_times_{name}.csv")))
    t_det = [float(r["t_detect_ms"]) for r in rows]
    t_slam = [float(r["t_track_ms"]) for r in rows]
    t_frame = [d + s for d, s in zip(t_det, t_slam)]
    n_det = [int(r["n_detections"]) for r in rows]

    power, temp, gpu_load, cpu_load, ram = parse_tegrastats(
        run_dir / "tegrastats.log", st["loop_start"], st["loop_end"])
    rails_sum = [sum(v) for v in zip(*power.values())] if power else []
    loop_s = st["loop_end"] - st["loop_start"]

    ate = evo_rmse(run_dir / "evo_ape.txt")
    rpe_t = evo_rmse(run_dir / "evo_rpe_trans.txt")
    rpe_r = evo_rmse(run_dir / "evo_rpe_rot.txt")

    m = {
        "run": label,
        "frames": len(rows),
        "init_s": st["Initialize time"],
        "tracking_ms": st["mean tracking time"] * 1000.0,
        "tracking_median_ms": st["median tracking time"] * 1000.0,
        "mean_fps": st["mean FPS"],
        "min_fps": st["min FPS"],
        "max_fps": st["max FPS"],
        "detect_ms": statistics.fmean(t_det),
        "slam_ms": statistics.fmean(t_slam),
        "ram_gb": statistics.fmean(ram) if ram else None,
        "ram_peak_gb": max(ram) if ram else None,
        "gpu_w": statistics.fmean(power["GPU"]) if "GPU" in power else None,
        "cpu_w": statistics.fmean(power["CPU"]) if "CPU" in power else None,
        "total_w": statistics.fmean(rails_sum) if rails_sum else None,
        "gpu_load_pct": statistics.fmean(gpu_load) if gpu_load else None,
        "gpu_temp_c": statistics.fmean(temp["GPU"]) if "GPU" in temp else None,
        "cpu_temp_c": statistics.fmean(temp["CPU"]) if "CPU" in temp else None,
        "ate_rmse_m": ate,
        "rpe_trans_rmse_m": rpe_t,
        "rpe_rot_rmse_deg": rpe_r,
    }
    (run_dir / "metrics.json").write_text(json.dumps(m, indent=2) + "\n")

    def num(v, prec):
        return f"{v:.{prec}f}" if v is not None else "n/a"

    out = [f"OA-SLAM computation cost — {label} / {name}", "=" * 72,
           f"Frames processed      : {len(rows)}",
           f"Init [s]              : {num(m['init_s'], 3)}",
           f"Tracking time [ms]    : {num(m['tracking_ms'], 2)} (median {num(m['tracking_median_ms'], 2)}; detection + TrackRGBD)",
           f"Mean FPS              : {num(m['mean_fps'], 2)}",
           f"Min FPS               : {num(m['min_fps'], 2)}",
           f"RAM [GB]              : {num(m['ram_gb'], 3)} (peak {num(m['ram_peak_gb'], 3)})",
           f"GPU [W]               : {num(m['gpu_w'], 3)}",
           f"ATE RMSE [m]          : {num(ate, 6)}",
           f"RPE trans RMSE [m]    : {num(rpe_t, 6)} (delta 1 frame)",
           f"RPE rot RMSE [deg]    : {num(rpe_r, 6)} (delta 1 frame)",
           f"Detections per frame  : {statistics.fmean(n_det):.2f}" if n_det else "Detections per frame  : n/a",
           "",
           "Per-frame time (ms)",
           f"  YOLOv8n-seg TRT FP16 : {fmt(stats(t_det), 'ms')}",
           f"  SLAM (TrackRGBD)     : {fmt(stats(t_slam), 'ms')}",
           f"  Detection + SLAM     : {fmt(stats(t_frame), 'ms')}",
           "",
           f"tegrastats (interval 100 ms, loop window {loop_s:.1f} s): {len(ram)} samples",
           f"  GPU load (GR3D_FREQ): {fmt(stats(gpu_load), '%', 1)}",
           f"  CPU load (avg cores): {fmt(stats(cpu_load), '%', 1)}",
           f"  RAM used            : {fmt(stats(ram), 'GB', 3)}",
           "", "Power (W) per rail"]
    for rail, v in power.items():
        out.append(f"  {rail:<12}: {fmt(stats(v), 'W', 3)}")
    if rails_sum:
        out.append(f"  {'SUM rails':<12}: {fmt(stats(rails_sum), 'W', 3)}")
    if m["gpu_w"] is not None:
        e = m["gpu_w"] * loop_s
        out.append(f"  GPU energy  : {e:.2f} J over the loop, {e / max(len(rows), 1) * 1000:.2f} mJ/frame")
    out += ["", "Temperature (C)"]
    for sensor, v in temp.items():
        out.append(f"  {sensor:<8}: {fmt(stats(v), 'C', 1)}")
    (run_dir / "computation_cost.txt").write_text("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
