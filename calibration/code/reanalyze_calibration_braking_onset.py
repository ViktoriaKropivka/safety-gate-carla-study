#!/usr/bin/env python3
"""Reanalyse braking-onset times from an existing CARLA calibration CSV.

This script does NOT connect to CARLA and does NOT modify input files.
It recomputes onset using a 5-step (0.05 s) speed-trend estimate and requires
2 consecutive qualifying windows. The input speed at the brake command is
read from the per-run summary and prepended as the t=0 sample.
"""
import argparse
import csv
import os

WINDOW_STEPS = 5
THRESHOLD_MPS2 = -0.5
CONSECUTIVE_WINDOWS = 2


def read_csv(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def as_float(row, key, default=None):
    value = row.get(key, "")
    if value is None or value == "":
        return default
    return float(value)


def main():
    p = argparse.ArgumentParser(description="Reanalyse brake-onset estimates without rerunning CARLA.")
    p.add_argument("--timestep", required=True, help="Timestep-level CSV from calibration v3")
    p.add_argument("--summary", required=True, help="Per-run summary CSV from calibration v3")
    p.add_argument("--output", default="lincoln_calibration_onset_reanalysis.csv", help="Output summary CSV")
    p.add_argument("--threshold-mps2", type=float, default=THRESHOLD_MPS2,
                   help="Smoothed deceleration threshold (negative), default -0.5 m/s^2")
    p.add_argument("--window-steps", type=int, default=WINDOW_STEPS,
                   help="Speed-trend window in simulation steps, default 5 (0.05 s at dt=0.01)")
    p.add_argument("--consecutive-windows", type=int, default=CONSECUTIVE_WINDOWS,
                   help="Number of consecutive qualifying smoothed windows, default 2")
    args = p.parse_args()

    if args.window_steps < 1 or args.consecutive_windows < 1:
        p.error("window-steps and consecutive-windows must be >= 1")

    timestep_rows = read_csv(args.timestep)
    summary_rows = read_csv(args.summary)
    brake_by_run = {}
    for row in timestep_rows:
        if row.get("phase") == "brake":
            brake_by_run.setdefault(row["run_id"], []).append(row)

    output_rows = []
    for summary in summary_rows:
        run_id = summary.get("run_id", "")
        brake_rows = brake_by_run.get(run_id, [])
        brake_rows.sort(key=lambda r: int(r["frame"]))
        if not brake_rows:
            output_rows.append({
                "run_id": run_id,
                "target_speed_kmh": summary.get("target_speed_kmh", ""),
                "actual_speed_at_brake_kmh": summary.get("actual_speed_at_brake_kmh", ""),
                "old_onset_delay_s": summary.get("observed_command_to_decel_s", ""),
                "new_smoothed_onset_delay_s": "",
                "new_onset_estimate_time_s": "",
                "window_steps": args.window_steps,
                "window_seconds": "",
                "threshold_mps2": args.threshold_mps2,
                "consecutive_windows": args.consecutive_windows,
                "onset_found": False,
                "note": "no brake-phase rows found",
            })
            continue

        command_time = as_float(summary, "brake_command_time_s")
        v0_kmh = as_float(summary, "actual_speed_at_brake_kmh")
        if command_time is None or v0_kmh is None:
            raise ValueError(f"Summary is missing brake_command_time_s or actual_speed_at_brake_kmh for {run_id}")

        # Prepend the measured speed at command time (relative time exactly zero).
        samples = [(0.0, v0_kmh / 3.6)]
        for row in brake_rows:
            t = as_float(row, "sim_time_s") - command_time
            v = as_float(row, "speed_mps")
            if t >= -1e-8 and v is not None:
                samples.append((max(0.0, t), v))
        # Remove duplicate/nonincreasing timestamps defensively.
        clean = []
        for item in samples:
            if not clean or item[0] > clean[-1][0] + 1e-9:
                clean.append(item)
            elif item[0] >= clean[-1][0] - 1e-9:
                clean[-1] = item
        samples = clean

        window_accels = []
        for j in range(args.window_steps, len(samples)):
            t0, v0 = samples[j - args.window_steps]
            t1, v1 = samples[j]
            dt = t1 - t0
            if dt <= 0:
                continue
            window_accels.append({
                "start_t": t0,
                "end_t": t1,
                "accel": (v1 - v0) / dt,
            })

        run_length = 0
        candidate_start = None
        detected = None
        for w in window_accels:
            if w["accel"] <= args.threshold_mps2:
                if run_length == 0:
                    candidate_start = w["start_t"]
                run_length += 1
                if run_length >= args.consecutive_windows:
                    detected = candidate_start
                    break
            else:
                run_length = 0
                candidate_start = None

        old_delay = as_float(summary, "observed_command_to_decel_s")
        if old_delay is None:
            old_abs = as_float(summary, "observed_decel_onset_time_s")
            old_delay = (old_abs - command_time) if old_abs is not None else None

        output_rows.append({
            "run_id": run_id,
            "target_speed_kmh": summary.get("target_speed_kmh", ""),
            "actual_speed_at_brake_kmh": summary.get("actual_speed_at_brake_kmh", ""),
            "old_onset_delay_s": "" if old_delay is None else f"{old_delay:.4f}",
            "new_smoothed_onset_delay_s": "" if detected is None else f"{detected:.4f}",
            "new_onset_estimate_time_s": "" if detected is None else f"{command_time + detected:.4f}",
            "window_steps": args.window_steps,
            "window_seconds": f"{args.window_steps * 0.01:.3f}",
            "threshold_mps2": args.threshold_mps2,
            "consecutive_windows": args.consecutive_windows,
            "onset_found": detected is not None,
            "note": "first qualifying smoothed-window start; derived metric, not a new CARLA event",
        })

    fields = [
        "run_id", "target_speed_kmh", "actual_speed_at_brake_kmh",
        "old_onset_delay_s", "new_smoothed_onset_delay_s", "new_onset_estimate_time_s",
        "window_steps", "window_seconds", "threshold_mps2", "consecutive_windows",
        "onset_found", "note",
    ]
    out_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(output_rows)

    print("Reanalysis complete; CARLA was not launched.")
    print(f"Method: {args.window_steps}-step speed-trend window, threshold <= {args.threshold_mps2:.3f} m/s^2, "
          f"{args.consecutive_windows} consecutive qualifying windows")
    for row in output_rows:
        print(f"{row['run_id']}: old onset delay={row['old_onset_delay_s'] or 'n/a'} s; "
              f"smoothed onset delay={row['new_smoothed_onset_delay_s'] or 'not detected'} s")
    print(f"Output: {os.path.abspath(args.output)}")


if __name__ == "__main__":
    main()
