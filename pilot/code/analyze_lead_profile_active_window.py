#!/usr/bin/env python3
"""Recompute lead braking-profile errors for the shared 1.42 s active window."""

import argparse
import csv
import math
import os


def read_csv(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    a = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + a * (ys[hi] - ys[lo])


def main():
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", default=os.path.join(project_root, "pilot", "data"))
    parser.add_argument("--profile", default=os.path.join(project_root, "calibration", "data", "leader_braking_profile_50kmh.csv"))
    parser.add_argument("--window-s", type=float, default=1.42)
    parser.add_argument("--output", default="lead_profile_active_window_comparison.csv")
    args = parser.parse_args()

    ref = read_csv(args.profile)
    ref_t = [float(r["time_since_brake_command_s"]) for r in ref]
    ref_v = [float(r["mean_speed_mps"]) for r in ref]
    ref_s = [float(r["mean_displacement_from_brake_start_m"]) for r in ref]
    sources = {
        "A": "two_vehicle_pd_only_30s_timestep.csv",
        "C": "baseline_c_v12_timestep.csv",
        "D": "baseline_d_05s_timestep.csv",
    }
    rows = []
    for case, filename in sources.items():
        path = os.path.join(args.data_dir, filename)
        data = read_csv(path)
        active = []
        full = []
        for r in data:
            try:
                t = float(r["lead_brake_elapsed_s"])
                measured_v = float(r["lead_speed_mps"])
                measured_s = float(r["lead_measured_displacement_m"])
            except (KeyError, TypeError, ValueError):
                continue
            if not math.isfinite(t) or t < 0:
                continue
            if r.get("lead_reference_speed_mps", ""):
                full.append((t, measured_v, measured_s))
            if t <= args.window_s + 1e-9:
                active.append((t, measured_v, measured_s))
        if not active:
            raise RuntimeError(f"No active braking samples found in {path}")
        speed_errors = [v - interp(t, ref_t, ref_v) for t, v, _ in active]
        disp_errors = [s - interp(t, ref_t, ref_s) for t, _, s in active]
        active_mae = sum(abs(x) for x in speed_errors) / len(speed_errors)
        active_rmse = math.sqrt(sum(x * x for x in speed_errors) / len(speed_errors))
        last_t, _, last_s = active[-1]
        endpoint_ref = interp(last_t, ref_t, ref_s)
        full_errors = [v - interp(t, ref_t, ref_v) for t, v, _ in full]
        full_mae = sum(abs(x) for x in full_errors) / len(full_errors) if full_errors else ""
        rows.append({
            "case": case,
            "source_file": filename,
            "n_active_samples": len(active),
            "active_window_start_s": f"{active[0][0]:.2f}",
            "active_window_end_s": f"{last_t:.2f}",
            "active_window_s": f"{args.window_s:.2f}",
            "active_speed_mae_mps": f"{active_mae:.8f}",
            "active_speed_rmse_mps": f"{active_rmse:.8f}",
            "active_displacement_mae_m": f"{sum(abs(x) for x in disp_errors) / len(disp_errors):.8f}",
            "endpoint_displacement_error_m": f"{last_s - endpoint_ref:.8f}",
            "full_logged_window_speed_mae_mps": f"{full_mae:.8f}" if full_errors else "",
        })

    output = args.output if os.path.isabs(args.output) else os.path.join(args.data_dir, args.output)
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    fields = list(rows[0])
    with open(output, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {output}")
    for r in rows:
        print(r["case"], "active MAE =", r["active_speed_mae_mps"], "m/s; endpoint displacement error =", r["endpoint_displacement_error_m"], "m")


if __name__ == "__main__":
    main()
