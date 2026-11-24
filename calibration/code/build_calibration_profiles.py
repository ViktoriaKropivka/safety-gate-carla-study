#!/usr/bin/env python3
"""Build reproducible summaries and mean braking profiles from the retained 12 runs.

The 12 manual_<speed>_r<repeat> files are the authoritative source for this
build. Some files reuse the same internal run_id, so record identity is formed
from the original timestep filename plus the repeat number parsed from it.
No CARLA instance is started and no calibration run is repeated.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import re
from collections import defaultdict
from datetime import datetime, timezone

FILE_RE = re.compile(r"^manual_(\d+)_r(\d+)_timestep\.csv$")
DEFAULT_ALLOWED_GAP_ERROR_M = 1.80


def read_csv(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows, fields):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def fnum(row, key, source):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Missing/non-numeric {key} in {source}") from exc
    if not math.isfinite(value):
        raise ValueError(f"Non-finite {key} in {source}")
    return value


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
    alpha = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + alpha * (ys[hi] - ys[lo])


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def profile_from_run(timestep_path, summary_path, target_speed, repeat_from_filename, dt):
    timestep = read_csv(timestep_path)
    summary_rows = read_csv(summary_path)
    if len(summary_rows) != 1:
        raise ValueError(f"Expected exactly one row in {summary_path}; found {len(summary_rows)}")
    summary = summary_rows[0]
    brake_rows = [r for r in timestep if r.get("phase") == "brake"]
    if not brake_rows:
        raise ValueError(f"No phase='brake' records found in {timestep_path}")

    command_times = [fnum(r, "brake_command_time_s", timestep_path) for r in brake_rows]
    command_time = command_times[0]
    if any(abs(x - command_time) > 1e-9 for x in command_times):
        raise ValueError(f"Inconsistent brake command timestamps in {timestep_path}")

    points = [(0.0, fnum(summary, "actual_speed_at_brake_kmh", summary_path) / 3.6, 0.0)]
    for row in brake_rows:
        t = fnum(row, "sim_time_s", timestep_path) - command_time
        v = fnum(row, "speed_mps", timestep_path)
        s = fnum(row, "distance_from_brake_start_m", timestep_path)
        if t < -1e-8:
            raise ValueError(f"Brake sample precedes brake command in {timestep_path}")
        points.append((max(0.0, t), v, s))
    points.sort(key=lambda x: x[0])

    # Keep the last sample if a duplicate timestamp exists; timestamps must then be strictly increasing.
    dedup = {}
    for point in points:
        dedup[round(point[0], 9)] = point
    points = [dedup[k] for k in sorted(dedup)]
    if len(points) < 2 or any(points[i][0] >= points[i + 1][0] for i in range(len(points) - 1)):
        raise ValueError(f"Invalid time axis in {timestep_path}")

    run_id_values = sorted({r.get("run_id", "") for r in timestep if r.get("run_id")})
    inner_run_id = run_id_values[0] if len(run_id_values) == 1 else ";".join(run_id_values)
    repeat_values = sorted({r.get("repeat", "") for r in timestep if r.get("repeat")})

    return {
        "target_speed_kmh": target_speed,
        "repeat_from_filename": repeat_from_filename,
        "source_timestep_file": os.path.basename(timestep_path),
        "source_summary_file": os.path.basename(summary_path),
        "source_timestep_sha256": sha256_file(timestep_path),
        "source_summary_sha256": sha256_file(summary_path),
        "inner_run_id": inner_run_id,
        "repeat_field_in_source": ";".join(repeat_values),
        "actual_initial_speed_kmh": fnum(summary, "actual_speed_at_brake_kmh", summary_path),
        "brake_command_time_s": fnum(summary, "brake_command_time_s", summary_path),
        "stopping_time_after_command_s": fnum(summary, "stopping_time_after_command_s", summary_path),
        "stopping_distance_m": fnum(summary, "stopping_distance_m", summary_path),
        "max_observed_deceleration_mps2": fnum(summary, "max_observed_deceleration_mps2", summary_path),
        "max_smoothed_deceleration_5step_mps2": fnum(summary, "max_smoothed_deceleration_5step_mps2", summary_path),
        "observed_command_to_decel_s": fnum(summary, "observed_command_to_decel_s", summary_path),
        "speed_stable_before_braking": summary.get("speed_stable_before_braking", ""),
        "points": points,
        "dt_s": dt,
    }


def resample(run, times, column):
    xs = [p[0] for p in run["points"]]
    ys = [p[column] for p in run["points"]]
    return [interp(t, xs, ys) for t in times]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default_data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "data"))
    parser.add_argument("--data-dir", default=default_data_dir)
    parser.add_argument("--summary-output", default="calibration_12_run_summary.csv")
    parser.add_argument("--profiles-output", default="braking_profiles_4speeds_mean.csv")
    parser.add_argument("--validation-output", default="profile_interpolation_validation.csv")
    parser.add_argument("--metadata-output", default="calibration_profile_validation.json")
    parser.add_argument("--allowed-gap-error-m", type=float, default=DEFAULT_ALLOWED_GAP_ERROR_M)
    args = parser.parse_args()
    if args.allowed_gap_error_m <= 0:
        parser.error("--allowed-gap-error-m must be > 0")

    data_dir = os.path.abspath(args.data_dir)
    summary_path = os.path.join(data_dir, args.summary_output)
    profiles_path = os.path.join(data_dir, args.profiles_output)
    validation_path = os.path.join(data_dir, args.validation_output)
    metadata_path = os.path.join(data_dir, args.metadata_output)

    files = sorted(os.listdir(data_dir))
    matches = []
    for name in files:
        match = FILE_RE.match(name)
        if match:
            speed, repeat = int(match.group(1)), int(match.group(2))
            matches.append((speed, repeat, os.path.join(data_dir, name)))
    expected_records = {(speed, repeat) for speed in (20, 30, 40, 50) for repeat in (1, 2, 3)}
    actual_records = {(speed, repeat) for speed, repeat, _ in matches}
    if len(matches) != 12 or actual_records != expected_records:
        missing = sorted(expected_records - actual_records)
        unexpected = sorted(actual_records - expected_records)
        raise RuntimeError(
            "Expected exactly three retained repeats at each of 20, 30, 40, and 50 km/h; "
            f"found {len(matches)} files, missing={missing}, unexpected={unexpected}"
        )

    # The sampling interval is fixed by the experiment, but infer/check it from the records.
    all_deltas = []
    for speed, repeat, path in matches:
        base = os.path.basename(path).replace("_timestep.csv", "")
        s_path = os.path.join(data_dir, base + "_summary.csv")
        if not os.path.isfile(s_path):
            raise FileNotFoundError(f"Missing paired summary file: {s_path}")
        rows = [r for r in read_csv(path) if r.get("phase") == "brake"]
        times = [fnum(r, "sim_time_s", path) for r in rows]
        all_deltas.extend(round(times[i + 1] - times[i], 8) for i in range(len(times) - 1))
    dt = min(all_deltas)
    if not math.isclose(dt, 0.01, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError(f"Expected 0.01 s calibration sampling, inferred {dt}")

    runs = []
    seen_inner_ids = defaultdict(list)
    for speed, repeat, path in matches:
        base = os.path.basename(path).replace("_timestep.csv", "")
        s_path = os.path.join(data_dir, base + "_summary.csv")
        run = profile_from_run(path, s_path, speed, repeat, dt)
        seen_inner_ids[run["inner_run_id"]].append(run["source_timestep_file"])
        runs.append(run)
    for run in runs:
        run["inner_run_id_reused"] = len(seen_inner_ids[run["inner_run_id"]]) > 1

    summary_fields = [
        "record_key", "source_timestep_file", "source_summary_file", "repeat_from_filename",
        "inner_run_id", "inner_run_id_reused", "repeat_field_in_source", "target_speed_kmh",
        "actual_initial_speed_kmh", "brake_command_time_s", "observed_command_to_decel_s",
        "stopping_time_after_command_s", "stopping_distance_m", "max_observed_deceleration_mps2",
        "max_smoothed_deceleration_5step_mps2", "speed_stable_before_braking",
        "source_timestep_sha256", "source_summary_sha256",
    ]
    summary_rows = []
    for r in runs:
        row = {k: r[k] for k in summary_fields if k in r}
        row["record_key"] = f"{r['source_timestep_file']}|repeat={r['repeat_from_filename']}"
        summary_rows.append(row)
    write_csv(summary_path, summary_rows, summary_fields)

    by_speed = defaultdict(list)
    for run in runs:
        by_speed[run["target_speed_kmh"]].append(run)
    mean_profiles = {}
    profile_rows = []
    for speed in sorted(by_speed):
        speed_runs = sorted(by_speed[speed], key=lambda r: r["repeat_from_filename"])
        max_t = max(r["points"][-1][0] for r in speed_runs)
        n_steps = int(round(max_t / dt))
        times = [round(i * dt, 8) for i in range(n_steps + 1)]
        run_speeds = [resample(r, times, 1) for r in speed_runs]
        run_displacements = [resample(r, times, 2) for r in speed_runs]
        mean_v = [mean([values[i] for values in run_speeds]) for i in range(len(times))]
        mean_s = [mean([values[i] for values in run_displacements]) for i in range(len(times))]
        v0 = mean([r["actual_initial_speed_kmh"] / 3.6 for r in speed_runs])
        stop_t = mean([r["stopping_time_after_command_s"] for r in speed_runs])
        mean_profiles[speed] = {"times": times, "v": mean_v, "s": mean_s, "v0": v0, "stop_t": stop_t}
        for t, v, s in zip(times, mean_v, mean_s):
            profile_rows.append({
                "target_speed_kmh": speed,
                "mean_actual_initial_speed_kmh": f"{v0 * 3.6:.6f}",
                "time_since_brake_command_s": f"{t:.2f}",
                "mean_speed_mps": f"{v:.6f}",
                "mean_speed_kmh": f"{v * 3.6:.6f}",
                "mean_displacement_from_brake_start_m": f"{s:.6f}",
                "n_repeats": len(speed_runs),
            })
    write_csv(profiles_path, profile_rows, [
        "target_speed_kmh", "mean_actual_initial_speed_kmh", "time_since_brake_command_s",
        "mean_speed_mps", "mean_speed_kmh", "mean_displacement_from_brake_start_m", "n_repeats",
    ])

    # Leave-one-speed-out interpolation checks at the two internal calibration speeds.
    validation_rows = []
    max_single_vehicle_error = 0.0
    for target, lower, upper in ((30, 20, 40), (40, 30, 50)):
        held = mean_profiles[target]
        lo = mean_profiles[lower]
        hi = mean_profiles[upper]
        alpha = (held["v0"] - lo["v0"]) / (hi["v0"] - lo["v0"])
        dt_grid = dt
        long_end = max(held["times"][-1], lo["times"][-1], hi["times"][-1])
        # The source validation used recorded samples t=0.01..T; the synthetic t=0
        # sample is retained in the mean library but excluded from error metrics.
        grid = [round(i * dt_grid, 8) for i in range(1, int(round(long_end / dt_grid)) + 1)]
        actual_s = [interp(t, held["times"], held["s"]) for t in grid]
        predicted_s = [
            (1.0 - alpha) * interp(t, lo["times"], lo["s"])
            + alpha * interp(t, hi["times"], hi["s"])
            for t in grid
        ]
        errors = [abs(a - p) for a, p in zip(actual_s, predicted_s)]
        signed_errors = [p - a for a, p in zip(actual_s, predicted_s)]
        stop_end = held["stop_t"]
        windows = [
            ("until_heldout_stop", stop_end),
            ("extended_to_longer_neighbor_profile", long_end),
        ]
        for window_name, end_t in windows:
            indices = [i for i, t in enumerate(grid) if t <= end_t + 1e-9]
            e = [errors[i] for i in indices]
            se = [signed_errors[i] for i in indices]
            max_i = max(indices, key=lambda i: errors[i])
            max_error = errors[max_i]
            max_single_vehicle_error = max(max_single_vehicle_error, max_error)
            validation_rows.append({
                "held_out_target_speed_kmh": target,
                "lower_profile_speed_kmh": lower,
                "upper_profile_speed_kmh": upper,
                "held_out_actual_initial_speed_kmh": f"{held['v0'] * 3.6:.6f}",
                "interpolation_weight_alpha": f"{alpha:.8f}",
                "held_out_stop_time_s": f"{held['stop_t']:.3f}",
                "interpolated_scalar_stop_time_s": f"{(1.0-alpha)*lo['stop_t'] + alpha*hi['stop_t']:.3f}",
                "scalar_stop_time_error_s": f"{(1.0-alpha)*lo['stop_t'] + alpha*hi['stop_t'] - held['stop_t']:.3f}",
                "window": window_name,
                "window_start_s": f"{grid[indices[0]]:.2f}",
                "window_end_s": f"{grid[indices[-1]]:.2f}",
                "mae_displacement_error_m": f"{mean(e):.6f}",
                "rmse_displacement_error_m": f"{math.sqrt(mean([x*x for x in e])):.6f}",
                "max_abs_displacement_error_m": f"{max_error:.6f}",
                "time_of_max_error_s": f"{grid[max_i]:.2f}",
                "signed_final_displacement_error_m": f"{se[-1]:.6f}",
            })
    write_csv(validation_path, validation_rows, [
        "held_out_target_speed_kmh", "lower_profile_speed_kmh", "upper_profile_speed_kmh",
        "held_out_actual_initial_speed_kmh", "interpolation_weight_alpha", "held_out_stop_time_s",
        "interpolated_scalar_stop_time_s", "scalar_stop_time_error_s", "window",
        "window_start_s", "window_end_s", "mae_displacement_error_m", "rmse_displacement_error_m",
        "max_abs_displacement_error_m", "time_of_max_error_s", "signed_final_displacement_error_m",
    ])

    two_vehicle_bound = 2.0 * max_single_vehicle_error
    metadata = {
        "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
        "source_dataset": "12 separate manual_<speed>_r<repeat> calibration runs",
        "source_identity_rule": "source_timestep_file + repeat_from_filename; inner run_id is retained only as descriptive metadata because it repeats across some files",
        "source_files": [r["source_timestep_file"] for r in runs],
        "n_runs": len(runs),
        "target_speeds_kmh": sorted(by_speed),
        "repeats_per_speed": {str(k): len(v) for k, v in sorted(by_speed.items())},
        "sampling_step_s": dt,
        "profile_alignment": "time relative to brake command; synthetic t=0 sample from measured initial speed and zero displacement; linear interpolation onto 0.01 s grid; last measured displacement held constant after the recorded profile end",
        "profile_meaning": "mean speed and displacement profile at each calibrated starting speed; these profiles are built for subsequent multi-speed use and are not currently selected by the retained v1.2.1 pilot executor",
        "holdout_validation": {
            "method": "Leave-one-speed-out linear interpolation in measured initial speed between adjacent calibrated mean profiles",
            "held_out_speeds_kmh": [30, 40],
            "maximum_single_vehicle_displacement_error_m": max_single_vehicle_error,
            "maximum_single_vehicle_error_scope_s": "0 to the end of the longer adjacent profile; maximum horizon 1.42 s",
            "conservative_two_vehicle_gap_error_bound_m": two_vehicle_bound,
            "bound_definition": "|e_gap| <= |e_lead_displacement| + |e_ego_displacement|; both terms are bounded by the maximum held-out single-vehicle displacement error",
            "max_allowed_two_vehicle_gap_error_m": args.allowed_gap_error_m,
            "acceptance": "Accept only when the validation bound is <= the configured allowance; otherwise Gate must use INVALID fail-safe braking",
            "interpretation": "Preliminary engineering screening bound from two internal held-out speeds; not a statistical confidence bound and not evidence of extrapolation accuracy",
        },
        "outputs": {
            "run_summary_csv": os.path.basename(summary_path),
            "mean_profiles_csv": os.path.basename(profiles_path),
            "interpolation_validation_csv": os.path.basename(validation_path),
        },
    }
    with open(metadata_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)

    print(f"Processed {len(runs)} run files without re-running CARLA calibration.")
    print(f"Run summary: {summary_path}")
    print(f"Mean profiles: {profiles_path}")
    print(f"Interpolation validation: {validation_path}")
    print(f"Validation metadata: {metadata_path}")
    print(f"Max single-vehicle displacement error: {max_single_vehicle_error:.6f} m")
    print(f"Conservative two-vehicle gap-error bound: {two_vehicle_bound:.6f} m")
    print(f"Allowed preliminary bound: {args.allowed_gap_error_m:.3f} m")
    if two_vehicle_bound > args.allowed_gap_error_m + 1e-12:
        raise SystemExit("ERROR: two-vehicle gap-error bound exceeds the configured allowance; the profile set must be treated as INVALID for Gate use.")


if __name__ == "__main__":
    main()
