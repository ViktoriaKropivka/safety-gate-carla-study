#!/usr/bin/env python3
"""Extended compact diagnostics for CARLA two-vehicle PD-only pilot CSV.
Uses only the Python standard library; does not modify source data.
"""
import argparse, csv, json, math, os
from statistics import mean


def num(row, key):
    try:
        x = float(row.get(key, ""))
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def f(x, digits=3, suffix=""):
    return "n/a" if x is None else f"{x:.{digits}f}{suffix}"


def main():
    p = argparse.ArgumentParser(description="Extended diagnostics for a CARLA PD-only two-vehicle pilot.")
    p.add_argument("csv_file", nargs="?", default="two_vehicle_pd_only_timestep.csv")
    p.add_argument("--summary-json", default=None)
    p.add_argument("--output", default=None)
    a = p.parse_args()
    csv_path = os.path.abspath(a.csv_file)
    if not os.path.isfile(csv_path):
        raise SystemExit(f"CSV not found: {csv_path}")
    with open(csv_path, newline="", encoding="utf-8-sig") as fh:
        reader = csv.DictReader(fh)
        rows_all = list(reader)
    rows = [r for r in rows_all if r.get("phase") == "pd_only_braking_episode"]
    if not rows:
        raise SystemExit("No pd_only_braking_episode rows found.")
    rows.sort(key=lambda r: num(r, "episode_time_s") if num(r, "episode_time_s") is not None else -1)
    required = ["episode_time_s", "bumper_gap_m", "ego_speed_mps", "lead_speed_mps",
                "ego_accel_raw_mps2", "ego_accel_smooth_5step_mps2",
                "ego_pd_accel_command_mps2", "ego_requested_throttle", "ego_requested_brake"]
    missing = [k for k in required if k not in (rows[0].keys() if rows else [])]
    if missing:
        raise SystemExit("Missing expected fields: " + ", ".join(missing))

    def extrema(key, fn=min):
        vals = [(num(r, key), num(r, "episode_time_s"), i) for i, r in enumerate(rows)]
        vals = [x for x in vals if x[0] is not None and x[1] is not None]
        return fn(vals, key=lambda z: z[0]) if vals else (None, None, None)

    t_first, t_last = num(rows[0], "episode_time_s"), num(rows[-1], "episode_time_s")
    gap_min = extrema("bumper_gap_m")
    raw_min = extrema("ego_accel_raw_mps2")
    smooth_min = extrema("ego_accel_smooth_5step_mps2")
    cmd_min = extrema("ego_pd_accel_command_mps2")
    brake_max = extrema("ego_requested_brake", max)
    throttle_max = extrema("ego_requested_throttle", max)

    # Stable stop criterion used by the pilot: both cars <= 0.08 m/s for 15 ticks.
    stable_n, stable_start = 0, None
    stop_confirm_time = None
    for r in rows:
        ev, lv, t = num(r, "ego_speed_mps"), num(r, "lead_speed_mps"), num(r, "episode_time_s")
        if ev is not None and lv is not None and ev <= 0.08 and lv <= 0.08:
            if stable_n == 0:
                stable_start = t
            stable_n += 1
            if stable_n >= 15:
                stop_confirm_time = t
                break
        else:
            stable_n, stable_start = 0, None

    final = rows[-1]
    final_gap = num(final, "bumper_gap_m")
    final_ev, final_lv = num(final, "ego_speed_mps"), num(final, "lead_speed_mps")
    final_des = num(final, "desired_gap_m")
    final_err = num(final, "gap_error_m")
    final_cmd = num(final, "ego_pd_accel_command_mps2")
    final_thr = num(final, "ego_requested_throttle")
    final_brk = num(final, "ego_requested_brake")

    def context(index, radius=5):
        lo, hi = max(0, index-radius), min(len(rows), index+radius+1)
        out = []
        for r in rows[lo:hi]:
            out.append(" | ".join([
                f(num(r,"episode_time_s"),3), f(num(r,"bumper_gap_m"),3),
                f(num(r,"ego_speed_kmh"),3), f(num(r,"lead_speed_kmh"),3),
                f(num(r,"ego_accel_raw_mps2"),3), f(num(r,"ego_accel_smooth_5step_mps2"),3),
                f(num(r,"ego_pd_accel_command_mps2"),3), f(num(r,"ego_requested_throttle"),3),
                f(num(r,"ego_requested_brake"),3)
            ]))
        return out

    summary_path = a.summary_json or os.path.join(os.path.dirname(csv_path), "two_vehicle_pd_only_summary.json")
    js = None
    if os.path.isfile(summary_path):
        try:
            with open(summary_path, encoding="utf-8-sig") as fh: js = json.load(fh)
        except Exception: pass

    lines = [
        "CARLA TWO-VEHICLE PD PILOT — EXTENDED DIAGNOSTICS",
        f"CSV: {csv_path}", f"Total rows: {len(rows_all)}; episode rows: {len(rows)}",
        f"Observed interval: {f(t_first,3)}–{f(t_last,3)} s ({f((t_last-t_first) if t_first is not None and t_last is not None else None,3)} s)", "",
        "1) COMPLETION AND GAP",
        f"Final ego speed: {f(final_ev,5,' m/s')} ({f(num(final,'ego_speed_kmh'),4,' km/h')})",
        f"Final lead speed: {f(final_lv,5,' m/s')} ({f(num(final,'lead_speed_kmh'),4,' km/h')})",
        f"Final bumper gap: {f(final_gap,4,' m')}; final desired gap: {f(final_des,4,' m')}; final gap error: {f(final_err,4,' m')}",
        f"Minimum observed bumper gap: {f(gap_min[0],4,' m')} at t={f(gap_min[1],3,' s')}",
        f"Is minimum at final sample? {'YES' if gap_min[2] == len(rows)-1 else 'NO'}",
        f"Both vehicles <= 0.08 m/s for 15 consecutive ticks: {'YES, confirmed at '+f(stop_confirm_time,3,' s') if stop_confirm_time is not None else 'NO'}",
        f"Final PD command: {f(final_cmd,4,' m/s^2')}; throttle={f(final_thr,4)}; brake={f(final_brk,4)}",
    ]
    if js:
        lines += [f"Collision detected in JSON: {js.get('collision_detected','n/a')}",
                  f"JSON reported duration: {f(js.get('episode_duration_s'),3,' s')}"]
    lines += ["", "2) PD COMMAND VS MEASURED ACCELERATION",
              f"Most negative PD acceleration command: {f(cmd_min[0],4,' m/s^2')} at t={f(cmd_min[1],3,' s')}",
              f"Most negative raw measured ego acceleration: {f(raw_min[0],4,' m/s^2')} at t={f(raw_min[1],3,' s')}",
              f"Most negative 5-step smoothed ego acceleration: {f(smooth_min[0],4,' m/s^2')} at t={f(smooth_min[1],3,' s')}",
              f"Peak requested brake: {f(brake_max[0],4)} at t={f(brake_max[1],3,' s')}",
              f"Peak requested throttle: {f(throttle_max[0],4)} at t={f(throttle_max[1],3,' s')}",
              "", "3) WINDOW AROUND MOST NEGATIVE RAW ACCELERATION",
              "t[s] | gap[m] | ego[km/h] | lead[km/h] | ego_a_raw | ego_a_smooth5 | PD_a_cmd | throttle | brake"]
    lines += context(raw_min[2]) if raw_min[2] is not None else ["No raw acceleration values."]

    # Representative samples spanning the complete observed duration.
    targets = [0.05,0.10,0.20,0.30,0.50,0.75,1.0,1.25,1.5,2.0,3.0,5.0,7.5,10.0,12.5,15.0,17.5,20.0,22.5,25.0,30.0]
    lines += ["", "4) REPRESENTATIVE SAMPLES", "t[s] | gap[m] | ego[km/h] | lead[km/h] | ego_a_smooth5 | PD_a_cmd | throttle | brake"]
    for target in targets:
        r = min(rows, key=lambda x: abs((num(x,"episode_time_s") or 0)-target))
        tt = num(r,"episode_time_s")
        if target > (t_last or 0)+0.02: continue
        lines.append(" | ".join([f(tt,3), f(num(r,"bumper_gap_m"),3), f(num(r,"ego_speed_kmh"),3),
                                  f(num(r,"lead_speed_kmh"),3), f(num(r,"ego_accel_smooth_5step_mps2"),3),
                                  f(num(r,"ego_pd_accel_command_mps2"),3), f(num(r,"ego_requested_throttle"),3),
                                  f(num(r,"ego_requested_brake"),3)]))
    if js:
        lines += ["", "5) LEAD PROFILE REPRODUCTION",
                  f"Lead speed MAE: {f(js.get('lead_profile_speed_mae_mps'),6,' m/s')}",
                  f"Lead displacement MAE: {f(js.get('lead_profile_displacement_mae_m'),6,' m')}"]
    output = a.output or os.path.join(os.path.dirname(csv_path), "two_vehicle_pd_only_extended_report.txt")
    text = "\n".join(lines) + "\n"
    with open(output,"w",encoding="utf-8") as fh: fh.write(text)
    print(text)
    print("Saved report: " + os.path.abspath(output))

if __name__ == "__main__": main()
