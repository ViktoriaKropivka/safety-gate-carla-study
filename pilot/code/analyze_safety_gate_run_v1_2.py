#!/usr/bin/env python3
"""Compact diagnostic report for CARLA Safety Gate outputs including separate request/readback/onset timestamps."""
import argparse
import csv
import json
import math
import os
import statistics


def read_csv(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def num(row, key):
    try:
        value = row.get(key, "")
        if value in (None, "", "nan", "NaN"):
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def nearest(rows, t, key="episode_time_s"):
    valid = [(abs(num(r, key) - t), r) for r in rows if num(r, key) is not None]
    return min(valid, key=lambda x: x[0])[1] if valid else None


def main():
    ap = argparse.ArgumentParser(description="Summarize a two-vehicle Safety Gate pilot CSV and JSON.")
    ap.add_argument("csv_path")
    ap.add_argument("--summary-json", default=None)
    ap.add_argument("--window-s", type=float, default=0.15,
                    help="Half-width of displayed windows around Gate events (default 0.15 s).")
    args = ap.parse_args()
    rows = read_csv(args.csv_path)
    if not rows:
        raise SystemExit("CSV has no rows")
    csv_path = os.path.abspath(args.csv_path)
    json_path = args.summary_json or os.path.splitext(csv_path)[0].replace("_timestep", "_summary") + ".json"
    summary = {}
    if os.path.isfile(json_path):
        with open(json_path, encoding="utf-8") as f:
            summary = json.load(f)

    episode = [r for r in rows if r.get("phase") == "safety_gate_episode"]
    if not episode:
        episode = rows
    def row_time(r):
        return num(r, "episode_time_s")
    times = [row_time(r) for r in episode if row_time(r) is not None]
    first, last = min(times), max(times)
    gap_rows = [(num(r, "bumper_gap_m"), row_time(r), r) for r in episode if num(r, "bumper_gap_m") is not None]
    min_gap, min_gap_t, _ = min(gap_rows, key=lambda x: x[0]) if gap_rows else (None, None, None)
    pred_rows = [(num(r, "predicted_min_gap_eb_m"), row_time(r), r) for r in episode if num(r, "predicted_min_gap_eb_m") is not None]
    min_pred, min_pred_t, _ = min(pred_rows, key=lambda x: x[0]) if pred_rows else (None, None, None)

    states = {}
    for r in episode:
        st = r.get("gate_state", "")
        if st:
            states[st] = states.get(st, 0) + 1
    gate = summary.get("safety_gate", {})
    event_fields = [
        ("t_U (UNSAFE detection)", gate.get("t_U_s")),
        ("t_B_req (Gate brake request)", gate.get("t_B_req_s")),
        ("t_B_applied (full-brake command readback)", gate.get("t_B_applied_s")),
        ("t_B_act (Gate-phase measured deceleration onset)", gate.get("t_B_act_s")),
        ("D_SG (detection to request)", gate.get("D_SG_s")),
        ("D_req_to_applied (request to CARLA readback)", gate.get("D_req_to_applied_s")),
        ("D_total (detection to measured onset; only if distinguishable)", gate.get("D_total_s")),
    ]

    print("SAFETY GATE PILOT — COMPACT REPORT")
    print(f"CSV: {csv_path}")
    print(f"Rows: {len(rows)}; episode rows: {len(episode)}; observed interval: {first:.3f}–{last:.3f} s ({last-first:.3f} s)")
    print("\n1) OUTCOME")
    print(f"Collision: {summary.get('collision_detected', 'not found in JSON')}")
    print(f"Minimum actual bumper gap: {min_gap:.4f} m at t={min_gap_t:.3f} s" if min_gap is not None else "Minimum actual gap: unavailable")
    print(f"Minimum predicted emergency-braking gap: {min_pred:.4f} m at t≈{min_pred_t:.3f} s" if min_pred is not None else "Minimum predicted gap: unavailable")
    print(f"Final bumper gap: {num(episode[-1], 'bumper_gap_m'):.4f} m" if num(episode[-1], "bumper_gap_m") is not None else "Final gap: unavailable")
    print(f"Final ego speed: {num(episode[-1], 'ego_speed_kmh'):.4f} km/h" if num(episode[-1], "ego_speed_kmh") is not None else "Final ego speed: unavailable")
    print(f"Final lead speed: {num(episode[-1], 'lead_speed_kmh'):.4f} km/h" if num(episode[-1], "lead_speed_kmh") is not None else "Final lead speed: unavailable")

    print("\n2) GATE STATE")
    print(f"Gate triggered: {gate.get('gate_triggered', 'not found in JSON')}")
    print(f"Gate reason: {gate.get('gate_reason')}")
    print(f"Gate latched at end: {gate.get('gate_latched')}")
    print(f"INVALID reason: {gate.get('invalid_reason')}")
    print(f"UNSAFE pending cancellations: {gate.get('unsafe_cancel_count')}")
    print("State tick counts: " + (", ".join(f"{k}={v}" for k, v in sorted(states.items())) if states else "unavailable"))
    for label, value in event_fields:
        print(f"{label}: {value}")
    print(f"Pre-existing ego deceleration at t_U: {gate.get('preexisting_ego_decel_at_tU')}")
    print(f"PD brake command at t_U: {gate.get('ego_pd_brake_at_tU')}")
    print(f"Recent smoothed ego acceleration at t_U: {gate.get('ego_smoothed_accel_at_tU_mps2')} m/s2")
    if gate.get("t_B_act_definition"):
        print(f"t_B_act definition: {gate.get('t_B_act_definition')}")

    print("\n3) PROFILE / CONFIGURATION CHECK")
    print(f"Lead profile speed MAE: {summary.get('lead_profile_speed_mae_mps')}")
    print(f"Lead profile displacement MAE: {summary.get('lead_profile_displacement_mae_m')}")
    print(f"Gate delay: {gate.get('delay_s')} s; safe margin: {gate.get('safe_margin_m')} m; horizon: {gate.get('prediction_horizon_s')} s")
    print(f"Valid prediction ticks: {gate.get('valid_forecast_ticks')}; invalid prediction ticks: {gate.get('invalid_forecast_ticks')}")
    print(f"Profile prediction assumption: {gate.get('ego_profile_prediction')}")

    print("\n4) SAMPLES AROUND GATE EVENTS")
    targets = []
    if gate.get("t_U_s") is not None:
        targets.append(("UNSAFE detection", float(gate["t_U_s"])))
    if gate.get("t_B_req_s") is not None:
        targets.append(("Brake request", float(gate["t_B_req_s"])))
    if not targets:
        # If there was no Gate event, show representative samples around the
        # minimum predicted gap and the minimum actual gap.
        if min_pred_t is not None:
            targets.append(("Minimum predicted gap", min_pred_t))
        if min_gap_t is not None and (not targets or abs(min_gap_t-targets[0][1]) > 0.2):
            targets.append(("Minimum actual gap", min_gap_t))
    shown = set()
    for title, t in targets:
        print(f"-- {title}: t={t:.3f} s --")
        for r in episode:
            rt = row_time(r)
            if rt is None or abs(rt-t) > args.window_s:
                continue
            key = round(rt, 3)
            if key in shown:
                continue
            shown.add(key)
            print(
                f"t={rt:7.3f} gap={num(r,'bumper_gap_m')!s:>8} "
                f"gEB={num(r,'predicted_min_gap_eb_m')!s:>8} "
                f"ego={num(r,'ego_speed_kmh')!s:>8} lead={num(r,'lead_speed_kmh')!s:>8} "
                f"PD={num(r,'ego_pd_accel_command_mps2')!s:>8} "
                f"thr={num(r,'ego_requested_throttle')!s:>7} brake={num(r,'ego_requested_brake')!s:>7} "
                f"state={r.get('gate_state','')} reason={r.get('gate_reason','')}"
            )
    print("\nSaved source report can be shared as plain text; the full CSV is not required.")


if __name__ == "__main__":
    main()
