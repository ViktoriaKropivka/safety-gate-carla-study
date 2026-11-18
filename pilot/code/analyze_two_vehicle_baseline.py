#!/usr/bin/env python3
"""Compact analysis for instrumented two-vehicle baseline runs (PD or immediate brake)."""
import argparse
import csv
import math
import os


def f(row, key, default=float('nan')):
    try:
        value = row.get(key, '')
        return float(value) if value not in ('', None) else default
    except (TypeError, ValueError):
        return default


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv_file')
    ap.add_argument('--window-s', type=float, default=0.12,
                    help='Half-width around the largest negative speed-derived acceleration (default 0.12 s)')
    args = ap.parse_args()
    with open(args.csv_file, newline='', encoding='utf-8-sig') as fh:
        all_rows = list(csv.DictReader(fh))
    rows = [r for r in all_rows if r.get('phase') == 'pd_only_braking_episode']
    if not rows:
        raise SystemExit('No pd_only_braking_episode rows found.')

    def t(r): return f(r, 'episode_time_s')
    def a_speed(r): return f(r, 'ego_accel_raw_mps2')
    def a_carla(r): return f(r, 'ego_carla_longitudinal_accel_mps2')

    peak = min(rows, key=a_speed)
    peak_t = t(peak)
    diffs = [abs(a_speed(r) - a_carla(r)) for r in rows
             if math.isfinite(a_speed(r)) and math.isfinite(a_carla(r))]
    gaps = [f(r, 'bumper_gap_m') for r in rows]
    gears = sorted({r.get('ego_gear', '') for r in rows})
    handbrake_values = sorted({r.get('ego_hand_brake', '') for r in rows})
    reverse_values = sorted({r.get('ego_reverse', '') for r in rows})
    modes = sorted({r.get('ego_control_mode', '') for r in rows})
    brake_req = [f(r, 'ego_requested_brake') for r in rows]
    brake_applied = [f(r, 'ego_applied_brake') for r in rows]
    print('INSTRUMENTED TWO-VEHICLE BASELINE — COMPACT REPORT')
    print('Ego control mode:', ', '.join(modes) if modes else 'not recorded')
    print(f'Max requested/applied ego brake: {max(brake_req):.4f} / {max(brake_applied):.4f}')
    print('File:', os.path.abspath(args.csv_file))
    print(f'Episode rows: {len(rows)}; duration: {t(rows[-1]):.3f} s')
    print(f'Minimum bumper gap: {min(gaps):.4f} m')
    print(f'Most negative speed-derived acceleration: {a_speed(peak):.4f} m/s2 at t={peak_t:.3f} s')
    print(f'CARLA-reported longitudinal acceleration at same tick: {a_carla(peak):.4f} m/s2')
    print(f'Most negative CARLA-reported longitudinal acceleration: {min(map(a_carla, rows)):.4f} m/s2')
    print(f'Max |speed-derivative accel - CARLA accel|: {max(diffs) if diffs else float("nan"):.4f} m/s2')
    print('Observed gear values:', ', '.join(gears))
    print('Handbrake values:', ', '.join(handbrake_values))
    print('Reverse values:', ', '.join(reverse_values))
    print('\nWINDOW AROUND MOST NEGATIVE SPEED-DERIVED ACCELERATION')
    hdr = ('t[s] | v[m/s] | a_speed[m/s2] | a_CARLA_long[m/s2] | a_CARLA_lat[m/s2] | '
           'PD_cmd | brake_req/applied | throttle_req/applied | gear | handbrake | reverse')
    print(hdr)
    for r in rows:
        if abs(t(r) - peak_t) <= args.window_s + 1e-9:
            print(
                f"{t(r):.3f} | {f(r,'ego_speed_mps'):.4f} | {a_speed(r):.3f} | "
                f"{a_carla(r):.3f} | {f(r,'ego_carla_lateral_accel_mps2'):.3f} | "
                f"{f(r,'ego_pd_accel_command_mps2'):.3f} | "
                f"{f(r,'ego_requested_brake'):.3f}/{f(r,'ego_applied_brake'):.3f} | "
                f"{f(r,'ego_requested_throttle'):.3f}/{f(r,'ego_applied_throttle'):.3f} | "
                f"{r.get('ego_gear','')} | {r.get('ego_hand_brake','')} | {r.get('ego_reverse','')}"
            )
    print('\nNOTE: CARLA acceleration is logged as the world acceleration vector projected onto the ego vehicle forward direction. Gear and control flags are read after each synchronous tick.')


if __name__ == '__main__':
    main()
