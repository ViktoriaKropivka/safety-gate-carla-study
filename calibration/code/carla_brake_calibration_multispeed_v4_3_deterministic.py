#!/usr/bin/env python3
"""CARLA multi-speed braking calibration with per-run world reloads and episode reacquisition.

The script uses CARLA 0.9.16-compatible API calls. It enables synchronous
mode and fixed_delta_seconds before loading the target map, then reloads the
world before every run while preserving settings. Each run starts from the same
spawn transform. Vehicle controls are sent with apply_batch_sync and followed
by one explicit world.tick(). The calibration controller, braking command,
thresholds, and stopping criterion are otherwise retained from v3. The script
does not modify vehicle physics; it verifies baseline tire friction.

Example:
  python carla_brake_calibration_multispeed_v4_3_deterministic.py --map Town04_Opt --vehicle vehicle.lincoln.mkz_2017 --speeds-kmh 20,30,40,50 --repeats 3 --spawn-index 90 --expected-tire-friction 3.5

Important: this is a protocol-corrected repeatability check. Target speeds are
provisional calibration points, not the main-experiment speed grid. Actual
speed at brake command is measured and reported for every run. This script
does not by itself establish that the results are repeatable; compare the new
repeats quantitatively after execution.
"""

import argparse
import csv
import gc
import json
import math
import os
import sys
import traceback
from datetime import datetime, timezone

try:
    import carla
except ImportError as exc:
    print("ERROR: cannot import CARLA Python API.")
    print("Run this with the Python version/client library bundled with your CARLA installation.")
    print("See CARLA_ROOT/PythonAPI/carla/dist and the official CARLA quick-start docs.")
    raise SystemExit(2) from exc

FIXED_DELTA_SECONDS = 0.01
BRAKE_COMMAND = 1.0
THROTTLE_COMMAND = 1.0
DECEL_ONSET_THRESHOLD = -0.5  # m/s^2; two consecutive samples required
STOP_SPEED_MPS = 0.08
STOP_STABLE_STEPS = 15
MAX_APPROACH_SIM_SECONDS = 45.0
DEFAULT_SPEED_TOLERANCE_KMH = 1.0
DEFAULT_SETTLE_TIME_SECONDS = 1.0
DEFAULT_APPROACH_KP = 0.8
DEFAULT_APPROACH_KI = 0.25
DEFAULT_APPROACH_MAX_BRAKE = 0.15
MAX_BRAKE_SIM_SECONDS = 45.0
STRAIGHT_SAMPLE_DISTANCE = 2.0
STRAIGHT_LOOKAHEAD_M = 80.0
MAX_YAW_DEVIATION_DEG = 3.0
MAX_PITCH_DEVIATION_DEG = 3.0


def angle_difference_deg(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def get_snapshot_time(world):
    return float(world.get_snapshot().timestamp.elapsed_seconds)


def forward_speed_mps(vehicle):
    """Velocity projected onto the vehicle's current forward direction."""
    transform = vehicle.get_transform()
    forward = transform.get_forward_vector()
    velocity = vehicle.get_velocity()
    return velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z


def longitudinal_displacement_m(start_location, current_location, forward):
    dx = current_location.x - start_location.x
    dy = current_location.y - start_location.y
    dz = current_location.z - start_location.z
    return dx * forward.x + dy * forward.y + dz * forward.z


def straight_spawn_candidates(world_map, min_lookahead=STRAIGHT_LOOKAHEAD_M):
    """Rank map spawn points by straight, same-lane road continuation."""
    ranked = []
    spawn_points = world_map.get_spawn_points()
    for idx, transform in enumerate(spawn_points):
        waypoint = world_map.get_waypoint(
            transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if waypoint is None:
            continue
        start_yaw = waypoint.transform.rotation.yaw
        start_pitch = waypoint.transform.rotation.pitch
        current = waypoint
        distance = 0.0
        min_z = waypoint.transform.location.z
        max_z = min_z
        for _ in range(int(min_lookahead / STRAIGHT_SAMPLE_DISTANCE)):
            candidates = current.next(STRAIGHT_SAMPLE_DISTANCE)
            candidates = [
                candidate for candidate in candidates
                if candidate.road_id == waypoint.road_id
                and candidate.lane_id == waypoint.lane_id
                and angle_difference_deg(candidate.transform.rotation.yaw, start_yaw)
                    <= MAX_YAW_DEVIATION_DEG
                and abs(candidate.transform.rotation.pitch - start_pitch)
                    <= MAX_PITCH_DEVIATION_DEG
            ]
            if not candidates:
                break
            candidates.sort(
                key=lambda candidate: (
                    angle_difference_deg(candidate.transform.rotation.yaw, current.transform.rotation.yaw),
                    abs(candidate.transform.location.z - current.transform.location.z),
                )
            )
            nxt = candidates[0]
            distance += current.transform.location.distance(nxt.transform.location)
            min_z = min(min_z, nxt.transform.location.z)
            max_z = max(max_z, nxt.transform.location.z)
            current = nxt
        # Prefer long, nearly level straight segments; ties resolve by spawn index.
        grade_range = max_z - min_z
        score = distance - 2.0 * grade_range
        ranked.append((score, distance, grade_range, idx, transform))
    ranked.sort(key=lambda item: (-item[0], -item[1], item[3]))
    return ranked


def physics_metadata(vehicle):
    physics = vehicle.get_physics_control()
    result = {
        "mass_kg": float(physics.mass),
        "drag_coefficient": float(physics.drag_coefficient),
        "use_gear_autobox": bool(physics.use_gear_autobox),
        "gear_switch_time_s": float(physics.gear_switch_time),
        "center_of_mass": {
            "x": float(physics.center_of_mass.x),
            "y": float(physics.center_of_mass.y),
            "z": float(physics.center_of_mass.z),
        },
        "wheels": [],
    }
    for wheel in physics.wheels:
        result["wheels"].append({
            "tire_friction": float(wheel.tire_friction),
            "max_brake_torque_Nm": float(wheel.max_brake_torque),
            "radius_cm": float(wheel.radius),
            "max_steer_angle_deg": float(wheel.max_steer_angle),
        })
    return result


def make_row(run_id, vehicle_id, map_name, spawn_index, target_kmh, repeat_i,
             phase, frame, sim_time_s, speed_mps, longitudinal_accel_mps2,
             distance_from_brake_start_m, requested_throttle, requested_brake,
             applied_throttle, applied_brake, brake_command_time_s,
             observed_decel_onset_time_s):
    return {
        "run_id": run_id,
        "vehicle_blueprint": vehicle_id,
        "map": map_name,
        "spawn_index": spawn_index,
        "target_speed_kmh": f"{target_kmh:.3f}",
        "repeat": repeat_i,
        "phase": phase,
        "frame": frame,
        "sim_time_s": f"{sim_time_s:.4f}",
        "speed_mps": f"{speed_mps:.5f}",
        "speed_kmh": f"{speed_mps * 3.6:.4f}",
        "longitudinal_accel_mps2": "" if longitudinal_accel_mps2 is None else f"{longitudinal_accel_mps2:.5f}",
        "distance_from_brake_start_m": "" if distance_from_brake_start_m is None else f"{distance_from_brake_start_m:.5f}",
        "requested_throttle": f"{requested_throttle:.3f}",
        "requested_brake": f"{requested_brake:.3f}",
        "applied_throttle": f"{applied_throttle:.3f}",
        "applied_brake": f"{applied_brake:.3f}",
        "brake_command_time_s": "" if brake_command_time_s is None else f"{brake_command_time_s:.4f}",
        "observed_decel_onset_time_s": "" if observed_decel_onset_time_s is None else f"{observed_decel_onset_time_s:.4f}",
    }


def configure_deterministic_world(world):
    """Enable fixed-step synchronous settings on the current world."""
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    settings.no_rendering_mode = True
    if hasattr(settings, "substepping"):
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
    world.apply_settings(settings)


def verify_deterministic_world(world):
    """Fail fast if a load/reload did not preserve the required settings."""
    settings = world.get_settings()
    if not settings.synchronous_mode:
        raise RuntimeError("World reload lost synchronous_mode=True")
    if settings.fixed_delta_seconds is None or abs(settings.fixed_delta_seconds - FIXED_DELTA_SECONDS) > 1e-9:
        raise RuntimeError(
            "World reload did not preserve fixed_delta_seconds=0.01; "
            f"read back {settings.fixed_delta_seconds!r}"
        )
    if not settings.no_rendering_mode:
        raise RuntimeError("World reload lost no_rendering_mode=True")
    if hasattr(settings, "substepping"):
        if not settings.substepping:
            raise RuntimeError("World load/reload lost physics substepping")
        if abs(settings.max_substep_delta_time - 0.01) > 1e-9 or settings.max_substeps < 1:
            raise RuntimeError("Physics substepping settings changed after world reload")
    return settings


def apply_vehicle_control_checked(client, vehicle, control):
    """Submit one acknowledged vehicle-control command as a synchronous batch."""
    responses = client.apply_batch_sync(
        [carla.command.ApplyVehicleControl(vehicle, control)],
        False,
    )
    if len(responses) != 1 or responses[0].error:
        detail = responses[0].error if responses else "no command response"
        raise RuntimeError(f"CARLA rejected vehicle control command: {detail}")


def main():
    parser = argparse.ArgumentParser(description="Calibrate baseline Lincoln braking in CARLA at multiple speeds without rendering.")
    parser.add_argument("--host", default="127.0.0.1", help="CARLA server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=2000, help="CARLA RPC port (default: 2000)")
    parser.add_argument("--map", default="Town04_Opt",
                        help="CARLA map to load; default: Town04_Opt")
    parser.add_argument("--vehicle", default="vehicle.lincoln.mkz_2017",
                        help="Exact CARLA blueprint ID; default: vehicle.lincoln.mkz_2017")
    parser.add_argument("--speeds-kmh", default="20,30,40,50",
                        help="Comma-separated calibration target speeds in km/h; default: 20,30,40,50")
    parser.add_argument("--speed-tolerance-kmh", type=float, default=DEFAULT_SPEED_TOLERANCE_KMH,
                        help="Allowed speed band for the pre-braking stabilization phase; default: 1.0 km/h")
    parser.add_argument("--settle-time-s", type=float, default=DEFAULT_SETTLE_TIME_SECONDS,
                        help="Required continuous time inside the speed band before braking; default: 1.0 s")
    parser.add_argument("--approach-kp", type=float, default=DEFAULT_APPROACH_KP,
                        help="Proportional speed-controller gain; default: 0.8 (normalized control per m/s error)")
    parser.add_argument("--approach-ki", type=float, default=DEFAULT_APPROACH_KI,
                        help="Integral speed-controller gain; default: 0.25 (normalized control per accumulated m/s error)")
    parser.add_argument("--approach-max-brake", type=float, default=DEFAULT_APPROACH_MAX_BRAKE,
                        help="Maximum normalized brake during speed stabilization; default: 0.15")
    parser.add_argument("--spawn-index", type=int, default=None,
                        help="Use this exact map spawn point index instead of automatic selection")
    parser.add_argument("--repeats", type=int, default=3,
                        help="Number of runs per speed; default: 3")
    parser.add_argument("--output", default="calibration_v4_deterministic_timestep.csv",
                        help="Timestep-level CSV output path")
    parser.add_argument("--summary", default="calibration_v4_deterministic_summary.csv",
                        help="Per-run summary CSV output path")
    parser.add_argument("--metadata", default="calibration_v4_deterministic_metadata.json",
                        help="JSON metadata output path")
    parser.add_argument("--expected-tire-friction", type=float, default=3.5,
                        help="Expected baseline tire friction readback on every wheel; this value is checked but NOT applied (default: 3.5)")
    args = parser.parse_args()

    try:
        speeds_kmh = [float(x.strip()) for x in args.speeds_kmh.split(",") if x.strip()]
    except ValueError as exc:
        parser.error("--speeds-kmh must be comma-separated numbers, e.g. 20,30,40")
    if not speeds_kmh or any(speed <= 0 for speed in speeds_kmh):
        parser.error("--speeds-kmh must contain positive values")
    if args.repeats < 1:
        parser.error("--repeats must be >= 1")
    if args.speed_tolerance_kmh <= 0:
        parser.error("--speed-tolerance-kmh must be > 0")
    if args.settle_time_s <= 0:
        parser.error("--settle-time-s must be > 0")
    if args.approach_kp <= 0:
        parser.error("--approach-kp must be > 0")
    if args.approach_ki < 0:
        parser.error("--approach-ki must be >= 0")
    if not (0.0 <= args.approach_max_brake <= 1.0):
        parser.error("--approach-max-brake must be between 0 and 1")

    for path in (args.output, args.summary, args.metadata):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    print("Connecting to CARLA...")
    print("Client version:", client.get_client_version())
    print("Server version:", client.get_server_version())
    # CARLA 0.9.16 determinism protocol:
    # 1) Enable synchronous fixed-step mode on the currently active world.
    # 2) Load the target map with reset_settings=False so those settings survive.
    # 3) Reload the world again before every subsequent calibration run.
    world = client.get_world()
    configure_deterministic_world(world)
    print(f"Loading target map with synchronous settings preserved: {args.map}")
    world = client.load_world(args.map, reset_settings=False)
    verify_deterministic_world(world)
    world.set_weather(carla.WeatherParameters.ClearNoon)
    warmup_frame = world.tick()  # exactly one warm-up tick after every world load/reload

    world_map = world.get_map()
    map_name = world_map.name
    print("Map:", map_name)
    print("Rendering disabled; fixed simulation step:", FIXED_DELTA_SECONDS, "s")
    print("Synchronous mode verified before the first deterministic run.")

    blueprint_library = world.get_blueprint_library()
    try:
        vehicle_bp = blueprint_library.find(args.vehicle)
    except Exception:
        vehicle_bp = None
    if vehicle_bp is None:
        available = sorted(bp.id for bp in blueprint_library.filter("vehicle.*"))
        print(f"ERROR: vehicle blueprint '{args.vehicle}' was not found.")
        print("Available vehicle blueprints (first 80):")
        print("\n".join(available[:80]))
        raise SystemExit(3)

    candidates = straight_spawn_candidates(world_map)
    if not candidates:
        print("ERROR: no driving-lane spawn points found on this map.")
        raise SystemExit(4)
    if args.spawn_index is None:
        score, straight_distance, grade_range, spawn_index, base_transform = candidates[0]
    else:
        matches = [item for item in candidates if item[3] == args.spawn_index]
        if not matches:
            print(f"ERROR: spawn index {args.spawn_index} was not found among valid driving-lane straight candidates.")
            print("Available candidates:")
            print("\n".join(f"index={item[3]} straight={item[1]:.1f}m grade_range={item[2]:.2f}m" for item in candidates[:30]))
            raise SystemExit(5)
        score, straight_distance, grade_range, spawn_index, base_transform = matches[0]
    print(f"Selected spawn point {spawn_index}; approximate straight continuation={straight_distance:.1f} m; elevation range={grade_range:.2f} m")
    if straight_distance < 40.0:
        print("WARNING: only a short straight segment was found. Inspect map suitability before relying on these measurements.")

    metadata = {
        "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
        "client_version": client.get_client_version(),
        "server_version": client.get_server_version(),
        "map": map_name,
        "vehicle_blueprint": args.vehicle,
        "spawn_index": spawn_index,
        "straight_continuation_estimate_m": straight_distance,
        "elevation_range_estimate_m": grade_range,
        "fixed_delta_seconds": FIXED_DELTA_SECONDS,
        "no_rendering_mode": True,
        "world_sync_enabled_before_target_map_load": True,
        "target_map_loaded_with_reset_settings_false": True,
        "world_reloaded_before_each_run_after_first": True,
        "world_reload_call": "world = client.reload_world(reset_settings=False); retain returned World handle",
        "warmup_ticks_after_each_load_or_reload": 1,
        "world_handle_reacquired_after_reload": True,
        "actor_cleanup_between_runs": "do not call Actor.destroy before reload; drop old Python actor/map/blueprint references and let reload_world destroy all actors",
        "reload_world_return_value_retained": True,
        "vehicle_control_submission": "client.apply_batch_sync([ApplyVehicleControl], due_tick_cue=False) followed by world.tick()",
        "spawn_transform_policy": "same selected spawn index and +0.15 m Z offset; no alternate-transform fallback",
        "initial_warmup_frame_after_target_map_load": int(warmup_frame),
        "weather": "ClearNoon",
        "target_speeds_kmh_provisional": speeds_kmh,
        "repeats_per_speed": args.repeats,
        "pre_brake_speed_control": "proportional_speed_controller_then_stability_window",
        "speed_tolerance_kmh": args.speed_tolerance_kmh,
        "settle_time_seconds": args.settle_time_s,
        "approach_kp": args.approach_kp,
        "approach_ki": args.approach_ki,
        "approach_controller": "PI with conditional integration anti-windup; used only to reach/stabilize target speed before braking",
        "approach_max_brake": args.approach_max_brake,
        "brake_command": BRAKE_COMMAND,
        "expected_baseline_tire_friction_readback_all_wheels": args.expected_tire_friction,
        "tire_friction_is_modified_by_script": False,
        "deceleration_onset_threshold_mps2": DECEL_ONSET_THRESHOLD,
        "deceleration_onset_requires_consecutive_samples": 2,
        "note": "Corrected unified multi-speed calibration series using PI speed stabilization with conditional-integration anti-windup. Target speeds are provisional calibration points, not the final main-experiment grid. Speed is stabilized in a target band before full braking. This script does not modify tire friction or other vehicle physics; it verifies the baseline tire-friction readback.",
    }

    timestep_fields = [
        "run_id", "vehicle_blueprint", "map", "spawn_index", "target_speed_kmh", "repeat",
        "phase", "frame", "sim_time_s", "speed_mps", "speed_kmh",
        "longitudinal_accel_mps2", "distance_from_brake_start_m",
        "requested_throttle", "requested_brake", "applied_throttle", "applied_brake",
        "brake_command_time_s", "observed_decel_onset_time_s",
    ]
    summary_fields = [
        "run_id", "vehicle_blueprint", "map", "spawn_index", "target_speed_kmh", "repeat",
        "actual_speed_at_brake_kmh", "max_speed_after_brake_command_kmh", "brake_command_time_s", "observed_decel_onset_time_s",
        "observed_command_to_decel_s", "stopping_time_after_command_s", "stopping_distance_m",
        "max_observed_deceleration_mps2", "max_smoothed_deceleration_5step_mps2", "onset_threshold_mps2", "fixed_delta_seconds",
        "stop_speed_threshold_mps", "speed_tolerance_kmh", "settle_time_s", "speed_stable_before_braking", "onset_detected",
    ]

    summary_rows = []
    metadata["per_run_world_state"] = {}
    actor = None
    run_counter = 0
    try:
        with open(args.output, "w", newline="", encoding="utf-8") as timestep_file:
            timestep_writer = csv.DictWriter(timestep_file, fieldnames=timestep_fields)
            timestep_writer.writeheader()

            for target_kmh in speeds_kmh:
                target_mps = target_kmh / 3.6
                for repeat_i in range(1, args.repeats + 1):
                    run_id = f"v{target_kmh:g}_r{repeat_i}"
                    print(f"\nRun {run_id}: reach and stabilize near {target_kmh:g} km/h, then full brake")

                    # The first run uses the freshly loaded target map above. Every
                    # later run starts from a newly reloaded world with settings kept.
                    if run_counter > 0:
                        # reload_world creates a new episode. Keep the World
                        # wrapper returned by the call and use it for all further
                        # operations in this episode.
                        print("  Releasing old episode-bound Python references...", flush=True)
                        # reload_world destroys every actor in the previous episode.
                        # Do not call actor.destroy() immediately before reload: that
                        # was redundant and is the one operation present in the failing
                        # calibration path but absent from the successful reload-only
                        # diagnostic. Drop client-side wrappers and let reload_world
                        # perform the complete episode reset.
                        actor = None
                        world_map = None
                        current_map = None
                        current_spawn_points = None
                        blueprint_library = None
                        vehicle_bp = None
                        gc.collect()
                        print("  Reloading CARLA world for a fresh episode...", flush=True)
                        world = client.reload_world(reset_settings=False)
                        if world is None:
                            raise RuntimeError("CARLA reload_world() returned None")
                        print(
                            f"  Reload returned active world: {world.get_map().name}; "
                            f"world_id={getattr(world, 'id', 'n/a')}",
                            flush=True,
                        )
                        verify_deterministic_world(world)
                        world.set_weather(carla.WeatherParameters.ClearNoon)
                        warmup_frame = world.tick()
                        print(f"  Fresh episode ready; warm-up frame={warmup_frame}.", flush=True)
                    run_counter += 1

                    current_map = world.get_map()
                    current_spawn_points = current_map.get_spawn_points()
                    if spawn_index < 0 or spawn_index >= len(current_spawn_points):
                        raise RuntimeError(f"Spawn index {spawn_index} missing after world reload")
                    current_base_transform = current_spawn_points[spawn_index]
                    # Ensure the selected spawn point did not change across world episodes.
                    transform_delta = max(
                        abs(current_base_transform.location.x - base_transform.location.x),
                        abs(current_base_transform.location.y - base_transform.location.y),
                        abs(current_base_transform.location.z - base_transform.location.z),
                        angle_difference_deg(current_base_transform.rotation.yaw, base_transform.rotation.yaw),
                    )
                    if transform_delta > 1e-4:
                        raise RuntimeError(
                            f"Spawn point {spawn_index} changed after reload (max delta={transform_delta:.6g}); "
                            "aborting rather than silently changing initial conditions."
                        )

                    blueprint_library = world.get_blueprint_library()
                    vehicle_bp = blueprint_library.find(args.vehicle)
                    transform = carla.Transform(
                        carla.Location(
                            current_base_transform.location.x,
                            current_base_transform.location.y,
                            current_base_transform.location.z + 0.15,
                        ),
                        current_base_transform.rotation,
                    )
                    metadata["per_run_world_state"][run_id] = {
                        "warmup_frame": int(warmup_frame),
                        "spawn_index": int(spawn_index),
                        "spawn_x_m": float(transform.location.x),
                        "spawn_y_m": float(transform.location.y),
                        "spawn_z_m": float(transform.location.z),
                        "spawn_yaw_deg": float(transform.rotation.yaw),
                    }

                    actor = world.try_spawn_actor(vehicle_bp, transform)
                    if actor is None:
                        raise RuntimeError(
                            "Could not spawn vehicle at the fixed calibration transform. "
                            "No alternate transform will be tried, to preserve identical initial conditions."
                        )
                    actor.set_autopilot(False)
                    # Keep the Lincoln blueprint's baseline physics unchanged. Apply
                    # a brief brake/neutral reset, then verify the original tire-friction
                    # value without assigning any VehiclePhysicsControl settings.
                    apply_vehicle_control_checked(client, actor, carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
                    for _ in range(10):
                        world.tick()
                    apply_vehicle_control_checked(client, actor, carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0))
                    for _ in range(10):
                        world.tick()

                    physics = physics_metadata(actor)
                    actual_frictions = [wheel["tire_friction"] for wheel in physics["wheels"]]
                    if args.expected_tire_friction <= 0:
                        raise ValueError("--expected-tire-friction must be > 0")
                    mismatched = [value for value in actual_frictions
                                  if abs(value - args.expected_tire_friction) > 1e-4]
                    if mismatched:
                        raise RuntimeError(
                            "Baseline tire-friction readback mismatch: expected "
                            f"{args.expected_tire_friction:.3f}, got {actual_frictions}. "
                            "Calibration aborted; check the blueprint and simulation configuration."
                        )
                    metadata.setdefault("actual_tire_friction_readback_all_wheels_by_run", {})[run_id] = actual_frictions
                    metadata.setdefault("vehicle_physics_by_run", {})[run_id] = physics
                    print(f"  baseline tire_friction readback verified on all wheels: {actual_frictions}")

                    # Approach and stabilize near target speed before issuing the full-brake command.
                    # This avoids starting the braking measurement during a full-throttle acceleration
                    # or an immediate transmission transient at the first target-speed crossing.
                    previous_speed = max(0.0, forward_speed_mps(actor))
                    stable_since = None
                    speed_stable = False
                    integral_error = 0.0
                    current_frame = world.get_snapshot().frame
                    tolerance_mps = args.speed_tolerance_kmh / 3.6
                    for _ in range(int(MAX_APPROACH_SIM_SECONDS / FIXED_DELTA_SECONDS)):
                        current_speed_before_control = max(0.0, forward_speed_mps(actor))
                        error_mps = target_mps - current_speed_before_control
                        # PI speed controller with conditional-integration anti-windup.
                        # Positive output commands throttle; negative output commands a
                        # small approach brake. Do not integrate further into saturation.
                        raw_control = args.approach_kp * error_mps + args.approach_ki * integral_error
                        saturated_high = raw_control >= 1.0 and error_mps > 0.0
                        saturated_low = raw_control <= -args.approach_max_brake and error_mps < 0.0
                        if not (saturated_high or saturated_low):
                            integral_error += error_mps * FIXED_DELTA_SECONDS
                            # Limit stored integral to avoid extreme recovery after long transients.
                            integral_error = max(-10.0, min(10.0, integral_error))
                        control_value = args.approach_kp * error_mps + args.approach_ki * integral_error
                        requested_throttle = max(0.0, min(1.0, control_value))
                        requested_brake = max(0.0, min(args.approach_max_brake, -control_value))
                        apply_vehicle_control_checked(client, actor, carla.VehicleControl(
                            throttle=requested_throttle, brake=requested_brake, steer=0.0
                        ))
                        current_frame = world.tick()
                        current_time = get_snapshot_time(world)
                        current_speed = max(0.0, forward_speed_mps(actor))
                        accel = (current_speed - previous_speed) / FIXED_DELTA_SECONDS
                        applied_control = actor.get_control()
                        timestep_writer.writerow(make_row(
                            run_id, args.vehicle, map_name, spawn_index, target_kmh, repeat_i,
                            "speed_control", current_frame, current_time, current_speed, accel,
                            None, requested_throttle, requested_brake,
                            float(applied_control.throttle), float(applied_control.brake), None, None,
                        ))
                        timestep_file.flush()
                        previous_speed = current_speed

                        if abs(current_speed - target_mps) <= tolerance_mps:
                            if stable_since is None:
                                stable_since = current_time
                            elif current_time - stable_since >= args.settle_time_s:
                                speed_stable = True
                                break
                        else:
                            stable_since = None

                    if not speed_stable:
                        raise RuntimeError(
                            f"Could not stabilize near {target_kmh:g} km/h within {MAX_APPROACH_SIM_SECONDS:.0f} s; "
                            f"last speed was {current_speed*3.6:.2f} km/h. No braking run was recorded for this point."
                        )

                    # Command full braking from a settled, measured starting speed.
                    brake_command_time = get_snapshot_time(world)
                    brake_start_location = actor.get_location()
                    start_transform = actor.get_transform()
                    forward = start_transform.get_forward_vector()
                    actual_speed_at_brake = max(0.0, forward_speed_mps(actor))
                    apply_vehicle_control_checked(client, actor, carla.VehicleControl(throttle=0.0, brake=BRAKE_COMMAND, steer=0.0))
                    previous_speed = actual_speed_at_brake
                    previous_time = brake_command_time
                    acceleration_samples = []
                    max_speed_after_brake = actual_speed_at_brake
                    onset_candidate_time = None
                    onset_candidate_count = 0
                    observed_onset_time = None
                    stop_stable_count = 0
                    last_time = brake_command_time
                    stopping_distance = 0.0

                    for _ in range(int(MAX_BRAKE_SIM_SECONDS / FIXED_DELTA_SECONDS)):
                        current_frame = world.tick()
                        current_time = get_snapshot_time(world)
                        current_speed = max(0.0, forward_speed_mps(actor))
                        accel = (current_speed - previous_speed) / (current_time - previous_time)
                        acceleration_samples.append(accel)
                        max_speed_after_brake = max(max_speed_after_brake, current_speed)

                        # Detect onset before writing the row so the timestamp is present on
                        # the first row where the two-sample criterion is satisfied.
                        if accel <= DECEL_ONSET_THRESHOLD:
                            if onset_candidate_count == 0:
                                # Acceleration is estimated over [previous_time, current_time];
                                # use the interval start as the onset estimate (resolution = fixed step).
                                onset_candidate_time = previous_time
                            onset_candidate_count += 1
                            if onset_candidate_count >= 2 and observed_onset_time is None:
                                observed_onset_time = onset_candidate_time
                                print(f"  detected braking onset about {observed_onset_time - brake_command_time:.3f} s after command")
                        else:
                            onset_candidate_count = 0
                            onset_candidate_time = None

                        displacement = longitudinal_displacement_m(brake_start_location, actor.get_location(), forward)
                        applied_control = actor.get_control()
                        timestep_writer.writerow(make_row(
                            run_id, args.vehicle, map_name, spawn_index, target_kmh, repeat_i,
                            "brake", current_frame, current_time, current_speed, accel,
                            displacement, 0.0, BRAKE_COMMAND,
                            float(applied_control.throttle), float(applied_control.brake),
                            brake_command_time, observed_onset_time,
                        ))
                        timestep_file.flush()

                        if current_speed <= STOP_SPEED_MPS:
                            stop_stable_count += 1
                        else:
                            stop_stable_count = 0

                        previous_speed = current_speed
                        previous_time = current_time
                        last_time = current_time
                        stopping_distance = displacement
                        if stop_stable_count >= STOP_STABLE_STEPS:
                            break

                    stopped = stop_stable_count >= STOP_STABLE_STEPS
                    stopping_time = last_time - brake_command_time
                    max_observed_decel = max(0.0, -min(acceleration_samples)) if acceleration_samples else 0.0
                    # A 5-sample (0.05 s) moving average is reported alongside the
                    # raw one-step peak to reduce sensitivity to single-frame spikes.
                    smoothed_decelerations = [
                        sum(acceleration_samples[i:i + 5]) / 5.0
                        for i in range(max(0, len(acceleration_samples) - 4))
                    ]
                    max_smoothed_decel = max(0.0, -min(smoothed_decelerations)) if smoothed_decelerations else 0.0
                    onset_delay = (observed_onset_time - brake_command_time) if observed_onset_time is not None else ""
                    summary = {
                        "run_id": run_id,
                        "vehicle_blueprint": args.vehicle,
                        "map": map_name,
                        "spawn_index": spawn_index,
                        "target_speed_kmh": f"{target_kmh:.3f}",
                        "repeat": repeat_i,
                        "actual_speed_at_brake_kmh": f"{actual_speed_at_brake * 3.6:.4f}",
                        "max_speed_after_brake_command_kmh": f"{max_speed_after_brake * 3.6:.4f}",
                        "brake_command_time_s": f"{brake_command_time:.4f}",
                        "observed_decel_onset_time_s": "" if observed_onset_time is None else f"{observed_onset_time:.4f}",
                        "observed_command_to_decel_s": "" if onset_delay == "" else f"{onset_delay:.4f}",
                        "stopping_time_after_command_s": f"{stopping_time:.4f}" if stopped else "NOT_STOPPED_WITHIN_LIMIT",
                        "stopping_distance_m": f"{stopping_distance:.4f}" if stopped else "",
                        "max_observed_deceleration_mps2": f"{max_observed_decel:.4f}",
                        "max_smoothed_deceleration_5step_mps2": f"{max_smoothed_decel:.4f}",
                        "onset_threshold_mps2": f"{DECEL_ONSET_THRESHOLD:.3f}",
                        "fixed_delta_seconds": f"{FIXED_DELTA_SECONDS:.3f}",
                        "stop_speed_threshold_mps": f"{STOP_SPEED_MPS:.3f}",
                        "speed_tolerance_kmh": f"{args.speed_tolerance_kmh:.3f}",
                        "settle_time_s": f"{args.settle_time_s:.3f}",
                        "speed_stable_before_braking": bool(speed_stable),
                        "onset_detected": bool(observed_onset_time is not None),
                    }
                    summary_rows.append(summary)
                    print(f"  actual start speed: {actual_speed_at_brake*3.6:.2f} km/h")
                    print(f"  maximum speed after brake command: {max_speed_after_brake*3.6:.2f} km/h")
                    print(f"  maximum observed deceleration (raw one-step difference): {max_observed_decel:.2f} m/s^2")
                    print(f"  maximum deceleration (5-step moving average, 0.05 s): {max_smoothed_decel:.2f} m/s^2")
                    print(f"  stopping distance: {stopping_distance:.2f} m" if stopped else "  WARNING: vehicle did not meet stop criterion in time")

                    # Do not explicitly destroy this actor before the next reload.
                    # CARLA reload_world() destroys all actors in the old episode.
                    # Retain the final actor handle so finally{} can clean it up if
                    # this is the last run in the requested series.
                    if run_counter < (len(speeds_kmh) * args.repeats):
                        print("  Previous actor will be removed by the next world reload.", flush=True)

        with open(args.summary, "w", newline="", encoding="utf-8") as summary_file:
            writer = csv.DictWriter(summary_file, fieldnames=summary_fields)
            writer.writeheader()
            writer.writerows(summary_rows)
        with open(args.metadata, "w", encoding="utf-8") as metadata_file:
            json.dump(metadata, metadata_file, ensure_ascii=False, indent=2)

        print("\nCalibration finished.")
        print("Timestep data:", os.path.abspath(args.output))
        print("Per-run summary:", os.path.abspath(args.summary))
        print("Configuration metadata:", os.path.abspath(args.metadata))
        print("Note: this corrected protocol logs requested and last-applied controls. Inspect the traces before freezing calibration values.")

    finally:
        try:
            if actor is not None:
                actor.destroy()
        except Exception:
            pass
        # Leave CARLA in no-rendering synchronous mode. This avoids the server
        # free-running after the client exits. Stop the server with Ctrl+C when done.
        try:
            current_settings = world.get_settings()
            current_settings.synchronous_mode = True
            current_settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
            current_settings.no_rendering_mode = True
            world.apply_settings(current_settings)
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted by user.")
        raise SystemExit(130)
    except Exception as exc:
        print("ERROR:", exc, file=sys.stderr)
        traceback.print_exc()
        raise SystemExit(1)
