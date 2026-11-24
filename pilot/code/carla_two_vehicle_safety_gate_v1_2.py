#!/usr/bin/env python3
"""Two-vehicle CARLA Safety Gate pilot (zero-delay baseline / configurable delay).

Derived from the instrumented PD-only two-vehicle pilot and the user's
carla_brake_calibration_multispeed_v4_3_deterministic.py. Preserves the same
CARLA 0.9.16 world protocol, vehicle blueprint, road segment, tire-friction
readback checks, PI speed stabilization, and unchanged vehicle physics.

Safety Gate criterion: predicted minimum bumper gap under immediate emergency
braking of ego, with the lead vehicle continuing its measured calibrated
braking profile. Horizon=2.0 s; the historical C/D pilot used M_safe=4.0 m. The selected braking
profile is fixed from the in-range initial speed. For ego's hypothetical
emergency-braking forecast, its current speed is matched to the corresponding
state along that already-selected full-stop profile; the remaining tail is
used, including below the minimum *initial calibration speed*. This is a
state-aligned continuation of the selected profile, not a new low-speed
initial-profile extrapolation. This assumption is explicitly logged for pilot
validation.

States: SAFE, UNSAFE_PENDING, INVALID, BRAKE_ACTIVE. INVALID before command
causes immediate fail-safe braking. Once an emergency-brake command is sent,
BRAKE_ACTIVE is latched until the episode ends; forecasts cannot clear it.
During a positive configured delay, PD continues. An UNSAFE condition that
clears before the deadline cancels that pending intervention. An INVALID
forecast during a pending delay invokes immediate fail-safe braking.

No vehicle physics parameter is modified.
"""

import argparse
import csv
import json
import math
import os
import queue
import sys
import traceback
from collections import deque
from datetime import datetime, timezone

try:
    import carla
except ImportError as exc:
    print("ERROR: cannot import CARLA Python API.")
    print("Run this with the Python version/client library bundled with CARLA 0.9.16.")
    raise SystemExit(2) from exc

FIXED_DELTA_SECONDS = 0.01
DEFAULT_MAP = "Town04_Opt"
DEFAULT_VEHICLE = "vehicle.lincoln.mkz_2017"
DEFAULT_SPAWN_INDEX = 90
DEFAULT_TARGET_SPEED_KMH = 49.2787
CALIBRATED_MIN_SPEED_KMH = 19.3085
CALIBRATED_MAX_SPEED_KMH = 49.2787
DEFAULT_SPEED_TOLERANCE_KMH = 0.10
DEFAULT_SETTLE_TIME_SECONDS = 1.0
DEFAULT_APPROACH_KP = 0.8
DEFAULT_APPROACH_KI = 0.25
DEFAULT_APPROACH_MAX_BRAKE = 0.15
DEFAULT_INITIAL_GAP_M = 22.53
DEFAULT_D0_M = 2.0
DEFAULT_HEADWAY_S = 1.5
DEFAULT_PD_KP = 0.2       # s^-2
DEFAULT_PD_KD = 0.6       # s^-1
DEFAULT_MIN_ACCEL_MPS2 = -6.0
DEFAULT_MAX_ACCEL_MPS2 = 2.0
DECEL_ONSET_THRESHOLD = -0.5  # m/s^2 on a 5-step moving average
ONSET_WINDOW = 5
ONSET_CONSECUTIVE_WINDOWS = 2
STOP_SPEED_MPS = 0.08
STOP_STABLE_STEPS = 15
DEFAULT_MAX_EPISODE_SECONDS = 30.0
STRAIGHT_SAMPLE_DISTANCE = 2.0
STRAIGHT_LOOKAHEAD_M = 80.0
MAX_YAW_DEVIATION_DEG = 3.0
MAX_PITCH_DEVIATION_DEG = 3.0

CSV_FIELDS = [
    "phase", "frame", "sim_time_s", "episode_time_s",
    "ego_actor_id", "lead_actor_id",
    "ego_x_m", "ego_y_m", "ego_z_m", "lead_x_m", "lead_y_m", "lead_z_m",
    "ego_speed_mps", "ego_speed_kmh", "lead_speed_mps", "lead_speed_kmh",
    "ego_accel_raw_mps2", "ego_accel_smooth_5step_mps2",
    "ego_carla_accel_x_mps2", "ego_carla_accel_y_mps2", "ego_carla_accel_z_mps2",
    "ego_carla_longitudinal_accel_mps2", "ego_carla_lateral_accel_mps2",
    "ego_carla_accel_norm_mps2",
    "ego_angular_velocity_x", "ego_angular_velocity_y", "ego_angular_velocity_z",
    "ego_gear", "ego_hand_brake", "ego_reverse", "ego_manual_gear_shift",
    "lead_accel_raw_mps2", "lead_accel_smooth_5step_mps2",
    "bumper_gap_m", "desired_gap_m", "gap_error_m", "closing_speed_mps",
    "ego_pd_accel_command_mps2", "ego_requested_throttle", "ego_requested_brake",
    "ego_applied_throttle", "ego_applied_brake",
    "lead_requested_throttle", "lead_requested_brake",
    "lead_applied_throttle", "lead_applied_brake",
    "lead_brake_elapsed_s", "lead_reference_speed_mps",
    "lead_measured_displacement_m", "lead_reference_displacement_m",
    "lead_speed_error_mps", "lead_displacement_error_m",
    "collision_detected",
    "gate_eval_time_s", "gate_state", "gate_reason", "gate_latched", "gate_delay_s",
    "predicted_min_gap_eb_m", "prediction_valid", "invalid_reason",
    "unsafe_candidate_t_U_s", "t_U_s", "t_B_req_s", "t_B_applied_s", "t_B_act_s",
    "preexisting_ego_decel_at_tU", "ego_pd_brake_at_tU", "ego_smoothed_accel_at_tU_mps2", "unsafe_pending_age_s",
]


def angle_difference_deg(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def get_snapshot_time(world):
    return float(world.get_snapshot().timestamp.elapsed_seconds)


def forward_speed_mps(vehicle):
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
    ranked = []
    for idx, transform in enumerate(world_map.get_spawn_points()):
        waypoint = world_map.get_waypoint(
            transform.location, project_to_road=True, lane_type=carla.LaneType.Driving
        )
        if waypoint is None:
            continue
        start_yaw = waypoint.transform.rotation.yaw
        start_pitch = waypoint.transform.rotation.pitch
        current = waypoint
        distance = 0.0
        min_z = max_z = waypoint.transform.location.z
        for _ in range(int(min_lookahead / STRAIGHT_SAMPLE_DISTANCE)):
            options = current.next(STRAIGHT_SAMPLE_DISTANCE)
            options = [w for w in options
                       if w.road_id == waypoint.road_id
                       and w.lane_id == waypoint.lane_id
                       and angle_difference_deg(w.transform.rotation.yaw, start_yaw) <= MAX_YAW_DEVIATION_DEG
                       and abs(w.transform.rotation.pitch - start_pitch) <= MAX_PITCH_DEVIATION_DEG]
            if not options:
                break
            options.sort(key=lambda w: (
                angle_difference_deg(w.transform.rotation.yaw, current.transform.rotation.yaw),
                abs(w.transform.location.z - current.transform.location.z)))
            nxt = options[0]
            distance += current.transform.location.distance(nxt.transform.location)
            min_z = min(min_z, nxt.transform.location.z)
            max_z = max(max_z, nxt.transform.location.z)
            current = nxt
        grade_range = max_z - min_z
        score = distance - 2.0 * grade_range
        ranked.append((score, distance, grade_range, idx, transform))
    ranked.sort(key=lambda x: (-x[0], -x[1], x[3]))
    return ranked


def advance_same_lane(start_waypoint, distance_m):
    """Walk forward along the same road/lane; stop rather than switch lanes."""
    current = start_waypoint
    travelled = 0.0
    while travelled < distance_m:
        step = min(STRAIGHT_SAMPLE_DISTANCE, distance_m - travelled)
        options = current.next(step)
        options = [w for w in options
                   if w.road_id == start_waypoint.road_id
                   and w.lane_id == start_waypoint.lane_id
                   and angle_difference_deg(w.transform.rotation.yaw,
                                            start_waypoint.transform.rotation.yaw) <= MAX_YAW_DEVIATION_DEG
                   and abs(w.transform.rotation.pitch - start_waypoint.transform.rotation.pitch) <= MAX_PITCH_DEVIATION_DEG]
        if not options:
            raise RuntimeError(
                f"Same-lane straight continuation ended after {travelled:.2f} m; "
                f"need {distance_m:.2f} m to place the lead vehicle."
            )
        options.sort(key=lambda w: (
            angle_difference_deg(w.transform.rotation.yaw, current.transform.rotation.yaw),
            abs(w.transform.location.z - current.transform.location.z)))
        nxt = options[0]
        travelled += current.transform.location.distance(nxt.transform.location)
        current = nxt
    return current, travelled


def configure_deterministic_world(world):
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
    settings = world.get_settings()
    if not settings.synchronous_mode:
        raise RuntimeError("World does not have synchronous_mode=True")
    if settings.fixed_delta_seconds is None or abs(settings.fixed_delta_seconds - FIXED_DELTA_SECONDS) > 1e-9:
        raise RuntimeError(f"fixed_delta_seconds is not 0.01: {settings.fixed_delta_seconds!r}")
    if not settings.no_rendering_mode:
        raise RuntimeError("World does not have no_rendering_mode=True")
    if hasattr(settings, "substepping"):
        if not settings.substepping or abs(settings.max_substep_delta_time - 0.01) > 1e-9 or settings.max_substeps < 1:
            raise RuntimeError("Physics substepping settings do not match the calibration setup")


def apply_controls_checked(client, controls_by_vehicle):
    commands = [carla.command.ApplyVehicleControl(vehicle, control)
                for vehicle, control in controls_by_vehicle]
    responses = client.apply_batch_sync(commands, False)
    if len(responses) != len(commands):
        raise RuntimeError("CARLA returned an incomplete control-command response batch")
    errors = [response.error for response in responses if response.error]
    if errors:
        raise RuntimeError("CARLA rejected a vehicle-control command: " + "; ".join(errors))


def physics_metadata(vehicle):
    physics = vehicle.get_physics_control()
    return {
        "mass_kg": float(physics.mass),
        "drag_coefficient": float(physics.drag_coefficient),
        "use_gear_autobox": bool(physics.use_gear_autobox),
        "gear_switch_time_s": float(physics.gear_switch_time),
        "wheels": [{
            "tire_friction": float(w.tire_friction),
            "max_brake_torque_Nm": float(w.max_brake_torque),
            "radius_cm": float(w.radius),
            "max_steer_angle_deg": float(w.max_steer_angle),
        } for w in physics.wheels],
    }


def bbox_longitudinal_bounds(vehicle, axis):
    vertices = vehicle.bounding_box.get_world_vertices(vehicle.get_transform())
    projections = [v.x * axis.x + v.y * axis.y + v.z * axis.z for v in vertices]
    return min(projections), max(projections)


def bumper_gap_m(ego, lead, axis):
    _, ego_front = bbox_longitudinal_bounds(ego, axis)
    lead_rear, _ = bbox_longitudinal_bounds(lead, axis)
    return lead_rear - ego_front


def make_transform(location, rotation):
    return carla.Transform(
        carla.Location(location.x, location.y, location.z + 0.15),
        rotation,
    )


def pi_speed_control(target_mps, current_mps, integral_error, kp, ki, max_brake):
    error = target_mps - current_mps
    raw = kp * error + ki * integral_error
    saturated_high = raw >= 1.0 and error > 0.0
    saturated_low = raw <= -max_brake and error < 0.0
    if not (saturated_high or saturated_low):
        integral_error += error * FIXED_DELTA_SECONDS
        integral_error = max(-10.0, min(10.0, integral_error))
    control_value = kp * error + ki * integral_error
    throttle = max(0.0, min(1.0, control_value))
    brake = max(0.0, min(max_brake, -control_value))
    return throttle, brake, integral_error


def pd_following_control(gap_m, ego_speed_mps, lead_speed_mps, d0_m, headway_s,
                         kp, kd, min_accel, max_accel):
    desired_gap = d0_m + headway_s * max(0.0, ego_speed_mps)
    gap_error = gap_m - desired_gap
    relative_speed = lead_speed_mps - ego_speed_mps
    accel = kp * gap_error + kd * relative_speed
    accel = max(min_accel, min(max_accel, accel))
    if accel >= 0.0:
        throttle = max(0.0, min(1.0, accel / 2.0))
        brake = 0.0
    else:
        throttle = 0.0
        brake = max(0.0, min(1.0, -accel / 6.0))
    return desired_gap, gap_error, relative_speed, accel, throttle, brake


def load_reference_profile(path):
    if not path:
        return []
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Reference braking profile not found: {path}")
    with open(path, newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    required = {"time_since_brake_command_s", "mean_speed_mps", "mean_displacement_from_brake_start_m"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"Reference profile must contain columns: {sorted(required)}")
    return [{
        "t": float(r["time_since_brake_command_s"]),
        "v": float(r["mean_speed_mps"]),
        "s": float(r["mean_displacement_from_brake_start_m"]),
    } for r in rows]


def interpolate_profile(profile, t):
    if not profile:
        return "", ""
    if t <= profile[0]["t"]:
        return profile[0]["v"], profile[0]["s"]
    if t >= profile[-1]["t"]:
        return profile[-1]["v"], profile[-1]["s"]
    lo, hi = 0, len(profile) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if profile[mid]["t"] <= t:
            lo = mid
        else:
            hi = mid
    a, b = profile[lo], profile[hi]
    alpha = (t - a["t"]) / (b["t"] - a["t"])
    return a["v"] + alpha * (b["v"] - a["v"]), a["s"] + alpha * (b["s"] - a["s"])


def profile_phase_for_speed(profile, speed_mps):
    """Find time/displacement on the selected full-brake profile at a speed.

    The profile is chosen using the measured in-range speed at episode start.
    Matching the current ego speed to a point on this same profile lets the
    forecast use the already-calibrated remaining braking trajectory even
    after the vehicle decelerates below the minimum initial calibration speed.
    """
    if not profile:
        return None, None, "EMPTY_BRAKING_PROFILE"
    if speed_mps < -1e-6:
        return None, None, "NEGATIVE_EGO_SPEED"
    max_v = max(p["v"] for p in profile)
    min_v = min(p["v"] for p in profile)
    if speed_mps > max_v + 0.15:
        return None, None, "EGO_SPEED_ABOVE_SELECTED_PROFILE"
    if speed_mps <= max(0.08, min_v + 1e-6):
        return profile[-1]["t"], profile[-1]["s"], ""

    # Find a descending segment that brackets the current speed. Ignore the
    # tiny initial speed rise immediately after the brake command.
    for a, b in zip(profile, profile[1:]):
        if a["v"] > b["v"] and a["v"] + 1e-9 >= speed_mps >= b["v"] - 1e-9:
            alpha = (a["v"] - speed_mps) / (a["v"] - b["v"])
            t = a["t"] + alpha * (b["t"] - a["t"])
            s = a["s"] + alpha * (b["s"] - a["s"])
            return t, s, ""
    return None, None, "EGO_SPEED_NOT_BRACKETED_BY_SELECTED_PROFILE"


def predict_min_gap_emergency(profile, current_gap_m, ego_speed_mps,
                              lead_brake_age_s, lead_speed_mps,
                              horizon_s=2.0, step_s=0.01,
                              lead_speed_tolerance_mps=0.75):
    """Predict minimum bumper gap if ego brakes now and lead follows profile."""
    if not profile:
        return None, None, "EMPTY_BRAKING_PROFILE"
    ego_phase_t, ego_phase_s, reason = profile_phase_for_speed(profile, ego_speed_mps)
    if reason:
        return None, None, reason
    lead_ref_v, lead_ref_s = interpolate_profile(profile, lead_brake_age_s)
    if lead_ref_v == "":
        return None, None, "LEAD_PROFILE_UNAVAILABLE"
    if abs(float(lead_speed_mps) - float(lead_ref_v)) > lead_speed_tolerance_mps:
        return None, None, "LEAD_BRAKING_PROFILE_MISMATCH"

    minimum_gap = float("inf")
    n_steps = int(round(horizon_s / step_s))
    for i in range(n_steps + 1):
        tau = i * step_s
        _, lead_future_s = interpolate_profile(profile, lead_brake_age_s + tau)
        _, ego_future_s = interpolate_profile(profile, ego_phase_t + tau)
        lead_delta = float(lead_future_s) - float(lead_ref_s)
        ego_delta = float(ego_future_s) - float(ego_phase_s)
        forecast_gap = current_gap_m + lead_delta - ego_delta
        minimum_gap = min(minimum_gap, forecast_gap)
    return minimum_gap, ego_phase_t, ""


class OnsetDetector:
    def __init__(self):
        self.window = deque(maxlen=ONSET_WINDOW)
        self.candidate_time = None
        self.count = 0
        self.onset_time = None

    def update(self, accel, window_timestamp):
        self.window.append(accel)
        if len(self.window) < ONSET_WINDOW:
            return "", self.onset_time
        smooth = sum(self.window) / len(self.window)
        if self.onset_time is None:
            if smooth <= DECEL_ONSET_THRESHOLD:
                if self.count == 0:
                    # Timestamp of the first completed qualifying 5-step window.
                    self.candidate_time = window_timestamp
                self.count += 1
                if self.count >= ONSET_CONSECUTIVE_WINDOWS:
                    self.onset_time = self.candidate_time
            else:
                self.candidate_time = None
                self.count = 0
        return smooth, self.onset_time


def drain_collision_events(event_queue, event_list):
    while True:
        try:
            event = event_queue.get_nowait()
        except queue.Empty:
            break
        event_list.append(event)


def collision_callback(event_queue):
    def _callback(event):
        other = getattr(event, "other_actor", None)
        collided_actor = getattr(event, "actor", None)
        impulse = getattr(event, "normal_impulse", None)
        event_queue.put({
            "frame": int(getattr(event, "frame", -1)),
            "actor_id": int(collided_actor.id) if collided_actor is not None else int(getattr(event, "actor_id", -1)),
            "other_actor_id": int(other.id) if other is not None else None,
            "other_actor_type": str(other.type_id) if other is not None else "unknown",
            "normal_impulse": None if impulse is None else {
                "x": float(impulse.x), "y": float(impulse.y), "z": float(impulse.z)},
        })
    return _callback


def main():
    parser = argparse.ArgumentParser(description="Two-vehicle CARLA Safety Gate pilot v1.1; delay is configurable, physics remains unchanged.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map", default=DEFAULT_MAP)
    parser.add_argument("--vehicle", default=DEFAULT_VEHICLE)
    parser.add_argument("--spawn-index", type=int, default=DEFAULT_SPAWN_INDEX)
    parser.add_argument("--target-speed-kmh", type=float, default=DEFAULT_TARGET_SPEED_KMH)
    parser.add_argument("--speed-tolerance-kmh", type=float, default=DEFAULT_SPEED_TOLERANCE_KMH)
    parser.add_argument("--settle-time-s", type=float, default=DEFAULT_SETTLE_TIME_SECONDS)
    parser.add_argument("--approach-kp", type=float, default=DEFAULT_APPROACH_KP)
    parser.add_argument("--approach-ki", type=float, default=DEFAULT_APPROACH_KI)
    parser.add_argument("--approach-max-brake", type=float, default=DEFAULT_APPROACH_MAX_BRAKE)
    parser.add_argument("--initial-gap-m", type=float, default=DEFAULT_INITIAL_GAP_M,
                        help="Initial bumper-to-bumper gap in metres; default 22.53 (pilot setting only).")
    parser.add_argument("--d0-m", type=float, default=DEFAULT_D0_M)
    parser.add_argument("--headway-s", type=float, default=DEFAULT_HEADWAY_S)
    parser.add_argument("--pd-kp", type=float, default=DEFAULT_PD_KP)
    parser.add_argument("--pd-kd", type=float, default=DEFAULT_PD_KD)
    parser.add_argument("--min-accel-mps2", type=float, default=DEFAULT_MIN_ACCEL_MPS2)
    parser.add_argument("--max-accel-mps2", type=float, default=DEFAULT_MAX_ACCEL_MPS2)
    parser.add_argument("--max-episode-s", type=float, default=DEFAULT_MAX_EPISODE_SECONDS)
    parser.add_argument("--gate-delay-s", type=float, default=0.0,
                        help="Artificial Safety Gate delay after UNSAFE detection; 0.0 for Baseline C.")
    parser.add_argument("--safe-margin-m", type=float, default=4.0,
                        help="Required minimum predicted bumper gap; default matches historical pilot C/D (4.0 m).")
    parser.add_argument("--prediction-horizon-s", type=float, default=2.0)
    parser.add_argument("--expected-tire-friction", type=float, default=3.5)
    parser.add_argument("--reference-profile", default="leader_braking_profile_50kmh.csv",
                        help="Optional CSV profile for leader-response comparison; set empty string to disable.")
    parser.add_argument("--output", default="safety_gate_timestep.csv")
    parser.add_argument("--summary", default="safety_gate_summary.json")
    args = parser.parse_args()

    if args.target_speed_kmh <= 0 or args.speed_tolerance_kmh <= 0:
        parser.error("Target speed and speed tolerance must be positive")
    if not (CALIBRATED_MIN_SPEED_KMH <= args.target_speed_kmh <= CALIBRATED_MAX_SPEED_KMH):
        parser.error(
            f"Target speed must be within calibrated range {CALIBRATED_MIN_SPEED_KMH:.4f}–"
            f"{CALIBRATED_MAX_SPEED_KMH:.4f} km/h"
        )
    if args.initial_gap_m <= 0 or args.settle_time_s <= 0 or args.max_episode_s <= 0:
        parser.error("Initial gap, settle time, and episode duration must be positive")
    if args.gate_delay_s < 0 or args.safe_margin_m <= 0 or args.prediction_horizon_s <= 0:
        parser.error("Gate delay must be >= 0; safety margin and prediction horizon must be > 0")
    if args.pd_kp < 0 or args.pd_kd < 0 or args.min_accel_mps2 >= args.max_accel_mps2:
        parser.error("Invalid PD gains or acceleration limits")
    if not (0.0 <= args.approach_max_brake <= 1.0):
        parser.error("--approach-max-brake must be between 0 and 1")

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.summary)), exist_ok=True)
    profile_path = args.reference_profile.strip() if args.reference_profile else ""
    if profile_path and not os.path.isabs(profile_path):
        profile_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), profile_path)
    reference_profile = load_reference_profile(profile_path) if profile_path else []

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    print("Connecting to CARLA...")
    print("Client version:", client.get_client_version())
    print("Server version:", client.get_server_version())
    world = client.get_world()
    configure_deterministic_world(world)
    print(f"Loading target map with synchronous settings preserved: {args.map}")
    world = client.load_world(args.map, reset_settings=False)
    verify_deterministic_world(world)
    world.set_weather(carla.WeatherParameters.ClearNoon)
    warmup_frame = world.tick()

    actors = []
    sensors = []
    collision_events_queue = queue.Queue()
    collision_events = []
    try:
        world_map = world.get_map()
        map_name = world_map.name
        spawn_points = world_map.get_spawn_points()
        if args.spawn_index < 0 or args.spawn_index >= len(spawn_points):
            raise ValueError(f"spawn-index {args.spawn_index} is outside 0..{len(spawn_points)-1}")
        candidates = straight_spawn_candidates(world_map)
        valid_indices = {item[3]: item for item in candidates}
        if args.spawn_index not in valid_indices:
            raise RuntimeError(f"Spawn index {args.spawn_index} is not a valid straight same-lane candidate")
        _, straight_distance, grade_range, _, base_transform = valid_indices[args.spawn_index]
        print(f"Map={map_name}; spawn={args.spawn_index}; straight continuation={straight_distance:.1f} m; grade range={grade_range:.2f} m")

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(args.vehicle)
        ego_transform = make_transform(base_transform.location, base_transform.rotation)
        ego = world.try_spawn_actor(vehicle_bp, ego_transform)
        if ego is None:
            raise RuntimeError("Could not spawn ego vehicle at the selected fixed spawn transform")
        actors.append(ego)
        ego.set_autopilot(False)
        ego_physics = physics_metadata(ego)
        ego_half_length = float(ego.bounding_box.extent.x)

        start_waypoint = world_map.get_waypoint(
            base_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving
        )
        if start_waypoint is None:
            raise RuntimeError("Could not project the selected spawn point to a driving-lane waypoint")
        # The vehicles use the same blueprint; use both measured half-lengths
        # when converting the required bumper gap to reference-point spacing.
        center_spacing = args.initial_gap_m + 2.0 * ego_half_length
        lead_waypoint, waypoint_distance = advance_same_lane(start_waypoint, center_spacing)
        lead_transform = make_transform(lead_waypoint.transform.location, lead_waypoint.transform.rotation)
        lead = world.try_spawn_actor(vehicle_bp, lead_transform)
        if lead is None:
            raise RuntimeError("Could not spawn lead vehicle at the calculated same-lane location")
        actors.append(lead)
        lead.set_autopilot(False)
        lead_physics = physics_metadata(lead)
        lead_half_length = float(lead.bounding_box.extent.x)
        if abs(lead_half_length - ego_half_length) > 1e-4:
            raise RuntimeError("The two same-blueprint vehicles have different longitudinal bounding-box extents")

        # Align the measured world-space bumper gap to the requested value before
        # the timed episode. The selected segment is straight by construction.
        world.tick()
        axis = ego.get_transform().get_forward_vector()
        measured_gap = bumper_gap_m(ego, lead, axis)
        gap_correction = args.initial_gap_m - measured_gap
        if abs(gap_correction) > 0.5:
            raise RuntimeError(
                f"Spawned bumper gap differs from request by {gap_correction:.3f} m; "
                "refusing a large transform correction."
            )
        if abs(gap_correction) > 0.01:
            corrected = lead.get_transform()
            corrected.location.x += axis.x * gap_correction
            corrected.location.y += axis.y * gap_correction
            corrected.location.z += axis.z * gap_correction
            lead.set_transform(corrected)
            world.tick()
            measured_gap = bumper_gap_m(ego, lead, ego.get_transform().get_forward_vector())
        if abs(measured_gap - args.initial_gap_m) > 0.05:
            raise RuntimeError(f"Could not set requested bumper gap: requested={args.initial_gap_m:.3f} m, measured={measured_gap:.3f} m")

        tire_values = [w["tire_friction"] for w in ego_physics["wheels"] + lead_physics["wheels"]]
        mismatch = [v for v in tire_values if abs(v - args.expected_tire_friction) > 1e-4]
        if mismatch:
            raise RuntimeError(
                f"Baseline tire-friction readback mismatch; expected {args.expected_tire_friction:.3f}, got {tire_values}. "
                "No physics parameters were modified."
            )
        print("Verified baseline tire friction on both vehicles:", tire_values)
        print(f"Initial bumper gap verified: {measured_gap:.3f} m")

        # Match the calibration script's brake/neutral reset before speed
        # acquisition: 10 fixed ticks with brake, then 10 ticks released.
        apply_controls_checked(client, [
            (ego, carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)),
            (lead, carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)),
        ])
        for _ in range(10):
            world.tick()
        apply_controls_checked(client, [
            (ego, carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0)),
            (lead, carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0)),
        ])
        for _ in range(10):
            world.tick()

        collision_bp = blueprint_library.find("sensor.other.collision")
        for vehicle in (ego, lead):
            sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
            if sensor is None:
                raise RuntimeError(f"Could not attach collision sensor to actor {vehicle.id}")
            sensors.append(sensor)
            sensor.listen(collision_callback(collision_events_queue))
        world.tick()  # allow collision sensors to register before the timed episode

        # Stabilize both vehicles independently using the same PI procedure as
        # the calibration script. Stable window is accepted only inside the
        # calibrated interval; overspeed above the upper calibration bound is
        # not allowed to start the braking episode.
        target_mps = args.target_speed_kmh / 3.6
        tolerance_mps = args.speed_tolerance_kmh / 3.6
        integrals = {ego.id: 0.0, lead.id: 0.0}
        stable_since = {ego.id: None, lead.id: None}
        stable = {ego.id: False, lead.id: False}
        previous_speeds = {ego.id: max(0.0, forward_speed_mps(ego)),
                           lead.id: max(0.0, forward_speed_mps(lead))}
        stabilization_start = get_snapshot_time(world)
        stabilization_limit = 45.0
        with open(args.output, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

            for _ in range(int(stabilization_limit / FIXED_DELTA_SECONDS)):
                now = get_snapshot_time(world)
                control_pairs = []
                requests = {}
                for vehicle, label in ((ego, "ego"), (lead, "lead")):
                    v = max(0.0, forward_speed_mps(vehicle))
                    throttle, brake, integrals[vehicle.id] = pi_speed_control(
                        target_mps, v, integrals[vehicle.id], args.approach_kp,
                        args.approach_ki, args.approach_max_brake)
                    control = carla.VehicleControl(throttle=throttle, brake=brake, steer=0.0)
                    control_pairs.append((vehicle, control))
                    requests[label] = (throttle, brake)
                apply_controls_checked(client, control_pairs)
                frame = world.tick()
                now = get_snapshot_time(world)
                for vehicle, label in ((ego, "ego"), (lead, "lead")):
                    v = max(0.0, forward_speed_mps(vehicle))
                    in_band = (target_mps - tolerance_mps <= v <= CALIBRATED_MAX_SPEED_KMH / 3.6)
                    if in_band:
                        if stable_since[vehicle.id] is None:
                            stable_since[vehicle.id] = now
                        elif now - stable_since[vehicle.id] >= args.settle_time_s:
                            stable[vehicle.id] = True
                    else:
                        stable_since[vehicle.id] = None
                        stable[vehicle.id] = False
                    old_v = previous_speeds[vehicle.id]
                    accel = (v - old_v) / FIXED_DELTA_SECONDS
                    previous_speeds[vehicle.id] = v
                    control = vehicle.get_control()
                    loc = vehicle.get_location()
                    if label == "ego":
                        ego_fields = (loc, v, accel, control, requests[label])
                    else:
                        lead_fields = (loc, v, accel, control, requests[label])
                ego_loc, ego_v, ego_a, ego_applied, ego_req = ego_fields
                lead_loc, lead_v, lead_a, lead_applied, lead_req = lead_fields
                row = {field: "" for field in CSV_FIELDS}
                row.update({
                    "phase": "speed_stabilization", "frame": frame, "sim_time_s": f"{now:.4f}",
                    "ego_actor_id": ego.id, "lead_actor_id": lead.id,
                    "ego_x_m": f"{ego_loc.x:.5f}", "ego_y_m": f"{ego_loc.y:.5f}", "ego_z_m": f"{ego_loc.z:.5f}",
                    "lead_x_m": f"{lead_loc.x:.5f}", "lead_y_m": f"{lead_loc.y:.5f}", "lead_z_m": f"{lead_loc.z:.5f}",
                    "ego_speed_mps": f"{ego_v:.5f}", "ego_speed_kmh": f"{ego_v*3.6:.4f}",
                    "lead_speed_mps": f"{lead_v:.5f}", "lead_speed_kmh": f"{lead_v*3.6:.4f}",
                    "ego_accel_raw_mps2": f"{ego_a:.5f}", "lead_accel_raw_mps2": f"{lead_a:.5f}",
                    "bumper_gap_m": f"{bumper_gap_m(ego, lead, ego.get_transform().get_forward_vector()):.5f}",
                    "ego_requested_throttle": f"{ego_req[0]:.5f}", "ego_requested_brake": f"{ego_req[1]:.5f}",
                    "ego_applied_throttle": f"{ego_applied.throttle:.5f}", "ego_applied_brake": f"{ego_applied.brake:.5f}",
                    "lead_requested_throttle": f"{lead_req[0]:.5f}", "lead_requested_brake": f"{lead_req[1]:.5f}",
                    "lead_applied_throttle": f"{lead_applied.throttle:.5f}", "lead_applied_brake": f"{lead_applied.brake:.5f}",
                })
                writer.writerow(row)
                csv_file.flush()
                if all(stable.values()):
                    break
            else:
                raise RuntimeError("Both vehicles did not stabilize inside the calibrated speed interval within 45 s")

            ego_v0 = max(0.0, forward_speed_mps(ego))
            lead_v0 = max(0.0, forward_speed_mps(lead))
            if not (CALIBRATED_MIN_SPEED_KMH / 3.6 <= ego_v0 <= CALIBRATED_MAX_SPEED_KMH / 3.6):
                raise RuntimeError(f"Ego initial speed {ego_v0*3.6:.4f} km/h is outside the calibrated profile range")
            if not (CALIBRATED_MIN_SPEED_KMH / 3.6 <= lead_v0 <= CALIBRATED_MAX_SPEED_KMH / 3.6):
                raise RuntimeError(f"Lead initial speed {lead_v0*3.6:.4f} km/h is outside the calibrated profile range")
            if abs(ego_v0 - lead_v0) > 0.15:
                raise RuntimeError(f"Initial vehicle speeds differ by {abs(ego_v0-lead_v0):.3f} m/s; aborting pilot")

            # The initial state is now frozen. Lead receives full braking while
            # ego is controlled only by the PD controller (no Safety Gate).
            episode_start_time = get_snapshot_time(world)
            gap_at_episode_start = bumper_gap_m(ego, lead, ego.get_transform().get_forward_vector())
            ego_start_loc = ego.get_location()
            lead_start_loc = lead.get_location()
            lead_forward = lead.get_transform().get_forward_vector()
            ego_previous_speed = ego_v0
            lead_previous_speed = lead_v0
            previous_time = episode_start_time
            ego_detector = OnsetDetector()
            lead_detector = OnsetDetector()
            gate_brake_detector = OnsetDetector()
            ego_smooth = ""
            stop_stable_count = 0
            min_gap = float("inf")
            collision_detected = False
            last_episode_time = 0.0
            profile_speed_errors = []
            profile_displacement_errors = []
            print("Starting timed episode: lead full brake; ego PD with Safety Gate enabled.")
            print(f"Measured start speeds: ego={ego_v0*3.6:.4f} km/h, lead={lead_v0*3.6:.4f} km/h")
            print(f"Measured initial bumper gap: {bumper_gap_m(ego, lead, ego.get_transform().get_forward_vector()):.4f} m")

            # Safety Gate episode state. A sent emergency-brake command is latched.
            gate_state = "SAFE"
            gate_latched = False
            gate_reason = ""
            invalid_reason = ""
            unsafe_candidate_t_u = None
            active_t_u = None
            t_u_s = None
            t_b_req_s = None
            t_b_applied_s = None
            # Overall first ego deceleration onset (may be caused by PD before Gate).
            t_b_act_s = None
            preexisting_ego_decel_at_tu = None
            ego_pd_brake_at_tu = None
            ego_smoothed_accel_at_tu = None
            unsafe_cancel_count = 0
            invalid_event_count = 0
            gate_triggered = False
            min_predicted_gap = float("inf")
            valid_forecast_ticks = 0
            invalid_forecast_ticks = 0
            stable_stop_count = 0
            stable_safe_stop_count = 0
            last_gap = gap_at_episode_start
            print(
                f"Starting Safety Gate episode: delay={args.gate_delay_s:.3f} s; "
                f"margin={args.safe_margin_m:.3f} m; horizon={args.prediction_horizon_s:.3f} s."
            )
            print("Latched rule: after a Gate brake request, BRAKE_ACTIVE cannot revert to SAFE/UNSAFE/INVALID.")

            for _ in range(int(args.max_episode_s / FIXED_DELTA_SECONDS)):
                sim_now = get_snapshot_time(world)
                episode_elapsed = sim_now - episode_start_time
                gap = bumper_gap_m(ego, lead, ego.get_transform().get_forward_vector())
                ego_speed = max(0.0, forward_speed_mps(ego))
                lead_speed = max(0.0, forward_speed_mps(lead))
                desired_gap, gap_error, closing_speed, pd_accel, pd_throttle, pd_brake = pd_following_control(
                    gap, ego_speed, lead_speed, args.d0_m, args.headway_s,
                    args.pd_kp, args.pd_kd, args.min_accel_mps2, args.max_accel_mps2)

                predicted_min_gap = None
                prediction_valid = False
                current_invalid_reason = ""
                unsafe_pending_age = None

                if not gate_latched:
                    predicted_min_gap, ego_profile_phase, current_invalid_reason = predict_min_gap_emergency(
                        reference_profile, gap, ego_speed, episode_elapsed, lead_speed,
                        horizon_s=args.prediction_horizon_s, step_s=FIXED_DELTA_SECONDS,
                    )
                    if current_invalid_reason:
                        invalid_forecast_ticks += 1
                        invalid_event_count += 1 if gate_state != "INVALID" else 0
                        invalid_reason = current_invalid_reason
                        gate_state = "INVALID"
                        # INVALID is fail-safe before the command is sent; it bypasses
                        # the artificial delay because no reliable safety forecast exists.
                        gate_latched = True
                        gate_triggered = True
                        gate_reason = "INVALID_FAILSAFE"
                        t_b_req_s = episode_elapsed
                        print(
                            f"INVALID at t={episode_elapsed:.3f} s ({invalid_reason}); "
                            "issuing immediate fail-safe full braking."
                        )
                    else:
                        prediction_valid = True
                        valid_forecast_ticks += 1
                        min_predicted_gap = min(min_predicted_gap, predicted_min_gap)
                        if predicted_min_gap <= args.safe_margin_m:
                            if unsafe_candidate_t_u is None:
                                unsafe_candidate_t_u = episode_elapsed
                                active_t_u = episode_elapsed
                                recent_measured_decel = (
                                    isinstance(ego_smooth, (int, float))
                                    and ego_smooth <= DECEL_ONSET_THRESHOLD
                                )
                                preexisting_ego_decel_at_tu = bool(
                                    pd_brake > 0.005 or recent_measured_decel
                                )
                                ego_pd_brake_at_tu = float(pd_brake)
                                ego_smoothed_accel_at_tu = (
                                    float(ego_smooth) if isinstance(ego_smooth, (int, float)) else None
                                )
                                print(
                                    f"UNSAFE detected at t={episode_elapsed:.3f} s; "
                                    f"predicted minimum gap={predicted_min_gap:.3f} m."
                                )
                            gate_state = "UNSAFE_PENDING"
                            unsafe_pending_age = episode_elapsed - unsafe_candidate_t_u
                            if unsafe_pending_age + 1e-9 >= args.gate_delay_s:
                                gate_latched = True
                                gate_triggered = True
                                gate_reason = "UNSAFE"
                                t_u_s = active_t_u
                                t_b_req_s = episode_elapsed
                                gate_state = "BRAKE_ACTIVE"
                                print(
                                    f"Safety Gate request at t={episode_elapsed:.3f} s "
                                    f"(D_SG={t_b_req_s - t_u_s:.3f} s); BRAKE_ACTIVE latched."
                                )
                        else:
                            if unsafe_candidate_t_u is not None:
                                unsafe_cancel_count += 1
                                unsafe_candidate_t_u = None
                                active_t_u = None
                            gate_state = "SAFE"
                else:
                    # Once latched, prediction is telemetry only; it cannot change
                    # the protective state or cancel full braking.
                    gate_state = "BRAKE_ACTIVE"

                if gate_latched:
                    ego_throttle, ego_brake = 0.0, 1.0
                else:
                    # During UNSAFE_PENDING, the ordinary PD controller remains active.
                    ego_throttle, ego_brake = pd_throttle, pd_brake

                ego_control = carla.VehicleControl(throttle=ego_throttle, brake=ego_brake, steer=0.0)
                lead_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
                apply_controls_checked(client, [(ego, ego_control), (lead, lead_control)])
                frame = world.tick()
                sim_time = get_snapshot_time(world)
                episode_elapsed = sim_time - episode_start_time
                dt = sim_time - previous_time
                ego_speed_after = max(0.0, forward_speed_mps(ego))
                lead_speed_after = max(0.0, forward_speed_mps(lead))
                ego_accel = (ego_speed_after - ego_previous_speed) / dt
                lead_accel = (lead_speed_after - lead_previous_speed) / dt
                ego_smooth, ego_onset = ego_detector.update(ego_accel, sim_time)
                lead_smooth, lead_onset = lead_detector.update(lead_accel, sim_time)
                current_gap = bumper_gap_m(ego, lead, ego.get_transform().get_forward_vector())
                min_gap = min(min_gap, current_gap)
                lead_disp = longitudinal_displacement_m(lead_start_loc, lead.get_location(), lead_forward)
                ref_v, ref_s = interpolate_profile(reference_profile, episode_elapsed)
                if ref_v != "":
                    profile_speed_errors.append(lead_speed_after - ref_v)
                    profile_displacement_errors.append(lead_disp - ref_s)
                ego_transform_after = ego.get_transform()
                ego_forward_after = ego_transform_after.get_forward_vector()
                ego_accel_vec = ego.get_acceleration()
                ego_angular_vec = ego.get_angular_velocity()
                ego_carla_long_accel = (
                    ego_accel_vec.x * ego_forward_after.x
                    + ego_accel_vec.y * ego_forward_after.y
                    + ego_accel_vec.z * ego_forward_after.z
                )
                ego_lateral_axis_x = -ego_forward_after.y
                ego_lateral_axis_y = ego_forward_after.x
                ego_carla_lateral_accel = (
                    ego_accel_vec.x * ego_lateral_axis_x
                    + ego_accel_vec.y * ego_lateral_axis_y
                )
                ego_carla_accel_norm = math.sqrt(
                    ego_accel_vec.x ** 2 + ego_accel_vec.y ** 2 + ego_accel_vec.z ** 2
                )
                ego_loc = ego.get_location()
                lead_loc = lead.get_location()
                last_episode_time = episode_elapsed
                ego_applied = ego.get_control()
                lead_applied = lead.get_control()
                if (gate_latched and ego_applied.brake >= 0.999
                        and ego_applied.throttle <= 0.001):
                    # CARLA applies the synchronous control on the tick just completed.
                    if t_b_applied_s is None:
                        t_b_applied_s = episode_elapsed
                    # Separate Gate-phase onset detector: unlike the episode-wide detector,
                    # this one starts only after full-brake readback confirms Gate actuation.
                    gate_brake_detector.update(ego_accel, sim_time)
                    if gate_brake_detector.onset_time is not None and t_b_act_s is None:
                        t_b_act_s = gate_brake_detector.onset_time - episode_start_time
                drain_collision_events(collision_events_queue, collision_events)
                collision_detected = bool(collision_events)
                if unsafe_candidate_t_u is not None and not gate_latched:
                    unsafe_pending_age = episode_elapsed - unsafe_candidate_t_u
                else:
                    unsafe_pending_age = 0.0 if gate_latched else None
                row = {field: "" for field in CSV_FIELDS}
                row.update({
                    "phase": "safety_gate_episode", "frame": frame,
                    "sim_time_s": f"{sim_time:.4f}", "episode_time_s": f"{episode_elapsed:.4f}",
                    "ego_actor_id": ego.id, "lead_actor_id": lead.id,
                    "ego_x_m": f"{ego_loc.x:.5f}", "ego_y_m": f"{ego_loc.y:.5f}", "ego_z_m": f"{ego_loc.z:.5f}",
                    "lead_x_m": f"{lead_loc.x:.5f}", "lead_y_m": f"{lead_loc.y:.5f}", "lead_z_m": f"{lead_loc.z:.5f}",
                    "ego_speed_mps": f"{ego_speed_after:.5f}", "ego_speed_kmh": f"{ego_speed_after*3.6:.4f}",
                    "lead_speed_mps": f"{lead_speed_after:.5f}", "lead_speed_kmh": f"{lead_speed_after*3.6:.4f}",
                    "ego_accel_raw_mps2": f"{ego_accel:.5f}",
                    "ego_accel_smooth_5step_mps2": "" if ego_smooth == "" else f"{ego_smooth:.5f}",
                    "ego_carla_accel_x_mps2": f"{ego_accel_vec.x:.5f}",
                    "ego_carla_accel_y_mps2": f"{ego_accel_vec.y:.5f}",
                    "ego_carla_accel_z_mps2": f"{ego_accel_vec.z:.5f}",
                    "ego_carla_longitudinal_accel_mps2": f"{ego_carla_long_accel:.5f}",
                    "ego_carla_lateral_accel_mps2": f"{ego_carla_lateral_accel:.5f}",
                    "ego_carla_accel_norm_mps2": f"{ego_carla_accel_norm:.5f}",
                    "ego_angular_velocity_x": f"{ego_angular_vec.x:.5f}",
                    "ego_angular_velocity_y": f"{ego_angular_vec.y:.5f}",
                    "ego_angular_velocity_z": f"{ego_angular_vec.z:.5f}",
                    "ego_gear": int(ego_applied.gear),
                    "ego_hand_brake": bool(ego_applied.hand_brake),
                    "ego_reverse": bool(ego_applied.reverse),
                    "ego_manual_gear_shift": bool(ego_applied.manual_gear_shift),
                    "lead_accel_raw_mps2": f"{lead_accel:.5f}",
                    "lead_accel_smooth_5step_mps2": "" if lead_smooth == "" else f"{lead_smooth:.5f}",
                    "bumper_gap_m": f"{current_gap:.5f}", "desired_gap_m": f"{desired_gap:.5f}",
                    "gap_error_m": f"{gap_error:.5f}", "closing_speed_mps": f"{closing_speed:.5f}",
                    "ego_pd_accel_command_mps2": f"{pd_accel:.5f}",
                    "ego_requested_throttle": f"{ego_throttle:.5f}", "ego_requested_brake": f"{ego_brake:.5f}",
                    "ego_applied_throttle": f"{ego_applied.throttle:.5f}", "ego_applied_brake": f"{ego_applied.brake:.5f}",
                    "lead_requested_throttle": "0.00000", "lead_requested_brake": "1.00000",
                    "lead_applied_throttle": f"{lead_applied.throttle:.5f}", "lead_applied_brake": f"{lead_applied.brake:.5f}",
                    "lead_brake_elapsed_s": f"{episode_elapsed:.4f}",
                    "lead_reference_speed_mps": "" if ref_v == "" else f"{ref_v:.5f}",
                    "lead_measured_displacement_m": f"{lead_disp:.5f}",
                    "lead_reference_displacement_m": "" if ref_s == "" else f"{ref_s:.5f}",
                    "lead_speed_error_mps": "" if ref_v == "" else f"{lead_speed_after-ref_v:.5f}",
                    "lead_displacement_error_m": "" if ref_s == "" else f"{lead_disp-ref_s:.5f}",
                    "collision_detected": collision_detected,
                    "gate_eval_time_s": f"{sim_now - episode_start_time:.4f}",
                    "gate_state": "BRAKE_ACTIVE" if gate_latched else gate_state,
                    "gate_reason": gate_reason,
                    "gate_latched": gate_latched,
                    "gate_delay_s": f"{args.gate_delay_s:.4f}",
                    "predicted_min_gap_eb_m": "" if predicted_min_gap is None else f"{predicted_min_gap:.5f}",
                    "prediction_valid": prediction_valid,
                    "invalid_reason": current_invalid_reason,
                    "unsafe_candidate_t_U_s": "" if unsafe_candidate_t_u is None else f"{unsafe_candidate_t_u:.4f}",
                    "t_U_s": "" if t_u_s is None else f"{t_u_s:.4f}",
                    "t_B_req_s": "" if t_b_req_s is None else f"{t_b_req_s:.4f}",
                    "t_B_applied_s": "" if t_b_applied_s is None else f"{t_b_applied_s:.4f}",
                    "t_B_act_s": "" if t_b_act_s is None else f"{t_b_act_s:.4f}",
                    "preexisting_ego_decel_at_tU": "" if preexisting_ego_decel_at_tu is None else preexisting_ego_decel_at_tu,
                    "ego_pd_brake_at_tU": "" if ego_pd_brake_at_tu is None else f"{ego_pd_brake_at_tu:.5f}",
                    "ego_smoothed_accel_at_tU_mps2": "" if ego_smoothed_accel_at_tu is None else f"{ego_smoothed_accel_at_tu:.5f}",
                    "unsafe_pending_age_s": "" if unsafe_pending_age is None else f"{unsafe_pending_age:.4f}",
                })
                writer.writerow(row)
                csv_file.flush()
                if collision_detected:
                    print(f"COLLISION detected at episode t={episode_elapsed:.3f} s")
                    break

                if ego_speed_after <= STOP_SPEED_MPS and lead_speed_after <= STOP_SPEED_MPS:
                    if gate_latched:
                        stable_stop_count += 1
                    elif (abs(pd_accel) <= 0.05 and abs(gap_error) <= 0.10
                          and ego_applied.throttle <= 0.005 and ego_applied.brake <= 0.005):
                        stable_safe_stop_count += 1
                    else:
                        stable_safe_stop_count = 0
                else:
                    stable_stop_count = 0
                    stable_safe_stop_count = 0

                if gate_latched and stable_stop_count >= STOP_STABLE_STEPS:
                    print("Both vehicles met the stable-stop criterion with BRAKE_ACTIVE latched.")
                    break
                if not gate_latched and stable_safe_stop_count >= 100:
                    print("PD-only system reached a stable no-actuation stop; no Gate intervention was needed.")
                    break

                ego_previous_speed = ego_speed_after
                lead_previous_speed = lead_speed_after
                previous_time = sim_time

        drain_collision_events(collision_events_queue, collision_events)
        final_bumper_gap = bumper_gap_m(ego, lead, ego.get_transform().get_forward_vector())
        summary = {
            "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
            "stage": "Baseline C / Safety Gate / configurable artificial delay",
            "client_version": client.get_client_version(),
            "server_version": client.get_server_version(),
            "map": map_name,
            "spawn_index": args.spawn_index,
            "vehicle_blueprint": args.vehicle,
            "fixed_delta_seconds": FIXED_DELTA_SECONDS,
            "no_rendering_mode": True,
            "weather": "ClearNoon",
            "warmup_frame": int(warmup_frame),
            "target_speed_kmh": args.target_speed_kmh,
            "ego_initial_speed_kmh": ego_v0 * 3.6,
            "lead_initial_speed_kmh": lead_v0 * 3.6,
            "calibrated_speed_range_kmh": [CALIBRATED_MIN_SPEED_KMH, CALIBRATED_MAX_SPEED_KMH],
            "requested_initial_bumper_gap_m": args.initial_gap_m,
            "measured_initial_bumper_gap_m": gap_at_episode_start,
            "final_bumper_gap_m": final_bumper_gap,
            "desired_gap_at_start_m": args.d0_m + args.headway_s * ego_v0,
            "ego_vehicle_physics": ego_physics,
            "lead_vehicle_physics": lead_physics,
            "tire_friction_readback_all_wheels": tire_values,
            "lead_brake_command": 1.0,
            "safety_gate": {
                "delay_s": args.gate_delay_s,
                "safe_margin_m": args.safe_margin_m,
                "prediction_horizon_s": args.prediction_horizon_s,
                "gate_triggered": gate_triggered,
                "gate_latched": gate_latched,
                "gate_reason": gate_reason or None,
                "invalid_reason": invalid_reason or None,
                "invalid_event_count": invalid_event_count,
                "unsafe_cancel_count": unsafe_cancel_count,
                "valid_forecast_ticks": valid_forecast_ticks,
                "invalid_forecast_ticks": invalid_forecast_ticks,
                "minimum_predicted_gap_m": min_predicted_gap if math.isfinite(min_predicted_gap) else None,
                "t_U_s": t_u_s,
                "t_B_req_s": t_b_req_s,
                "t_B_applied_s": t_b_applied_s,
                # Existing detector timestamp is retained as the overall first ego deceleration
                # onset, not attributed to Gate unless it follows t_U.
                "t_B_act_s": t_b_act_s,
                "D_SG_s": (t_b_req_s - t_u_s) if t_b_req_s is not None and t_u_s is not None else None,
                "D_req_to_applied_s": (t_b_applied_s - t_b_req_s) if t_b_applied_s is not None and t_b_req_s is not None else None,
                "D_total_s": (t_b_act_s - t_u_s) if (
                    t_b_act_s is not None and t_u_s is not None
                    and t_b_act_s >= t_u_s and preexisting_ego_decel_at_tu is False
                ) else None,
                "preexisting_ego_decel_at_tU": preexisting_ego_decel_at_tu,
                "ego_pd_brake_at_tU": ego_pd_brake_at_tu,
                "ego_smoothed_accel_at_tU_mps2": ego_smoothed_accel_at_tu,
                "t_B_act_definition": "First detected ego deceleration onset after Gate full-brake command readback; D_total is omitted if ego was already decelerating at t_U, because a distinct Gate-caused onset cannot then be isolated by this detector.",
                "latch_rule": "After Gate sends full-brake command, BRAKE_ACTIVE remains latched and cannot revert to SAFE/UNSAFE/INVALID.",
                "invalid_policy": "INVALID before a brake request triggers immediate fail-safe full braking; it bypasses artificial delay.",
                "ego_profile_prediction": "The current ego speed is phase-matched to the remaining tail of the full-brake profile selected from the in-range episode-start speed; this assumption requires pilot validation.",
            },
            "ego_pd_parameters": {
                "d0_m": args.d0_m, "headway_s": args.headway_s,
                "kp_s_minus_2": args.pd_kp, "kd_s_minus_1": args.pd_kd,
                "acceleration_limits_mps2": [args.min_accel_mps2, args.max_accel_mps2],
                "throttle_mapping": "max(0, min(1, a_cmd/2)) when a_cmd >= 0",
                "brake_mapping": "max(0, min(1, -a_cmd/6)) when a_cmd < 0",
            },
            "ego_deceleration_onset_estimate_s": ego_detector.onset_time - episode_start_time if ego_detector.onset_time is not None else None,
            "lead_deceleration_onset_estimate_s": lead_detector.onset_time - episode_start_time if lead_detector.onset_time is not None else None,
            "min_bumper_gap_m": min_gap if math.isfinite(min_gap) else None,
            "collision_detected": bool(collision_events),
            "collision_events": collision_events,
            "episode_duration_s": last_episode_time,
            "lead_profile_speed_mae_mps": (sum(abs(x) for x in profile_speed_errors) / len(profile_speed_errors)) if profile_speed_errors else None,
            "lead_profile_displacement_mae_m": (sum(abs(x) for x in profile_displacement_errors) / len(profile_displacement_errors)) if profile_displacement_errors else None,
            "reference_profile_csv": os.path.abspath(profile_path) if profile_path else None,
            "csv_output": os.path.abspath(args.output),
            "ego_acceleration_diagnostics": "CARLA-reported world acceleration vector plus longitudinal/lateral projection; gear, hand_brake, reverse and manual_gear_shift are logged per tick.",
            "note": f"Pilot result. Vehicle physics remains unchanged. The 22.53 m initial gap and PD parameters are provisional experimental inputs. A no-trigger result means this scenario did not enter UNSAFE under the configured M_safe={args.safe_margin_m:.1f} m predicted-gap criterion.",
        }
        with open(args.summary, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print("\nSafety Gate pilot finished.")
        print("Gate triggered:", summary["safety_gate"]["gate_triggered"])
        print("Gate reason:", summary["safety_gate"]["gate_reason"])
        print("Gate latched:", summary["safety_gate"]["gate_latched"])
        print("Collision:", summary["collision_detected"])
        print("Minimum bumper gap:", summary["min_bumper_gap_m"])
        print("Leader profile speed MAE:", summary["lead_profile_speed_mae_mps"])
        print("Leader profile displacement MAE:", summary["lead_profile_displacement_mae_m"])
        print("Timestep CSV:", os.path.abspath(args.output))
        print("Summary JSON:", os.path.abspath(args.summary))

    finally:
        for sensor in sensors:
            try:
                sensor.stop()
            except Exception:
                pass
        for actor in reversed(actors + sensors):
            try:
                actor.destroy()
            except Exception:
                pass
        # Preserve the same stopped-server policy as the calibration script.
        try:
            current = world.get_settings()
            current.synchronous_mode = True
            current.fixed_delta_seconds = FIXED_DELTA_SECONDS
            current.no_rendering_mode = True
            world.apply_settings(current)
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
