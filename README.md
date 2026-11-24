# Safety Gate: Braking Calibration in CARLA

## Project overview

This repository contains the code and experimental data for two parts of the Safety Gate study:
(1) braking-response calibration of a simulated vehicle in CARLA and
(2) a two-vehicle pilot evaluating the Safety Gate approach in a vehicle-following scenario.

The calibration experiments characterize the braking response of a `vehicle.lincoln.mkz_2017` vehicle at four target initial speeds: 20, 30, 40, and 50 km/h. Three runs were performed at each target speed, giving 12 calibration runs in total.

The resulting time series are intended to support the analysis of emergency braking and collision outcomes in a vehicle-following scenario with an artificial Safety Gate reaction delay.

All experiments described here were conducted in simulation. The parameters and results should not be interpreted as real-world vehicle specifications.

## Simulation configuration

| Parameter | Configuration |
|---|---|
| Simulator | CARLA 0.9.16 |
| Map | Town04_Opt |
| Vehicle blueprint | `vehicle.lincoln.mkz_2017` |
| Spawn point index | 90 |
| Simulation mode | Synchronous |
| Fixed time step | 0.01 s |
| Weather | ClearNoon |
| Tire friction coefficient | 3.5 (read-back value for all wheels) |
| Target initial speeds | 20, 30, 40, 50 km/h |
| Repetitions | 3 per target speed |
| Total calibration runs | 12 |

Each calibration run is executed in a separate Python process. The script loads the simulation world, configures synchronous mode and the fixed time step, creates the vehicle, stabilizes its speed near the target value, and then applies a full-brake command.

Vehicle physics parameters are not modified by the calibration procedure.

The target speeds are control setpoints. For quantitative analysis, the measured vehicle speed at the brake-command time should be used as the actual initial braking speed.

## Repository structure

```text
safety-gate-carla-study/
│
├── calibration/
│   ├── code/
│   └── data/
│
├── pilot/
│   ├── code/
│   └── data/
│
├── README.md
└── .gitignore
```

The `data` directory contains separate time-series, summary, and metadata files for each of the 12 runs. The filename identifies the target speed and repetition number.

Earlier calibration datasets and their analyses, if retained in the repository, should be treated as separate historical datasets and not combined with the 12 runs described above without an explicit analysis procedure.

## Code

### Deterministic calibration

`calibration/code/carla_brake_calibration_multispeed_v4_3_deterministic.py`

Runs one calibration experiment in a separate Python process. The script configures the CARLA world, stabilizes the vehicle near the selected target speed, applies full braking, and saves the time-series data, run summary, and simulation metadata.

The 12-run series is produced by launching the script separately for each target-speed and repetition combination.

Run the script from an environment where CARLA 0.9.16 and its Python API are available. Consult the script's command-line arguments for the exact options used for an individual run.

### Braking onset reanalysis

`calibration/code/reanalyze_calibration_braking_onset.py`

Performs post-processing of the recorded calibration data. The selected onset estimator uses:

- a five-step speed-trend window, corresponding to 0.05 s at the 0.01 s simulation time step;
- a deceleration threshold of -0.5 m/s²;
- two consecutive qualifying windows to confirm the detection.

The onset is assigned retrospectively to the first qualifying window after the second qualifying window confirms the trend. Therefore, this estimate is a post-processing measure and should not be interpreted as an online detection time or as a direct measurement of physical brake-system latency.

This is a post-processing operation and does not launch CARLA.

## Data files

### Time-series CSV

Each `manual_<speed>_r<repeat>_timestep.csv` file contains the time-series measurements and control information for one calibration run.

Important fields include:

- `run_id` — identifier of the run;
- `vehicle_blueprint` — vehicle blueprint;
- `map` — CARLA map;
- `spawn_index` — vehicle spawn point index;
- `target_speed_kmh` — target initial speed;
- `repeat` — repetition number;
- `phase` — experimental phase (`speed_control` or `brake`);
- `frame` — simulation frame;
- `sim_time_s` — simulation time;
- `speed_mps`, `speed_kmh` — vehicle speed;
- `longitudinal_accel_mps2` — longitudinal acceleration estimated from changes in speed;
- `distance_from_brake_start_m` — longitudinal displacement from the beginning of braking;
- `requested_throttle`, `requested_brake` — requested control inputs;
- `applied_throttle`, `applied_brake` — applied control inputs;
- `brake_command_time_s` — time of the full-brake command;
- `observed_decel_onset_time_s` — estimated onset of deceleration, where available.

Acceleration is calculated from the speed time series using finite differences; it is not a direct accelerometer measurement. The recorded distance is displacement from the braking start, not absolute vehicle position.

### Summary CSV

Each `manual_<speed>_r<repeat>_summary.csv` file contains run-level calibration results, including the measured initial speed, stopping time, stopping distance, and deceleration-related values.

### Metadata JSON

Each `manual_<speed>_r<repeat>_metadata.json` file contains the simulation and vehicle configuration metadata recorded during the corresponding calibration run.

### Braking onset analysis

The braking onset reanalysis produces derived onset estimates for the individual calibration runs. These estimates depend on the selected window and threshold criteria and should be interpreted accordingly.

## Interpretation and limitations

Maximum deceleration values are transient peak measurements and must not be interpreted as constant deceleration throughout the braking process.

The braking trajectory should be evaluated using the recorded speed and displacement time series, rather than by assuming that the maximum measured deceleration remains constant.

Braking onset estimates are method-dependent. A reported delay of 0.00 s means that deceleration was detected within the temporal resolution and criteria of the analysis; it does not establish zero physical response latency.

The three repetitions at each target speed are repeated simulation runs intended to support an assessment of procedural repeatability. Quantitative repeatability conclusions require comparison of the measured initial speeds, braking trajectories, stopping times, and stopping distances across repetitions.

The calibration profiles are intended for subsequent Safety Gate forecasting. Their use for forecasting requires a separate validation of the profile construction and interpolation procedure. Calibration data alone do not establish the accuracy of future collision predictions.

## Two-vehicle pilot

The two-vehicle pilot evaluates the Safety Gate approach in a simulated vehicle-following scenario.

The retained pilot dataset consists of five selected cases:

- A: PD-only baseline (30-second run);
- B: immediate-braking baseline;
- C: Safety Gate run using version 1.2;
- D: Safety Gate run using version 1.2.1;
- INVALID: invalid case used in the pilot analysis.

### Pilot code

The following scripts are retained for the selected pilot experiments:

| Script | Purpose |
|---|---|
| `carla_two_vehicle_pd_pilot.py` | Baseline A: PD-only |
| `carla_two_vehicle_baseline_immediate_brake.py` | Baseline B: immediate braking |
| `carla_two_vehicle_safety_gate_v1_2.py` | Safety Gate run C |
| `carla_two_vehicle_safety_gate_v1_2_1.py` | Safety Gate runs D and INVALID |
| `summarize_two_vehicle_pd_pilot_v2.py` | Analysis of A |
| `analyze_two_vehicle_baseline.py` | Analysis of B |
| `analyze_safety_gate_run_v1_2.py` | Analysis of C, D, and INVALID |

### Pilot data

Each retained pilot case is represented by a JSON summary and a CSV time-series file.

| Case | Summary | Time series |
|---|---|---|
| A | `two_vehicle_pd_only_30s_summary.json` | `two_vehicle_pd_only_30s_timestep.csv` |
| B | `baseline_b_summary.json` | `baseline_b_timestep.csv` |
| C | `baseline_c_v12_summary.json` | `baseline_c_v12_timestep.csv` |
| D | `baseline_d_05s_summary.json` | `baseline_d_05s_timestep.csv` |
| INVALID | `baseline_invalid_summary.json` | `baseline_invalid_timestep.csv` |

The braking profile `leader_braking_profile_50kmh.csv`, stored in `calibration/data/`, is also used in the pilot.

The pilot dataset contains the selected runs used in the reported analysis. Earlier exploratory runs, intermediate versions, and diagnostic datasets are not part of the retained pilot dataset.

## Reproducibility

The scripts and recorded data are provided to document the calibration procedure and allow the analysis to be repeated.

For reproducibility, keep the CARLA version, map, vehicle blueprint, spawn point, synchronous mode, fixed time step, weather, and vehicle configuration consistent. Each calibration run should be launched in a separate Python process using the documented configuration.

The target speed is a setpoint; analyses should use the measured speed at the brake-command time. Any comparison between calibration runs should retain the individual run identifiers and repetition numbers.
## Post-review reproducible calibration profile build

The calibration campaign is complete. **Do not rerun CARLA calibration** to create the following files. They are post-processing outputs made from the 12 retained `manual_<speed>_r<repeat>_timestep.csv` and paired summary CSV files:

```bash
python calibration/code/build_calibration_profiles.py
```

The builder writes these files into `calibration/data/`:

- `calibration_12_run_summary.csv` — exactly one record per original run file. It preserves the source timestep filename, paired summary filename, SHA-256 hashes, and repetition number parsed from the filename. The internal `run_id` is retained for reference but is not used as a unique key, because it is reused in some of the separated files.
- `braking_profiles_4speeds_mean.csv` — mean speed/displacement profiles for the four calibrated starting-speed groups, time-aligned to the brake command. These profiles are prepared for the future multi-speed implementation.
- `profile_interpolation_validation.csv` — leave-one-speed-out interpolation results for the internal 30 and 40 km/h points, including both the held-out stopping window and the extended neighbouring-profile window.
- `calibration_profile_validation.json` — provenance and the two-vehicle forecast-gap error bound consumed by the post-review pilot runner.

The profile builder is a deterministic post-processing program; it does not connect to CARLA. The original single-speed file `leader_braking_profile_50kmh.csv` is retained unchanged so that the historical A–D runs remain traceable.

### Forecast-error criterion

For an individual vehicle, let `e_i(tau)` be the absolute displacement error between an interpolated profile and the held-out calibration profile. The maximum observed error in the extended internal hold-out checks is 0.898051 m. A conservative bound for the forecast bumper-gap error is therefore

`E_gap <= |e_lead| + |e_ego| <= 2 * 0.898051 = 1.796102 m`.

The preliminary acceptance allowance is rounded upward to 1.80 m. The bound is limited to the two internal hold-out checks and the tested profile time span (up to 1.42 s, with displacement held constant after stopping); it is not a statistical confidence bound and does not validate extrapolation. A profile-validation bound that is missing, invalid, or greater than 1.80 m must lead to `INVALID` and immediate fail-safe braking. When a bound is accepted, the Gate compares the lower bound `predicted_min_gap - E_gap` with `M_safe`, rather than comparing the raw predicted gap alone.

### Post-review pilot runner

`pilot/code/carla_two_vehicle_safety_gate_v1_2_2.py` is the post-review runner for the next CARLA pilot. It adds:

- integer-tick delay handling (0.50 s = exactly 50 ticks at 0.01 s; non-grid-aligned delays are rejected);
- profile-validation metadata checks and conservative two-vehicle error subtraction;
- a defensive `INVALID` result for missing/unknown lead-braking age;
- separate causal `t_B_act_s` and diagnostic `t_B_act_diagnostic_s` fields;
- a root-relative default path to `calibration/data/leader_braking_profile_50kmh.csv` and the profile-validation JSON;
- a 4.0 m default `M_safe`, matching the documented pilot C/D configuration.

For the next content-focused pilot, the preliminary screening proposal is to retain the calibrated single-speed target (49.2787 km/h) and use one common initial bumper gap of 6.0 m. The value is a screening candidate slightly above `M_safe + E_gap,max = 5.8 m`; it is **not yet validated in CARLA**. First verify the physical-preventability control at the same 6.0 m gap, then compare zero-delay Gate and integer-tick delays (for example 0.10, 0.20, 0.30, 0.40, and 0.50 s). Keep the 6.0 m gap identical across all compared runs. If the event is not physically preventable or no meaningful UNSAFE event occurs during active lead braking, revise the single common gap before the factor series—not separately for each speed. The current runner remains single-speed; do not use it for a multi-speed series until separate ego/lead profile selection is implemented.

Example command from the repository root (one run at the proposed 6.0 m gap):

```bash
python pilot/code/carla_two_vehicle_safety_gate_v1_2_2.py \
  --reference-profile calibration/data/leader_braking_profile_50kmh.csv \
  --profile-validation-json calibration/data/calibration_profile_validation.json \
  --target-speed-kmh 49.2787 --initial-gap-m 6.0 \
  --safe-margin-m 4.0 --gate-delay-s 0.50
```

Important implementation boundary: v1.2.2 still loads **one fixed 50 km/h profile** for the present single-speed pilot. The new four-speed mean-profile CSV is not automatically selected by separate ego/lead initial speeds. Multi-speed profile selection and speed interpolation remain a subsequent implementation task. The historical A–D and INVALID results in `pilot/data/` were generated before this post-review runner and are not retroactively recalculated with its conservative error correction.

### Reproducing the active-window leader-profile metrics

```bash
python pilot/code/analyze_lead_profile_active_window.py
```

This produces `pilot/data/lead_profile_active_window_comparison.csv` for A, C, and D on the common first 1.42 s active braking window. The saved A file does not contain CARLA `get_acceleration()` or gear telemetry columns; claims based on those channels must therefore be attributed to a separate instrumented diagnostic run, not to the archived A CSV.

### Helper tests

The pure helper tests do not require CARLA:

```bash
python -m unittest discover -s pilot/tests -v
```

They cover exact 50-tick timing, zero delay, rejection of delays that are not integer multiples of the fixed step, conversion of unknown lead-braking start time into an `INVALID` reason, loading of the profile-validation bound, rejection of missing validation metadata, and treating a missing braking-profile file as an empty profile so the episode can enter `INVALID` rather than aborting before the episode.

The historical v1.2.1 runner and saved A–D results are retained for provenance. Use v1.2.2 for the next pilot only after checking the chosen scenario in CARLA; the post-review runner has not itself been executed in CARLA in this repository build environment.
