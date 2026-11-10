# Safety Gate: Braking Calibration in CARLA

## Project overview

This repository contains the code and experimental data for braking calibration of a simulated vehicle in CARLA.

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
├── README.md
└── calibration/
    ├── code/
    │   ├── carla_brake_calibration_multispeed_v4_3_deterministic.py
    │   ├── reanalyze_calibration_braking_onset.py
    │   └── analyze_calibration_timeseries_v2.py
    ├── data/
    │   ├── manual_20_r1_timestep.csv
    │   ├── manual_20_r1_summary.csv
    │   ├── manual_20_r1_metadata.json
    │   ├── ...
    │   ├── manual_50_r3_timestep.csv
    │   ├── manual_50_r3_summary.csv
    │   ├── manual_50_r3_metadata.json
    │   └── ...
    └── figures/
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

### Time-series analysis

`calibration/code/analyze_calibration_timeseries_v2.py`

Analyzes the recorded calibration time series and produces derived braking metrics and visualizations. Acceleration and displacement values are derived from the recorded speed and time data; they are not direct measurements from dedicated acceleration or position sensors.

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

## Reproducibility

The scripts and recorded data are provided to document the calibration procedure and allow the analysis to be repeated.

For reproducibility, keep the CARLA version, map, vehicle blueprint, spawn point, synchronous mode, fixed time step, weather, and vehicle configuration consistent. Each calibration run should be launched in a separate Python process using the documented configuration.

The target speed is a setpoint; analyses should use the measured speed at the brake-command time. Any comparison between calibration runs should retain the individual run identifiers and repetition numbers.