# Safety Gate: Braking Calibration in CARLA

## Project overview

This repository contains the code and experimental data for braking calibration of a simulated vehicle in CARLA.

The calibration experiments characterize the braking response of a `vehicle.lincoln.mkz_2017` vehicle at different initial speeds. The resulting time series are intended to support the analysis of emergency braking and collision outcomes in a vehicle-following scenario with an artificial Safety Gate reaction delay.

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

The calibration script uses a speed-control phase to reach and stabilize the target speed, followed by a full-brake command. Vehicle physics parameters are not modified by the calibration procedure.

## Repository structure

```text
safety-gate-carla-study/
├── README.md
└── calibration/
    ├── code/
    │   ├── carla_brake_calibration_multispeed_v3.py
    │   └── reanalyze_calibration_braking_onset.py
    ├── data/
    │   ├── calibration_v3_repeat3_timestep.csv
    │   ├── calibration_v3_repeat3_summary.csv
    │   ├── calibration_v3_repeat3_metadata.json
    │   ├── calibration_v3_repeat3_onset_reanalysis.csv
    │   └── lincoln_calibration_v3_onset_reanalysis.csv
    └── figures/
```

The `lincoln_calibration_v3_onset_reanalysis.csv` file contains an earlier analysis of the original single-run calibration series. The `calibration_v3_repeat3_onset_reanalysis.csv` file contains the reanalysis of the three-repeat dataset.

## Code

### Calibration

`calibration/code/carla_brake_calibration_multispeed_v3.py`

Runs the multi-speed braking calibration and records time-series data, run summaries, and simulation metadata.

Example command:

```powershell
python carla_brake_calibration_multispeed_v3.py --map Town04_Opt --vehicle vehicle.lincoln.mkz_2017 --speeds-kmh 20,30,40,50 --repeats 3 --spawn-index 90 --speed-tolerance-kmh 1 --settle-time-s 1 --approach-kp 0.8 --approach-ki 0.25 --approach-max-brake 0.15 --expected-tire-friction 3.5 --output calibration_v3_repeat3_timestep.csv --summary calibration_v3_repeat3_summary.csv --metadata calibration_v3_repeat3_metadata.json
```

Run the command from an environment where CARLA 0.9.16 and its Python API are available.

### Braking onset reanalysis

`calibration/code/reanalyze_calibration_braking_onset.py`

Performs post-processing of the recorded calibration data. It estimates braking onset using a five-step speed-trend window (0.05 s), a deceleration threshold of -0.5 m/s², and two consecutive qualifying windows.

Example command:

```powershell
python reanalyze_calibration_braking_onset.py --timestep calibration_v3_repeat3_timestep.csv --summary calibration_v3_repeat3_summary.csv --output calibration_v3_repeat3_onset_reanalysis.csv
```

This is a post-processing operation and does not launch CARLA.

## Data files

### Time-series CSV

`calibration_v3_repeat3_timestep.csv`

Contains time-series measurements and control information for each calibration run.

Important fields include:

- `run_id` — identifier of the run;
- `target_speed_kmh` — target initial speed;
- `repeat` — repetition number;
- `phase` — experimental phase (`speed_control` or `brake`);
- `sim_time_s` — simulation time;
- `speed_mps`, `speed_kmh` — vehicle speed;
- `longitudinal_accel_mps2` — longitudinal acceleration estimated from changes in speed;
- `distance_from_brake_start_m` — longitudinal displacement from the beginning of braking;
- `requested_throttle`, `requested_brake` — requested control inputs;
- `applied_throttle`, `applied_brake` — applied control inputs;
- `brake_command_time_s` — time of the emergency brake command;
- `observed_decel_onset_time_s` — estimated onset of deceleration.

Acceleration is calculated from the speed time series using finite differences; it is not a direct accelerometer measurement. The recorded distance is displacement from the braking start, not absolute vehicle position.

### Summary CSV

`calibration_v3_repeat3_summary.csv`

Contains run-level calibration results, including initial speed, stopping time, stopping distance, and deceleration-related values.

### Metadata JSON

`calibration_v3_repeat3_metadata.json`

Contains simulation and vehicle configuration metadata recorded during calibration.

### Braking onset analysis CSV

`calibration_v3_repeat3_onset_reanalysis.csv`

Contains braking onset estimates for all 12 runs, calculated using the post-processing method described above.

## Interpretation and limitations

Maximum deceleration values are transient peak measurements and must not be interpreted as constant deceleration throughout the braking process.

The braking trajectory should be evaluated using the recorded speed and displacement time series, rather than by assuming that the maximum measured deceleration remains constant.

Braking onset estimates are method-dependent. A reported delay of 0.00 s means that deceleration was detected within the temporal resolution and criteria of the analysis; it does not establish zero physical response latency.

The three repetitions are used to assess the reproducibility of the simulation procedure. They are repeated simulation runs, not independent real-world experimental samples.

## Reproducibility

The scripts and recorded data are provided to document the calibration procedure and allow the analysis to be repeated. The simulator version, map, vehicle blueprint, time step, and other configuration parameters should be kept consistent when reproducing the experiments.

