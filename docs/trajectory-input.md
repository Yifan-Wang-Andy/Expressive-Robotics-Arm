# Trajectory input and waypoint conversion

The replay script accepts a **reference directory, named SO-101 CSV, or Bandai V2 SO-101 NPZ**, passed
as `--trajectory PATH`. A reference directory contains `trajectory.npz` with the
sampled motion and `metadata.json` describing the motion and robot. CSV input is
prepared on the configured control grid before replay. A JSON waypoint file is
an input to the converter, not a direct input to replay. A Bandai V2 NPZ uses its
own source format and requires the original package metadata described below.

## Read Bandai V2 exact trajectories

Keep `Bandai_V2_Optimized_2026-11-16` outside this repository. Its `index.csv`
lists motions by dataset, action, style, take, and robot. Choose the `so101`
row's `exact_trajectory_npz` path; Gen3 trajectories cannot drive this SO-101.

```sh
export BANDAI_DATASET_ROOT="$HOME/Documents/Bandai_V2_Optimized_2026-11-16"
TRAJECTORY="$BANDAI_DATASET_ROOT/motions/dataset_1/dataset-1_bow_tired_001/so101.npz"

python3 scripts/control_so101.py --trajectory "$TRAJECTORY" --check
bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --trajectory "$TRAJECTORY" --start-delay 5
```

Set `ISAAC_SIM_PATH` before the simulator command. To switch motions, change
only `TRAJECTORY`. The runnable `Standard_cmd.txt` also accepts a full trajectory
path as its first argument, or a package-relative path through the `MOTION`
environment variable. Its default selects the tired-bow example above. Do not
edit the Isaac Sim launch helper to choose a data file.

The source package must retain this structure:

```text
Bandai_V2_Optimized_2026-11-16/
  motions/dataset_N/SOURCE_STEM/so101.npz
  motions/dataset_N/SOURCE_STEM/so101.json
  provenance/robot_models/so101/waypoint_model.json
  provenance/robot_models/so101/model.urdf
```

The importer loads NPZ arrays with pickle disabled and evaluates the supplied
cubic joint polynomial plus its local robot clock with NumPy only. It does not
execute the package's reader code. Source arrays are `path_c`, `path_x`, `grid`,
`squared_speed`, `clock`, and `joint_names`; these are distinct from this
project's `so101-trajectory/1` arrays. Joint radians are mapped by name. The
source URDF hash must match both the package model and the configured replay
model. Sidecar units and frame metadata are retained, including `base_link`
and `gripper_frame_link`. The fixed gripper angle is read from
`waypoint_model.json` (**0.35 rad** for this delivery).

The robot clock is nonlinear relative to source progress. No uniform slowdown
factor is substituted. Preparation samples the exact source curve on the
configured control grid (60 Hz by default), then pads the final pose to the
next tick. The simulator starts at the first pose without an approach from
zero and holds each command for one control interval, including the last row.

Continuous interior position, velocity, and acceleration are checked before
the existing sampled checks, including start/end rest boundaries. No source
positions, timing, limits, or margins are silently changed. Some supplied clips
are incompatible with the current replay envelope: `dataset-1_kick_normal_001`
exceeds the 1.5 rad/s cap, and `dataset-1_dance-long_normal_001` violates the
0.02 rad position margin. The package's interior rate certificate excludes
clip-end stops; endpoint velocities need not be zero. A successful source
export therefore does not guarantee replay acceptance.

Replay archives the source NPZ, required provenance, and prepared reference
under the ignored `runs/so101-.../reference/` directory. Saved prepared references
can subsequently be replayed without the original package. Keep the external
dataset and its contents out of Git. Collision/workspace, jerk, measured
acceleration, and expressive validity remain unchecked; dataset eligibility
remains false.

For a **CSV preview** of the same package, select `so101.csv` and pass
`--csv-gripper-rad 0.35`. CSV preparation linearly interpolates the sampled
waypoints using `robot_time_s`; this changes derivatives and does not reproduce
the supplied continuous curve or inherit its certificate. A bare CSV does not
verify source calibration or tool-coordinate frames.

## Read CSV trajectories

Place source files in `data/` beside `src/` and run from the repository root:

```sh
# Read raw values and report both timelines; no robot assets or Isaac Sim needed.
python3 scripts/read_trajectory.py data/dataset-1_kick_normal_001_right_so101_option-9.csv
python3 scripts/read_trajectory.py data/dataset-1_kick_normal_001_right_gen3_option-9.csv

# Prepare and validate the SO-101 input without opening Isaac Sim or saving files.
python3 scripts/control_so101.py \
  --trajectory data/dataset-1_kick_normal_001_right_so101_option-9.csv --check
```

The reader requires `source_time_s` and `robot_time_s`. Both contain finite,
nonnegative, strictly increasing times in seconds. At least two data rows are
required. Joint columns are identified by header name, so column order may vary:

| Layout | Joint columns | Supported use |
|---|---|---|
| SO-101 | `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`; optional `gripper` | Inspection and replay preparation |
| Gen3 | `joint_1` through `joint_7` | Inspection only; no mapping to the SO-101 |

Joint values are already **radians** and are preserved without degree conversion,
angle offsets, wrapping, or clipping. Optional `tool_x_m`, `tool_y_m`, and
`tool_z_m` must appear together and are read separately in meters. Their frame
and measurement provenance are unspecified; they are source context, not joint
commands or verified simulator measurements. Missing, duplicate, or unknown
columns, ragged rows, and nonfinite cells are rejected. Input is bounded to
256 MiB and 1,000,000 rows; resampling is limited to 1,000,000 output samples.

Playback uses **`robot_time_s`**, as selected for this dataset. Its initial time
is subtracted so the prepared reference starts at zero. `source_time_s` is
preserved as context. The reader linearly interpolates joint radians onto the
configured uniform control grid, **60 Hz** by default. It extends the prepared
timeline to the first control tick at or after the last CSV timestamp, setting
that final row to the last CSV pose. Metadata records the extension as
`end_padding_s`. Replay holds each target until the next control tick and holds
the terminal row for its usual one control interval;
there is no automatic speed adjustment or smoothing. Linear interpolation can
produce velocity jumps at source knots, so continuous acceleration is unchecked.

The five SO-101 arm joints are mapped by name. An omitted gripper is explicitly
filled with zero by default. Use `--csv-gripper-rad VALUE` to select another
fixed angle, such as `0.35` for Bandai V2; a supplied gripper column must remain
at that selected value. The angle must satisfy the same position limits and
margin as other joints. CSV references record
`initial_pose_policy: "first_sample"`: the simulator initializes directly at the
first CSV pose before advancing physics, including when that pose is nonzero.
No approach motion from zero is inserted. Existing references without this
field retain the zero-initial-pose policy.

Replay checks every original source position and linear interval slope against
the configured/model limits, then checks the resampled positions and finite
differences, including start/end rest boundaries. Reading successfully does not
imply replay eligibility. The supplied SO-101 file has **358 rows** over
**110.914143210 robot seconds** and fits the position limits. On 2026-11-05,
the project reference limits in `configs/so101-demo.json` were raised at the
user's request to **1.5 rad/s** and **40 rad/s²**. Its `--check` command now
passes, with sampled peaks of **1.223521 rad/s** and **32.702266 rad/s²**.
URDF limits and tracking checks remain enforced. These settings are project
simulation thresholds, not manufacturer ratings or a continuous acceleration
bound; no simulator replay has been verified for this CSV. The dummy generator
retains its separate, lower limits in `configs/dummy-trajectory.json`.

For an SO-101 CSV that passes `--check`, set `ISAAC_SIM_PATH` as described in the
README and launch it directly, using that validated file's path:

```sh
bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --trajectory data/your-validated-so101.csv
```

Each CSV replay archives the prepared `trajectory.npz`, its `metadata.json`, and
an unchanged `source.csv` under `runs/so101-.../reference/`. The source copy
retains both clocks and tool coordinates. Metadata records its SHA-256, selected
clock, time origin, interpolation, end padding, gripper policy, and initial-pose
policy. CSVs do not identify a URDF revision or calibration: the selected replay
asset is recorded, while source asset identity/calibration remains unverified.
Collision, workspace, jerk, and measured dynamics remain unchecked; dataset
eligibility remains false. User CSVs directly under `data/` are ignored by Git.

## Write a waypoint file

Use the versioned JSON format below. The complete runnable example is
[`configs/example-waypoints.json`](../configs/example-waypoints.json).

```json
{
  "schema_version": "so101-waypoints/1",
  "joint_names": [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper"
  ],
  "waypoints": [
    {"time_s": 0.0, "positions_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
    {"time_s": 4.0, "positions_rad": [0.2, 0.0, 0.0, 0.0, 0.1, 0.0]},
    {"time_s": 8.0, "positions_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
  ]
}
```

This moves the shoulder pan and wrist roll together to the middle pose over
four seconds, then returns over four seconds. All other joints hold zero.
Angles are absolute joint positions in the selected `so101_new_calib.urdf`
calibration, not changes relative to the preceding waypoint.

| Field | Required meaning |
|---|---|
| `schema_version` | Exactly `so101-waypoints/1` |
| `joint_names` | All six names shown above, each exactly once; any order is accepted |
| `waypoints` | At least two waypoint objects |
| `time_s` | Absolute time in seconds from the beginning of the motion |
| `positions_rad` | Six finite numeric joint angles in radians, in `joint_names` order |

Rules:

- The first time must be zero and later times must strictly increase. Every
  waypoint must fall on the configured control-rate grid. At the default 60 Hz,
  use times that are multiples of 1/60 second. Roundoff of at most `1e-8` control
  steps is accepted; off-grid times are rejected rather than retimed.
- All six initial angles must be zero. This matches replay initialization;
  replay sets that first pose before control begins. A nonzero first target
  must appear as a later waypoint so its transition from zero is included.
- The gripper angle must remain zero at every waypoint. Gripper motion is not
  supported by the current replay policy.
- Include all six positions in every row. Omitted angles are not inferred.
  The final pose may be nonzero; returning to the start is optional.
- Repeat a pose at a **later** time to create a hold. Duplicate timestamps are
  invalid. Interpolation comes to rest at every waypoint, even without a hold.
- JSON strings, booleans, `null`, NaN, and infinity are not numeric positions
  or times. Unknown, missing, and duplicate JSON fields are rejected.
- This format contains joint angles, not Cartesian XYZ targets, degrees,
  normalized servo values, or a measured control log.

## Convert and replay

Use the offline Python/NumPy environment and downloaded SO-101 assets described
in the [README](../README.md). Isaac Sim is not needed for conversion. Run from
the repository root:

```sh
# Check input, model limits, continuous motion, and replay sample validity.
python3 scripts/convert_waypoints.py configs/example-waypoints.json --check

# Create a new directory with trajectory.npz and metadata.json.
python3 scripts/convert_waypoints.py configs/example-waypoints.json \
  --output runs/converted-waypoints

# Independently check the saved output using the existing replay entry point.
python3 scripts/control_so101.py --trajectory runs/converted-waypoints --check

# With ISAAC_SIM_PATH set to your external installation, replay in Isaac Sim.
bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --trajectory runs/converted-waypoints
```

Omit `--output` to create a unique directory under `runs/`. Existing output
directories are rejected; no existing trajectory is overwritten. `--check`
does not create output files. The converter reports invalid input and exits
with a nonzero status.

By default, the converter uses [`configs/so101-demo.json`](../configs/so101-demo.json)
for the URDF, control and physics rates, position margin, and reference velocity
and acceleration limits. Use `--config PATH` to select another compatible replay
configuration and `--urdf PATH` to override the model path. Pass the same options
to replay. As in the existing scripts, a relative `urdf` inside the configuration
is resolved from the repository root; a relative CLI path is resolved from the
working directory. The model loader requires the referenced local mesh files.

## Interpolation and validation

Between waypoints at times `t0` and `t1`, the converter evaluates

```text
T = t1 - t0
u = (t - t0) / T
q(t) = q0 + (q1 - q0) * (10*u^3 - 15*u^4 + 6*u^5)
```

Each segment begins and ends with zero velocity and acceleration. Position,
velocity, and acceleration are continuous across waypoints; jerk may jump and
is not validated. This provides a simple smooth baseline with a stop at every
waypoint, rather than a spline that flows through poses at nonzero speed.

Before sampling, the converter validates the whole continuous reference. Each
joint moves monotonically between its two endpoint angles, so checking those
endpoints against the URDF position limits plus the configured margin bounds
the entire segment. The exact peak speed is `1.875 * abs(q1-q0) / T`; the exact
peak acceleration is `(10/sqrt(3)) * abs(q1-q0) / T^2`. Speed must satisfy both
the URDF limit and the configured reference limit. Acceleration uses the
explicit project limit, not an invented manufacturer rating.

The converter then samples this curve and runs the same finite-difference
screening used by replay, including start/end rest boundaries. It rejects
infeasible motion without clipping positions or changing requested times.
Increase the affected segment's duration or reduce its motion, then revalidate.
Conversion is capped at 1,000,000 output samples to reject excessive allocations.

These are reference-motion checks. Replay holds each sampled drive target for
one control interval; the continuous curve is not a measurement of robot
motion. Collision, workspace, jerk, and measured dynamics remain unchecked.
Exports are marked `dataset_eligible: false` and are offline candidates.

## Expected replay files

```text
converted-waypoints/
  trajectory.npz
  metadata.json
```

`trajectory.npz` is a compressed NumPy archive with exactly these arrays:

| Array | Shape | Meaning |
|---|---|---|
| `timestamps_s` | `[N]` | Finite times `0, 1/control_hz, ..., (N-1)/control_hz`, with `N >= 2` |
| `positions_rad` | `[N, 6]` | Finite absolute joint angles; columns match `metadata.json`'s `joint_names` |

There are no measured positions, velocities, or accelerations in this reference
archive. For the example at 60 Hz, there are **481 samples**, from 0 through 8
seconds inclusive. At the default 120 Hz physics rate, replay holds each row
for two physics steps, including the final row, so replay covers **8 + 1/60
simulation seconds**.

`metadata.json` uses `schema_version: "so101-trajectory/1"` and
`data_kind: "reference"`. The converter fills it automatically with:

- The trajectory ID, six joint names and types in input column order, and units.
- The source URDF hash, model limits, asset repository/revision, and license.
- The entire source waypoint document, resolved replay configuration,
  interpolation parameters, and an SHA-256 hash of the source JSON file bytes.
- Generator name `waypoint-quintic-rest-to-rest`, version `1`, `seed: null`, code
  revision/dirty state, Python/NumPy versions, and `simulator: null`.
- Scene and end-effector identities, control/physics intervals, analytic
  validation in `validation.joint_reference`, sampled validation in
  `validation.replay_screen`, applied limits, and unchecked properties.

Prepared CSV and Bandai V2 references also record
`initial_pose_policy: "first_sample"`. Optional top-level `fixed_gripper_rad`
declares the constant gripper angle used during replay; omission retains the
zero-gripper default for older references. This does not enable gripper motion
or change the waypoint converter's zero-gripper input policy.

Use the existing loader to inspect output without Isaac Sim:

```python
from pathlib import Path
from isaacsim_trajectories.trajectory import load_trajectory

metadata, arrays = load_trajectory(Path("runs/converted-waypoints"))
print(metadata["joint_names"])
print(arrays["timestamps_s"].shape, arrays["positions_rad"].shape)
```

The loader validates the file schema and disables pickle loading. The separate
replay check also verifies asset identity, limits, initial/gripper policy, and
the configured schedule. Successful conversion does not establish simulator
tracking; commanded/measured logs are produced only by an actual replay.
