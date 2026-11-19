# Expressive Robotics Arm

Python tools for generating, validating, and replaying SO-101 follower arm motion in NVIDIA Isaac Sim, with commanded and measured data exported for expressive-motion research.

The current workflow accepts **radian CSV trajectories**, a **deterministic dummy trajectory generator**, or a **timed waypoint converter**, followed by a separate **Isaac Sim replay script**. Seeded random generation and expressive-state labels are future work. This project is self-contained and does not require the broader research archive.

## Current capabilities

- Generate a smooth quintic out-and-back reference without Isaac Sim or a GPU.
- Convert supplied joint waypoints into smooth, validated replay files without Isaac Sim.
- Read CSV joint data and both time columns without robot assets or Isaac Sim; prepare supported SO-101 CSVs for replay using `robot_time_s`.
- Validate model limits, trajectory shape, timestamps, units, joint names, and asset identity.
- Launch Isaac Sim, import the SO-101 follower, and replay saved angles through position-drive targets.
- Map commands by joint name and record commanded positions, measured positions, and measured velocities separately.
- Run with a window or headless, using explicit simulation-time control and physics schedules.
- Select a named USD environment independently of the trajectory with `--scene`.

The default motion moves `shoulder_pan` from **0 → 0.2 → 0 radians** over eight reference seconds. The other joints, including the gripper, remain at zero target. No ROS connection is required.

## Requirements

- Git and Python **3.10+** for offline generation and tests; NumPy is installed with the package.
- For replay: a separate **Isaac Sim 6.2.0 Linux workstation installation** and a compatible NVIDIA GPU. The inspected runtime uses bundled Python 3.12.14; exact build details are in [the runtime record](configs/isaacsim-runtime.json).
- SO-101 simulation assets, downloaded separately as described below.

Run the following commands from the repository root. Use an external development environment; keep environments outside the repository and any synced research directory.

## Quick start

### 1. Clone and install the offline tools

Authenticate to GitHub with access to this private repository, then run:

```sh
git clone https://github.com/Yifan-Wang-Andy/Expressive-Robotics-Arm.git
cd Expressive-Robotics-Arm
python3 -m venv "$HOME/.venvs/expressive-robotics-arm"
source "$HOME/.venvs/expressive-robotics-arm/bin/activate"
python3 -m pip install -e .
```

This installs the offline package and NumPy. Isaac Sim remains a separate installation.

### 2. Download the verified robot assets

The selected model is `Simulation/SO101/so101_new_calib.urdf` from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), verified at revision `a758567c3978dfeefe282ede0500085a48fe8f78`. For a fresh checkout:

```sh
mkdir -p assets/downloaded
git clone --depth 1 --filter=blob:none --sparse https://github.com/TheRobotStudio/SO-ARM100.git assets/downloaded/SO-ARM100
git -C assets/downloaded/SO-ARM100 fetch --depth 1 origin a758567c3978dfeefe282ede0500085a48fe8f78
git -C assets/downloaded/SO-ARM100 checkout --detach FETCH_HEAD
git -C assets/downloaded/SO-ARM100 sparse-checkout set Simulation
```

Keep the neighboring mesh folders with the URDF. These third-party assets are licensed **Apache-2.0** by their upstream project and are excluded from this repository. That attribution applies to the downloaded assets; no license is assigned here to this project's original code.

### 3. Generate a trajectory

```sh
python3 scripts/generate_dummy_trajectory.py --output runs/my-trajectory
```

This creates `trajectory.npz` and `metadata.json`. Existing output directories are rejected; omitting `--output` creates a unique directory and prints its path. To change the dummy motion:

```sh
python3 scripts/generate_dummy_trajectory.py \
  --joint wrist_roll --offset-rad 0.3 --move-seconds 5 \
  --output runs/wrist-trajectory
```

`--move-seconds` is the duration of **each direction**. Replace `generate_dummy()` in [the generator script](scripts/generate_dummy_trajectory.py) to introduce a new method while preserving the file contract and recording its algorithm/configuration.

To supply your own timed joint poses, start with [configs/example-waypoints.json](configs/example-waypoints.json):

```sh
python3 scripts/convert_waypoints.py configs/example-waypoints.json \
  --output runs/converted-waypoints
```

The converter creates the same replay files, interpolating a quintic segment that comes to rest at each waypoint. It checks continuous position, velocity, and acceleration bounds plus the replay sample checks. Use the resulting directory as `--trajectory` in step 4. See [the input format and conversion guide](docs/trajectory-input.md) for JSON fields, joint ordering, timing rules, output details, and check-only commands.

For existing CSV data, place files in **`data/` beside `src/`** and inspect them directly:

```sh
python3 scripts/read_trajectory.py data/dataset-1_kick_normal_001_right_so101_option-9.csv
python3 scripts/read_trajectory.py data/dataset-1_kick_normal_001_right_gen3_option-9.csv

# Validate SO-101 replay preparation without opening Isaac Sim.
python3 scripts/control_so101.py \
  --trajectory data/dataset-1_kick_normal_001_right_so101_option-9.csv --check
```

CSV joint positions are already **radians**. Playback uses **`robot_time_s`**;
`source_time_s` and tool XYZ remain source context. The reader recognizes the
five named SO-101 arm columns and the seven numbered Gen3 columns, but this
project replays only the verified SO-101 model. Gen3 columns are not mapped to
SO-101 joints. Raw CSV inspection does not establish replay eligibility.

The supplied SO-101 file inspected on **2026-11-04** has **358 rows** and a
robot-time duration of **110.914143210 seconds**. Its positions fit the current
model envelope. On **2026-11-05**, the project reference limits were raised at
the user's request to **1.5 rad/s** and **40 rad/s²**. The CSV passes offline
replay checks with sampled peaks of **1.223521 rad/s** and **32.702266 rad/s²**.
These are project-selected simulation thresholds, not manufacturer ratings;
URDF limits and tracking checks remain enforced. The CSV and its timing were
not modified, and simulator tracking of this motion remains unverified.
See [CSV input](docs/trajectory-input.md#read-csv-trajectories) for timing,
initialization, and the distinction between reading and replay validation.

### 4. Launch Isaac Sim and replay

Set the path to your existing Isaac Sim installation:

```sh
export ISAAC_SIM_PATH="/absolute/path/to/isaac-sim"
bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --trajectory runs/my-trajectory
```

For a supported CSV that passes `--check`, pass its file path instead of the
reference directory. The supplied SO-101 example above passes with the current
project reference limits.

Add `--headless` to replay without a window:

```sh
bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --trajectory runs/my-trajectory --headless
```

The main script loads and validates the file, opens its own simulator process, replays the motion, and saves logs under a unique `runs/so101-.../` directory. Windowed runs show the initial pose for **3 seconds**, pace playback for viewing, and **keep the final pose open** after saving. Close the window or press Ctrl+C in the terminal when finished. Headless runs exit automatically. Run it through the launcher rather than pasting it into an existing Script Editor.

To replay that same trajectory in a different scene:

```sh
python3 scripts/control_so101.py --list-scenes
bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --scene table --trajectory runs/my-trajectory
```

`empty` (the default) and `table` are defined in [configs/scenes.json](configs/scenes.json).
Add your own static environment USD files there and select them by name; the script
adds the SO-101 at the origin. Scene sources remain unchanged, and the selected
scene is recorded in each run's logs. See [scene selection](docs/scene-selection.md)
for stage requirements, custom catalogs, and a loop over scene/trajectory pairs.

For more time to watch the small default shoulder rotation (about **11.5°** out and back):

```sh
bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --trajectory runs/my-trajectory --start-delay 5 --playback-speed 0.5
```

`--playback-speed 0.5` presents the motion at half speed; slow rendering may make it take longer. `--exit-on-complete` restores automatic window closure for scripted runs. Start delay and speed apply only to windowed viewing. These options do not change the trajectory, physics timestep, or logged simulation times. Each sample is still held for one control interval, including the final sample: the default replay lasts **8 + 1/60 simulation seconds** and records 963 physics samples. Preview and final viewing render without advancing physics or adding measurements.

The terminal prints motion size, replay progress, `SO101_DEMO_COMPLETED`, and the saved log directory. A failure instead prints `SO101_DEMO_FAILED` and records the reason in that run's `metadata.json`. If an older version closed immediately, check this status first: closing was also the old behavior after a successful replay. Existing trajectories can be replayed again without regenerating them.

The viewport uses a light gray background. Change `background_color_rgb` in [configs/so101-demo.json](configs/so101-demo.json) before launching to choose another color: `[0.65, 0.65, 0.65]` is the default gray, `[1, 1, 1]` is white, and `[0, 0, 0]` is black. Each red/green/blue value must be between 0 and 1. This changes the displayed backdrop independently of robot illumination and is recorded in the replay metadata.

Relaunch the script after changing the color. Windowed replay uses **RTX - Real-Time** (`RaytracedLighting`, the legacy renderer): the inspected build's Minimal mode kept the backdrop black even with the correct color settings. For an existing window, select **RTX - Real-Time** in the viewport render-mode menu if available, then use **Window → Render Settings → Common → Background** to adjust the color. Edit the config for the color to apply on subsequent script launches.

The launcher isolates inherited Python/Conda/library settings and prioritizes Isaac Sim's bundled CUDA runtime. Preserve this process-local fix: system CUDA 12.0 caused a PyTorch missing-symbol error on the inspected workstation. Both replay modes use RaytracedLighting. Every replay scene has a sun-like distant light at `/World/light` (display name `Sun`), directed down toward the robot, plus ambient illumination. The sun explicitly uses intensity 3000, exposure 0, white color with 6500 K color temperature enabled, and a 0.53° angular diameter. These display settings are recorded in each run's metadata; the gray backdrop is configured separately. See [verification and troubleshooting](docs/verification.md) for the full-editor startup limitation and captured background checks.

## Configuration and data

| File | Purpose |
|---|---|
| [configs/dummy-trajectory.json](configs/dummy-trajectory.json) | Dummy joint, offset, duration, rates, and reference limits |
| [configs/example-waypoints.json](configs/example-waypoints.json) | Example timed joint poses for the waypoint converter |
| [configs/so101-demo.json](configs/so101-demo.json) | Model, replay rates, drive gains, tracking thresholds, and scene policy |
| [configs/isaacsim-runtime.json](configs/isaacsim-runtime.json) | Verified simulator build and runtime observations |

The default scene has a fixed base, meters, Z-up, gravity −9.81 m/s², and no obstacles. Physics and measurements run at **120 Hz**, with held position commands at **60 Hz**. Existing generated references require a zero initial pose. CSV imports record `initial_pose_policy: "first_sample"` and initialize the arm directly at that pose before control; no approach from zero is added. The gripper remains at zero in both cases. The five arm joints and gripper are revolute; gripper commands use radians, not normalized 0–100 values.

| Data | Files and schema | Arrays |
|---|---|---|
| Source CSV | `data/*.csv`; named header columns | Both time columns, source joint radians, and separate tool XYZ in meters |
| Reference | `trajectory.npz` + `metadata.json`; `so101-trajectory/1` | `timestamps_s` `[N]`, `positions_rad` `[N, 6]` |
| Replay log | `motion.npz` + `metadata.json`; `so101-control-log/1` | `timestamps_s` `[M]`, `commanded_positions_rad`, `measured_positions_rad`, `measured_velocities_rad_s` `[M, 6]` |

Metadata defines column order, units, model hash/revision, resolved settings, generator/code provenance, timing, and validation status. The dummy method records `seed: null`. Reference timestamps start at zero and lie on the uniform control-rate grid. Replay logs record the initial state, then each physics endpoint against the command held during that interval. Failed or interrupted runs retain their status and available measurements.

Use `isaacsim_trajectories.trajectory.load_trajectory(Path(...))` for saved reference directories and `isaacsim_trajectories.demo_log.load_log(Path(...))` for measured logs. Both validate their schemas and load NPZ with pickle disabled. Use `scripts/read_trajectory.py INPUT.csv` to inspect source CSV data. A CSV replay archives the normalized reference and unchanged `source.csv` under its run's `reference/` directory, retaining both source clocks and tool coordinates. The tool coordinate frame is unknown and those values are not simulator measurements. User CSVs in `data/`, generated data, simulator caches, and downloaded assets are ignored by Git.

## Tests and verification

On 2026-11-05, **85 offline tests passed with zero skipped**, including CSV
parsing, robot-time resampling, source preservation, and direct CSV replay
preflight. Both supplied CSV files were read successfully. The SO-101 CSV was
initially rejected by the original limits and subsequently passed after the
user-requested limit increase. No simulator replay was executed for the CSV
addition or limit change. See [the verification record](docs/verification.md#csv-reader-checks).

```sh
# Synthetic offline tests: no robot download, simulator, or GPU required.
PYTHONPATH=src python3 -m unittest discover -s tests -v

# Validate the downloaded model and generated reference without opening Isaac Sim.
python3 scripts/generate_dummy_trajectory.py --check
python3 scripts/control_so101.py --trajectory runs/my-trajectory --check
```

On 2026-10-28, **65 offline tests passed with zero skipped**, including scene selection and waypoint conversion. **11 separate Isaac Sim stage-loading tests passed**. Headless replays in both `empty` and `table`, plus a rendered table replay, completed with 963 samples each; commands matched the source trajectory and timing matched 120 Hz. Initial/midpoint captures showed the arm and tabletop. Maximum tracking error was approximately 0.03033 rad, below the configured 0.1 rad threshold. The converter's multi-joint example has only passed offline replay checks; these scene replays used `runs/my-trajectory`. Detailed executed commands, historical results, and unchecked properties are in [docs/verification.md](docs/verification.md).

## Limits and next steps

The dummy generator and waypoint converter check their continuous quintic references analytically; replay screens stored samples with finite differences, including transitions from/to rest. CSV input is linearly resampled on the control grid using robot time, with the last pose held through the next grid boundary when needed. Linear interpolation can have velocity discontinuities at source rows; replay screening is not a continuous acceleration proof. The converter uses supplied waypoints and does not yet sample random poses. Asset position/velocity/effort limits and project-selected margins/gains are recorded explicitly. These checks do not establish collision freedom or measured dynamic feasibility.

Self-collision is disabled. Workspace/collision checks and measured acceleration validation are unimplemented. There is no Cartesian planner, inverse kinematics, ROS bridge, physical robot controller, or validated expressive-state mapping. The next milestone is seeded smooth waypoint generation, additional validation, and a small replay batch before producing research datasets.

## Project layout

```text
scripts/                  # CSV inspection, generation, replay, and simulator launcher
src/isaacsim_trajectories/ # Offline model, trajectory schema, validation, and logging
data/                     # User-supplied CSV trajectories (CSVs ignored)
configs/                  # Portable model, generation, replay, and runtime settings
configs/scenes/           # Small project-authored USD environment examples
tests/                    # Simulator-independent tests
docs/                     # Verification and implementation notes
runs/                     # Generated references and logs (ignored)
assets/downloaded/        # External robot assets (ignored)
```

See [AGENTS.md](AGENTS.md) and [.github/copilot-instructions.md](.github/copilot-instructions.md) for contributor-agent guidance. The simulation implementation follows NVIDIA's [6.2 standalone lifecycle](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/python_scripting/manual_standalone_python.html) and [articulation control documentation](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/python_scripting/robots_simulation.html).
