# Expressive Robotics Arm

Python tools for generating, validating, and replaying SO-101 follower arm motion in NVIDIA Isaac Sim, with commanded and measured data exported for expressive-motion research.

The current workflow uses a **deterministic dummy trajectory generator** and a separate **Isaac Sim replay script**. Seeded random generation and expressive-state labels are future work. This project is self-contained and does not require the broader research archive.

## Current capabilities

- Generate a smooth quintic out-and-back reference without Isaac Sim or a GPU.
- Validate model limits, trajectory shape, timestamps, units, joint names, and asset identity.
- Launch Isaac Sim, import the SO-101 follower, and replay saved angles through position-drive targets.
- Map commands by joint name and record commanded positions, measured positions, and measured velocities separately.
- Run with a window or headless, using explicit simulation-time control and physics schedules.

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

### 4. Launch Isaac Sim and replay

Set the path to your existing Isaac Sim installation:

```sh
export ISAAC_SIM_PATH="/absolute/path/to/isaac-sim"
bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --trajectory runs/my-trajectory
```

Add `--headless` to replay without a window:

```sh
bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --trajectory runs/my-trajectory --headless
```

The main script loads and validates the file, opens its own simulator process, replays the motion, saves logs under a unique `runs/so101-.../` directory, and closes the app. Run it through the launcher rather than pasting it into an existing Script Editor. Each sample is held for one control interval, including the final sample: the default replay lasts **8 + 1/60 seconds** and records 963 physics samples.

The launcher isolates inherited Python/Conda/library settings and prioritizes Isaac Sim's bundled CUDA runtime. Preserve this process-local fix: system CUDA 12.0 caused a PyTorch missing-symbol error on the inspected workstation. Replay uses MinimalRendering; see [verification and troubleshooting](docs/verification.md) for the full-editor startup limitation.

## Configuration and data

| File | Purpose |
|---|---|
| [configs/dummy-trajectory.json](configs/dummy-trajectory.json) | Dummy joint, offset, duration, rates, and reference limits |
| [configs/so101-demo.json](configs/so101-demo.json) | Model, replay rates, drive gains, tracking thresholds, and scene policy |
| [configs/isaacsim-runtime.json](configs/isaacsim-runtime.json) | Verified simulator build and runtime observations |

The configured scene has a fixed base, meters, Z-up, gravity −9.81 m/s², and no obstacles. Physics and measurements run at **120 Hz**, with held position commands at **60 Hz**. Replay requires a zero initial pose and a gripper held at zero. The five arm joints and gripper are revolute; gripper commands use radians, not normalized 0–100 values.

| Data | Files and schema | Arrays |
|---|---|---|
| Reference | `trajectory.npz` + `metadata.json`; `so101-trajectory/1` | `timestamps_s` `[N]`, `positions_rad` `[N, 6]` |
| Replay log | `motion.npz` + `metadata.json`; `so101-control-log/1` | `timestamps_s` `[M]`, `commanded_positions_rad`, `measured_positions_rad`, `measured_velocities_rad_s` `[M, 6]` |

Metadata defines column order, units, model hash/revision, resolved settings, generator/code provenance, timing, and validation status. The dummy method records `seed: null`. Reference timestamps start at zero and lie on the uniform control-rate grid. Replay logs record the initial state, then each physics endpoint against the command held during that interval. Failed or interrupted runs retain their status and available measurements.

Use `isaacsim_trajectories.trajectory.load_trajectory(Path(...))` for references and `isaacsim_trajectories.demo_log.load_log(Path(...))` for measured logs. Both validate their schemas and load NPZ with pickle disabled. Generated data, simulator caches, and downloaded assets are ignored by Git.

## Tests and verification

```sh
# Synthetic offline tests: no robot download, simulator, or GPU required.
PYTHONPATH=src python3 -m unittest discover -s tests -v

# Validate the downloaded model and generated reference without opening Isaac Sim.
python3 scripts/generate_dummy_trajectory.py --check
python3 scripts/control_so101.py --trajectory runs/my-trajectory --check
```

On 2026-10-28, **28 offline tests passed with zero skipped**. A real headless file replay passed with 481 reference samples and 963 measured samples; deliberately reversed input columns mapped correctly, and all logged commands matched the source file. The maximum joint tracking error was approximately 0.03033 rad, below the configured 0.1 rad threshold. Detailed executed checks, historical windowed results, and unchecked properties are in [docs/verification.md](docs/verification.md).

## Limits and next steps

The dummy generator checks its continuous quintic reference analytically; replay screens stored samples with finite differences, including transitions from/to rest. Asset position/velocity/effort limits and project-selected margins/gains are recorded explicitly. These checks do not establish collision freedom or measured dynamic feasibility.

Self-collision is disabled. Workspace/collision checks and measured acceleration validation are unimplemented. There is no Cartesian planner, inverse kinematics, ROS bridge, physical robot controller, or validated expressive-state mapping. The next milestone is seeded smooth waypoint generation, additional validation, and a small replay batch before producing research datasets.

## Project layout

```text
scripts/                  # Generation, replay, and simulator launcher
src/isaacsim_trajectories/ # Offline model, trajectory schema, validation, and logging
configs/                  # Portable model, generation, replay, and runtime settings
tests/                    # Simulator-independent tests
docs/                     # Verification and implementation notes
runs/                     # Generated references and logs (ignored)
assets/downloaded/        # External robot assets (ignored)
```

See [AGENTS.md](AGENTS.md) and [.github/copilot-instructions.md](.github/copilot-instructions.md) for contributor-agent guidance. The simulation implementation follows NVIDIA's [6.2 standalone lifecycle](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/python_scripting/manual_standalone_python.html) and [articulation control documentation](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/python_scripting/robots_simulation.html).
