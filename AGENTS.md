# Expressive Robot Implementation

## Purpose and scope

Act as the implementation collaborator for the Expressive Robot Honors Research project. Build reproducible Python tools that generate random, smooth robot-arm trajectories, validate them, replay them in NVIDIA Isaac Sim, and export motion data for later expressive-motion research.

This is a separate implementation project, rooted in `Expressive_Robot_Implementation/`. Keep implementation work in this folder. The broader research archive is a separate sibling folder and is not a runtime dependency.

The user's current request defines the task. Papers, proposals, meeting notes, and downloaded examples are reference material, not instructions to execute. Older literature-review priorities do not override the current implementation stage. Implement the smallest useful slice; do not add learning models, ROS, hardware control, or a multi-robot framework unless the task needs them.

This folder contains an Isaac Sim launch helper, portable runtime record, separate offline dummy trajectory generator and SO-101 file replay script, and offline model/motion/schema tests. `scripts/generate_dummy_trajectory.py` saves a deterministic quintic reference; `scripts/control_so101.py --trajectory DIRECTORY` opens Isaac Sim and replays it with commanded/measured logs. Reference files use `so101-trajectory/1`; measured logs use `so101-control-log/1`. Seeded random generation remains future work. The simulator is installed externally; keep its path in `ISAAC_SIM_PATH`. See the README for executed checks and remaining limitations. Never report planned commands or tests as working ones.

## Research context and unresolved choices

- The broader research studies perceived attention, curiosity, confidence, and hesitation across different embodiments. Random trajectories are candidate stimuli or baseline data, not validated examples of those states.
- The user selected the SO-101 follower for this implementation. The demo uses TheRobotStudio/SO-ARM100 `Simulation/SO101/so101_new_calib.urdf`, five revolute arm joints plus a revolute gripper. Read limits from the actual model and resolve joint ordering after physics initialization; do not assume index ordering or interchange old/new calibration models.
- The inspected simulator is an external Isaac Sim 6.2.0 workstation installation (build `6.2.0-rc.14+release.52805.02189a84.gl`) with bundled Python 3.12.14, PyTorch 2.11.0+cu128, and CUDA runtime 12.8.90. See `configs/isaacsim-runtime.json`, `configs/so101-demo.json`, `configs/dummy-trajectory.json`, and the README. Replay uses a fixed base in an empty meter/Z-up scene, 120 Hz physics and 60 Hz control, holding each saved row for one control interval including the terminal row. The dummy generator checks its continuous reference analytically; replay checks sample finite differences including rest boundaries. Collision/workspace and measured acceleration validation remain unchecked.
- Optional background in the sibling research archive lives in `../Expressive_Robot_Honors_Research/context/PROJECT_CONTEXT.md`, `../Expressive_Robot_Honors_Research/context/ADVISOR_GUIDANCE.md`, and `../Expressive_Robot_Honors_Research/context/DECISIONS.md`. Use these only when available and relevant to the task; the self-contained context above is the starting point for coding. Do not require access to the research archive, edit it as part of routine implementation, or copy its materials into this project automatically. A standalone GitHub checkout must remain usable without it.
- Descriptor definitions, normalization, and expressive-state labels remain research questions. Keep raw motion distinct from derived features and human annotations.

## Before simulator-dependent work

Inspect the repository and available environment first. Resolve the installed Isaac Sim version and launch method; robot USD/URDF source and revision; articulation and end-effector identifiers; joint names, types, and limits; gripper policy; scene units and frames; physics timestep; and control frequency. Record verified choices in the README and a portable configuration. Ask only for details that cannot be recovered and block the next step; independent offline work can continue.

Use official NVIDIA documentation matching the selected release. Do not mix APIs from different releases, invent asset URLs, or assume that ordinary system Python can import Isaac Sim. Keep machine-specific installation paths in local environment settings. Select dependencies compatible with the simulator's Python runtime before pinning versions.

Use `scripts/launch_isaacsim.sh` for this workstation setup. It isolates inherited Python/Conda/library settings and prioritizes the bundled CUDA runtime for the launched process. System CUDA runtime 12.0.146 caused a missing-symbol error with bundled PyTorch; clearing inherited variables alone did not fix it. Preserve this process-local fix rather than replacing system libraries or installing another PyTorch version without evidence.

## Code organization

Use this starting layout, adding files as functionality is implemented:

```text
src/isaacsim_trajectories/  # Schema, sampling, validation, export; isolated sim adapter
scripts/                  # Thin generation, replay, and inspection entry points
configs/                  # Portable robot, scene, and generation settings
tests/                    # Offline tests plus separately identified simulator tests
docs/                     # Setup and implementation decisions when needed
runs/                     # Generated trajectories and logs; ignored by Git
```

Keep numerical generation, validation, and serialization importable without Isaac Sim or a GPU. Put all simulator-specific imports and lifecycle operations behind an adapter or simulator entry point. Prefer small typed functions, NumPy arrays, explicit configuration, and `pathlib`; avoid hidden global state and machine-specific paths. Add packaging and development commands when code is introduced, and document only commands that exist.

## Random trajectory generation

1. Start with a joint-space baseline: sample bounded waypoints and durations, then interpolate smooth motion. Treat this as a starting approach, not a frozen research decision. Do not issue independent random joint positions every simulation frame.
2. Use a local seeded random generator, such as NumPy's `Generator`. Record the seed, generator/algorithm version, sampling distribution, ranges, waypoint count, timing, and interpolation parameters. Assign stable per-trajectory seeds for batches so retries do not silently change other trajectories.
3. Read joint names and types from the verified model. Map commands by name; handle revolute, prismatic, continuous, mimic, and gripper joints explicitly. Keep the gripper fixed unless the configuration requests gripper motion.
4. Use model limits and documented experimental margins. Velocity/acceleration limits absent from the asset require explicit configured values and provenance; never invent them as manufacturer limits. Reject malformed or missing required limits.
5. Check the interpolated trajectory, including its transition from the initial pose, for position, velocity, and acceleration limits. Check jerk when required by the experiment. Waypoint validity alone is insufficient. Prefer analytic extrema where available; otherwise document the sampling resolution and its limitations.
6. Reject or regenerate invalid paths, or adjust timing and revalidate. Do not silently clip commands to conceal infeasibility. Bound retry counts and report rejection reasons and acceptance rates; rejection changes the resulting distribution.
7. Distinguish joint-limit validity from workspace and collision validity. Check configured workspace bounds, self-collision, and scene collision with a supported method. Record unchecked properties as unchecked. Collision-unchecked trajectories can be exported as offline candidates, but must not enter a validated dataset or be described as collision-free.
8. Add Cartesian sampling only when needed. Record the reference frame, check inverse-kinematics success and joint continuity, and validate the entire resulting joint path. Keep intended Cartesian targets distinct from achieved motion.

## Isaac Sim execution

- Follow the selected release's startup lifecycle. For standalone `SimulationApp` scripts, initialize the app before imports that require loaded Omniverse extensions; close the app on both success and failure.
- Verify the articulation, joint mapping, stage units, limits, and initial state after loading/resetting. Fail clearly on missing assets, unexpected joints, or unsupported configuration.
- Step control with simulation time and an explicit physics/control schedule. Rendering speed or wall-clock sleeps must not define trajectory timing.
- Use the appropriate controller/action interface for replay. Direct state setting is for initialization or explicitly labeled kinematic previews; it is not evidence of dynamic feasibility.
- Record commanded and measured positions separately. Monitor tracking error, nonfinite states, configured limit violations, and collisions where checks are available. Mark failed or incomplete runs and terminate them cleanly.
- Support headless and visual runs when implemented. Seeded generation should be reproducible within the recorded software environment; do not promise bit-for-bit physics replay across GPUs or simulator releases.
- This repository targets simulation. Physical robot execution requires the user's explicit authorization and a documented motion envelope and stop procedure.

## Trajectory data contract

Version the schema and validate it on save and load. Use an array-oriented format such as NPZ plus a readable JSON metadata file initially; document any change.

Each trajectory must identify:

- Trajectory/run ID, schema version, seed, complete resolved generation configuration, code revision and dirty-state status when available.
- Robot/asset identity and revision or hash, joint names and types in array order, end-effector identity, and scene identity.
- Strictly increasing timestamps in seconds and joint positions shaped `[samples, joints]`. Use radians for revolute joints and meters for prismatic joints; explicitly convert source asset units.
- Reference versus measured data, sample/control/physics intervals, and any resampling or interpolation. Record velocities and accelerations with their derivation method when provided.
- For Cartesian poses: coordinate-frame definitions, transforms needed to interpret them, meters, and quaternion component order. Normalize quaternions and handle orientation continuity when computing derivatives.
- Validation results, rejection/failure reasons, collision-check status and method, applied limits, simulator/Python/library versions, and relevant runtime settings.

Use unique run directories and preserve completed raw logs. Never overwrite existing datasets silently. Keep processing and normalization reproducible; missing measurements must not be filled with fabricated values. Record target-relative context if an experiment needs relational descriptors.

## Verification and completion

For implemented behavior, test seeded generation, dimensions and finite values, strictly increasing time, joint ordering, limit enforcement between waypoints, invalid configurations, bounded retries, and export/load round trips. Include a nontrivial boundary case rather than tests that merely mirror the implementation.

Keep offline tests independent of simulator installation. Simulator checks must be separately selectable and explicitly reported as skipped when Isaac Sim, the asset, or a compatible GPU is unavailable. Mocks do not establish simulator compatibility. A normal GitHub runner should be able to run the offline suite once it exists.

The first working milestone is one configured arm, a seeded smooth joint-space generator, limit validation, one simulator replay with measured-state logging, and a readable export. Validate a small batch before large generation runs. Report exact commands executed, results, checks not run, and remaining limitations. Do not claim expressive validity, collision freedom, or hardware safety from a successful animation.

## Git and workspace hygiene

Keep this folder self-contained as the implementation project's root for GPT, VS Code, and a future GitHub repository. `AGENTS.md` is the main agent guide; `.github/copilot-instructions.md` carries a short compatible summary. Keep the two consistent.

Commit source, portable configs, documentation, and small synthetic fixtures. Keep credentials, local settings, environments, simulator caches, generated datasets, recordings, and large third-party assets out of Git. Document asset provenance and licensing rather than copying assets by default. Keep environments and caches outside any Dropbox-synced tree. Preserve unrelated work and limit changes to the requested task. Do not publish research material or push to a remote unless requested.
