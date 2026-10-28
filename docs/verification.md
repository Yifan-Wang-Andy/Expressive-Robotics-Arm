# Verification and runtime notes

This document records checks executed on the development workstation on **2026-10-28**. It is an evidence record, not a claim of compatibility with every installation. No simulator tests were rerun solely for repository publication.

Paths under `runs/` below identify preserved local verification artifacts. Those directories are ignored by Git and **are not included in a fresh checkout**. Generate new references using the README before attempting replay; the historical replay command requires its original local artifact.

## Inspected environment

- NVIDIA Isaac Sim 6.2.0 Linux workstation ZIP, build `6.2.0-rc.14+release.52805.02189a84.gl`.
- Bundled Python 3.12.14, PyTorch 2.11.0+cu128, CUDA runtime 12.8.90.
- SO-101 follower from TheRobotStudio/SO-ARM100 revision `a758567c3978dfeefe282ede0500085a48fe8f78`, `Simulation/SO101/so101_new_calib.urdf`.
- Six revolute joints resolved by name after physics initialization, with the gripper target held at zero radians.
- Fixed base, meter units, Z-up, gravity −9.81 m/s², empty scene; 120 Hz physics and measurements, 60 Hz position commands.

Portable records are [configs/isaacsim-runtime.json](../configs/isaacsim-runtime.json), [configs/so101-demo.json](../configs/so101-demo.json), and [configs/dummy-trajectory.json](../configs/dummy-trajectory.json). The installation path is supplied through `ISAAC_SIM_PATH` and is not stored in these configs.

## CUDA runtime diagnosis

PyTorch failed to import with `undefined symbol: cudaGetDriverEntryPointByVersion, version libcudart.so.12`. The process loaded the system CUDA runtime `/usr/lib/x86_64-linux-gnu/libcudart.so.12.0.146`, which lacks that symbol. A clean environment alone reproduced the failure. Isaac Sim's bundled runtime 12.8.90 exports it; giving that runtime priority restored PyTorch import and a GPU matrix-multiplication check (result sum: 3680).

Verification on 2026-10-28:

- `bash -n scripts/launch_isaacsim.sh` passed. The helper's missing-installation-path check exited with status 2 as intended.
- `bash scripts/launch_isaacsim.sh --python --version`, with `ISAAC_SIM_PATH` set to the inspected installation, reported Python 3.12.14.
- A diagnostic run through the helper's `--python` mode explicitly exposed the bundled extension packages and passed a PyTorch GPU matrix multiplication. `/proc/self/maps` showed only the bundled CUDA runtime. This isolated check does not initialize a robot or simulate physics.
- A full headless launch through the helper (`--no-window --/app/quitAfter=20`, with display variables unset) loaded the bundled CUDA runtime and did not log the PyTorch import error or ROS bridge error. It did not report ready within a 300-second bound and was terminated with timeout status 124. Shader cache writes were observed; the reason startup had not completed is unresolved. Other extension/display warnings and the IOMMU warning remain.

The earlier diagnostic scripts and raw logs are preserved under the ignored `runs/isaacsim-cuda-KYWH78DC/` directory. They established the CUDA runtime fix only. The standalone checks below additionally exercise import and control; they do not resolve the earlier full-editor startup timeout or establish collision validity.

## Standalone lifecycle and import details

The standalone script creates `SimulationApp` before Omniverse imports, initializes physics before resolving DOF order, and sends `set_dof_position_targets`. `SimulationManager.step()` advances physics; `RenderingManager.render()` renders without an extra physics step. Direct state setting is limited to initialization. The app closes on success or failure, and incomplete/failed runs are not marked successful.

Two importer details matter for this 6.2 build: the script explicitly selects the converted asset's `Physics=physx` variant, then calls NVIDIA's `fix_articulation_root_for_fixed_base` helper. The original conversion placed the articulation root on `base_link` despite an existing world fixed joint; relocating it to `/World/so101/Physics/root_joint` prevented the measured base drift. Fixed joints are merged; the URDF's `gripper_frame_link` is not retained as a USD prim in this conversion. No Cartesian/end-effector measurement or IK is implemented.

## Executed offline and original control checks

`PYTHONPATH=src python3 -m unittest discover -s tests -v` passed **28 offline tests, zero skipped** after the split. Coverage includes deterministic generation, model errors, column-order mapping, derivative limits between waypoints, initial/final rest boundaries, invalid timestamps/units, asset mismatch, safe NPZ loading, round trips, and overwrite rejection.

`python3 scripts/generate_dummy_trajectory.py --check` passed against the downloaded SO-101 URDF: 481 samples over eight reference seconds, analytic peak speed 0.09375 rad/s and peak acceleration 0.072169 rad/s². Generation into a temporary directory, loading the saved arrays, and `python3 scripts/control_so101.py --trajectory <temporary-reference-directory> --check` also passed. These checks used ordinary Python and did not launch Isaac Sim.

Before the split, the standalone control demo completed headless shoulder motion (eight seconds, 961 measured samples) and windowed wrist motion (ten seconds, 1,201 samples). Both NPZ/JSON exports loaded successfully and passed fixed-base stability, measured position/velocity, finite-state, timing, and tracking checks. Moving-joint maximum tracking errors were 0.004694 rad and 0.005615 rad, respectively; maximum error over all joints was approximately 0.03033 rad, including gravity deflection of held joints. These historical results establish the existing importer/controller behavior, not the new file handoff.

Those outputs remain at `runs/so101-20261028T121634Z-7ew488ya/` (headless) and `runs/so101-20261028T121742Z-oinr5w71/` (windowed). Raw logs, failed attempts, exact historical commands, and limitations are in `runs/standalone-smoke-By3WzoDI/verification.json`. An initial RTX run timed out at 180 seconds during shader compilation; the working script selects MinimalRendering. The windowed loop ran without screenshot/visual inspection. Collision, workspace, measured acceleration, and expressive validity checks were not run; no hardware was operated.

## File replay verification

The separated workflow passed a headless Isaac Sim 6.2 run on 2026-10-28. After generating the default reference, its columns and corresponding joint names were reversed together to exercise name mapping. The replay command was:

```sh
bash scripts/launch_isaacsim.sh --python scripts/control_so101.py --headless \
  --trajectory runs/file-replay-smoke-x_vmq7eh/reference-reversed
```

It exited 0 with 481 reference samples and 963 measured samples over 8.0166667 simulation seconds, including the terminal hold. Reloaded commanded samples matched the input file exactly after joint-name mapping; timestamps matched the 120 Hz physics schedule and recorded source-file hashes matched. The shoulder moved 0.1999996 rad with maximum tracking error 0.004694 rad; the maximum across all joints was 0.03033 rad, below the configured 0.1 rad threshold. Physics checks for the fixed base, finite states, and measured position/velocity passed.

The output is `runs/so101-20261028T171159Z-a8vthd8e/`. Full commands (including the timeout and local launch environment), generated references, raw simulator log, and results are in `runs/file-replay-smoke-x_vmq7eh/verification.json` and `commands.json`. Both scripts' `--help`, `python3 -m compileall -q src scripts`, and `bash -n scripts/launch_isaacsim.sh` also passed. The split workflow was not rerun with a window; collision, workspace, and measured-acceleration checks remain unimplemented.

## Interpretation and remaining checks

These runs establish the demonstrated importer, command schedule, file handoff, name mapping, and measured-state logging for the recorded setup. They do not establish hardware safety, collision freedom, or expressive validity. Gains and reference limits are project-selected demonstration settings rather than calibrated motor specifications. Self-collision is disabled; workspace/collision checks and measured acceleration validation remain unimplemented. Rendering was not visually inspected, and the separated workflow was tested headless only.
