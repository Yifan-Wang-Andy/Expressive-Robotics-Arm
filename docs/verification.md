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

The standalone script creates `SimulationApp` before Omniverse imports, initializes physics before resolving DOF order, and sends `set_dof_position_targets`. `SimulationManager.step()` advances physics; `RenderingManager.render()` renders without an extra physics step. Direct state setting is limited to initialization. Windowed runs now save completed logs before keeping the final pose open for inspection; headless runs and failures close automatically. Incomplete/failed runs are not marked successful.

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

## Windowed viewing and visibility fix

The user's `runs/my-trajectory` replay at `runs/so101-20261028T172710Z-c0nfz04k/` had already completed successfully: 963 samples, 0.1999996 rad measured shoulder-pan excursion, and no failure reason. The previous script closed immediately after completion, with no preview delay or presentation pacing.

The updated script adds a three-second windowed preview, per-control-interval presentation pacing (`--playback-speed`, default 1), progress output, and a final viewer that stays open after export. `--exit-on-complete` restores automatic window closure. Viewing never changes the saved targets or physics/control schedule. The final viewer handles Ctrl+C without reclassifying completed logs as failed. Headless execution remains unpaced and exits automatically.

A first genuine windowed check completed, but initial/midpoint viewport captures were black. Explicit MinimalRendering textured-diffuse mode (`minimal_shading_mode=2`), reapplying render settings after stage creation, and distant plus ambient lighting produced visible captures. This follows NVIDIA's [6.2 rendering-mode documentation](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/reference_material/rendering_modes.html): simplified Minimal rendering uses a distant light rather than the previous dome-only setup. The first lighting trial failed because `SimulationApp.set_setting` does not accept a color array; using `carb.settings.set_float_array` corrected that API error. These diagnostic attempts remain preserved locally.

Executed checks on 2026-10-28:

```sh
PYTHONPATH=src python3 -m unittest discover -s tests -v
python3 scripts/control_so101.py --trajectory runs/my-trajectory --check
python3 scripts/control_so101.py --help
python3 -m compileall -q src scripts tests
ISAAC_SIM_PATH=/home/yifan/isaacsim-6.2.0 timeout 180 bash scripts/launch_isaacsim.sh \
  --python runs/viewing-smoke-1x_mzwh7/verify_lit_fixed.py
ISAAC_SIM_PATH=/home/yifan/isaacsim-6.2.0 timeout 180 bash scripts/launch_isaacsim.sh \
  --python scripts/control_so101.py --trajectory runs/my-trajectory --headless
```

All **33 offline tests passed, zero skipped**. The simulator commands both exited 0. The local windowed harness called the real replay at 0.5× presentation speed, captured the initial and midpoint viewport images, checked that every render-only wait left simulation time unchanged, verified files existed before final viewing, then injected KeyboardInterrupt after two seconds of final viewing to exercise the Ctrl+C path. The arm and its changed pose were visually confirmed in both captures. Motion took approximately 18.6 wall seconds for 8.0166667 simulation seconds at requested 0.5× speed; rendering overhead can make presentation slower than requested.

Both runs saved 963 samples with maximum tracking error 0.0303253 rad. Reloaded windowed commands exactly matched every reference row held for two physics steps plus the initial measurement; timestamps matched the 120 Hz schedule. All four raw arrays in the headless run were identical to the windowed run. Preview and final viewing added no measurements. The native window close button and `--exit-on-complete` branch were not separately exercised in Isaac Sim; offline checks cover viewer closure callbacks.

Windowed artifacts are under `runs/viewing-smoke-1x_mzwh7/lit-fixed/`; the headless log is `runs/so101-20261028T173552Z-ab77rsge/`. The ignored `runs/viewing-smoke-1x_mzwh7/verification.json` records commands/results, and the harness and console logs are preserved beside it. These are local evidence, not dependencies or fixtures shipped with the repository.

## Configurable background

The replay now defaults to a gray color override using `background_color_rgb: [0.65, 0.65, 0.65]` in `configs/so101-demo.json`. RGB entries are checked for finite values in [0, 1] before simulator startup. The resolved background settings are recorded in the run metadata. The installed 6.2 Common/Background UI and NVIDIA's [RTX background documentation](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/rtx-renderer_common.html#background) confirm the color override settings.

On 2026-10-28, `python3 scripts/control_so101.py --trajectory runs/my-trajectory --check`, `python3 -m compileall -q src scripts`, and `PYTHONPATH=src python3 -m unittest discover -s tests -v` passed (33 tests, zero skipped). No new tests were added for this display change.

The initial gray-background change was **unconfirmed and ineffective**, as the user's subsequent black viewport demonstrated. With `ISAAC_SIM_PATH=/home/yifan/isaacsim-6.2.0`, `timeout 120 bash scripts/launch_isaacsim.sh --python runs/background-smoke-t43r20km/verify_background.py` timed out during first-frame shader compilation when using post-tone-map compositing. `timeout 180 bash scripts/launch_isaacsim.sh --python runs/background-smoke-t43r20km/verify_simple.py` also timed out during startup viewport shader compilation. Both exited 124 without captures. Logs/harnesses remain in that directory; these were not successful simulator checks.

The follow-up on 2026-10-28 resolved this with **RaytracedLighting for windowed replay**, retaining MinimalRendering for headless execution. Background color is authored directly on the active viewport's USD render product after physics initialization. In real offscreen captures, Minimal mode 2 still produced opaque black pixels even with the correct USD color readback, post-tonemap compositing, and a fully enabled compositing pass. Changing the renderer to RaytracedLighting produced a gray backdrop. This is observed behavior of the inspected build, not a claim that all Minimal renderer versions lack background support.

Final verification commands:

```sh
ISAAC_SIM_PATH=/home/yifan/isaacsim-6.2.0 timeout -k 10 180 bash scripts/launch_isaacsim.sh \
  --python runs/usd-background-smoke-lwpw9s98/verify_final.py
ISAAC_SIM_PATH=/home/yifan/isaacsim-6.2.0 timeout -k 10 120 bash scripts/launch_isaacsim.sh \
  --python scripts/control_so101.py --trajectory runs/my-trajectory --headless
PYTHONPATH=src python3 -m unittest discover -s tests -v
python3 -m compileall -q src scripts
```

Both simulator runs exited 0; all 33 offline tests passed. The final harness used the actual windowed replay/rendering path with its OS window hidden (`headless=True`, `disable_viewport_updates=False` in the test wrapper only). It did not override renderer selection. Initial and midpoint PNGs were visually inspected: the arm was visible against a gray backdrop, with background RGB `[217, 217, 217]` at pixel (10, 10), compared with `[0, 0, 0]` in all three Minimal trials. Both final runs saved 963 samples, and all four raw arrays were identical between the rendered and headless replays. Commands matched the source trajectory and timestamps matched the 120 Hz schedule. Maximum tracking error was 0.0303253 rad.

Artifacts, diagnostic comparisons, and exact commands are in `runs/usd-background-smoke-lwpw9s98/verification.json`; final captures/logs are under its `final/` subdirectory. The headless output is `runs/so101-20261028T180741Z-7qgpowid/`. An initial visible diagnostic window closed during startup and was recorded failed; subsequent tests rendered offscreen to avoid additional windows. Native window interactions after the renderer change were not retested. The user's existing simulator process was left running, so it needs a restart to use the updated script.

### Explicit sun light (2026-11-19)

The existing distant light at `/World/light` is now explicitly named `Sun`, with intensity 3000, exposure 0, white color, enabled 6500 K color temperature, and a 0.53-degree angular diameter. Its rotation remains `[-35, -25, 0]` with ambient fill at 0.3. A distant light represents illumination from a faraway source; the angular size follows the [OpenUSD sun example](https://openusd.org/release/user_guides/schemas/usdLux/DistantLight.html). These are visual settings, not a calibrated solar-light measurement. The solid background remains independently configured. Both replay modes now select `RaytracedLighting` at the user's request.

Executed with `ISAAC_SIM_PATH` set to the inspected 6.2 installation:

```sh
timeout -k 10 180 bash scripts/launch_isaacsim.sh --python runs/sun-light-smoke-0pbj3i36/verify_sun.py > runs/sun-light-smoke-0pbj3i36/console.log 2>&1
python3 scripts/control_so101.py --trajectory runs/my-trajectory --check > runs/sun-light-smoke-0pbj3i36/preflight.json
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_replay_viewing.py' -v
```

All commands exited 0; the five offline viewing tests passed. The local harness used the actual replay code with a rendered viewport and the OS window hidden, checked the Sun's USD type/name/intensity/temperature/angle, and captured `initial.png` and `midpoint.png`. Both images were visually inspected: the robot was illuminated against a gray backdrop. Replay saved 963 samples with maximum tracking error 0.0303253 rad. Preview/final viewing did not advance physics. Native window interactions, the full editor, and a separate headless replay were not retested. This confirms the current rendered result; it does not establish the cause of the user's earlier black window. The ignored artifact directory preserves the harness, captures, observations, console output, and raw replay logs.

### Renderer restored after black-background regression (2026-11-19)

The user's replay `runs/so101-20261119T112354Z-japi86gm/` recorded
`MinimalRendering` with the correct gray background configuration. The script
had been changed to select Minimal for every run, reproducing the previously
observed black backdrop. Restored `RaytracedLighting` and made the recorded
`minimal_shading_mode` depend on the selected renderer rather than headless mode.
The supported renderer names are documented in NVIDIA's
[6.2 SimulationApp API](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/extensions/isaacsim.simulation_app/docs/index.html).

Executed from the project root:

```sh
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_replay_viewing.py' -v
python3 -m py_compile scripts/control_so101.py
python3 scripts/control_so101.py --trajectory runs/my-trajectory --check
ISAAC_SIM_PATH=/home/yifan/isaacsim-6.2.0 timeout -k 10 180 bash scripts/launch_isaacsim.sh --python runs/background-restore-8n_c7k_o/verify_background.py > runs/background-restore-8n_c7k_o/console.log 2>&1
git diff --check
```

All commands passed, including five offline viewing tests. The local capture
harness uses the actual replay code with its OS window hidden, adapted from the
earlier harness without its unrelated Sun-property assertions. The replay
completed with 963 samples and maximum tracking error 0.0303253 rad. Initial and
midpoint captures were visually inspected and show the robot against gray;
pixel (10, 10) is RGB `[217, 217, 217]` in both. Artifacts and logs are preserved
under `runs/background-restore-8n_c7k_o/`. Native window interaction and a separate
headless replay were not retested. Existing processes require a restart to use
the changed renderer.

## Offline waypoint converter

On 2026-10-28, added `scripts/convert_waypoints.py` and the `so101-waypoints/1`
input contract documented in [trajectory-input.md](trajectory-input.md). The
converter accepts supplied timed joint poses and writes the existing reference
format; it does not generate random poses. Analytic checks cover every
rest-to-rest quintic segment, followed by the existing replay sample screening.

Executed commands:

```sh
PYTHONPATH=src python3 -m unittest discover -s tests -v
python3 scripts/convert_waypoints.py configs/example-waypoints.json --check
python3 scripts/convert_waypoints.py configs/example-waypoints.json
python3 scripts/convert_waypoints.py --help
python3 scripts/control_so101.py --trajectory runs/waypoints-20261028T202718Z-455cb73140a9 --check
python3 -m compileall -q src scripts tests
git diff --check
```

All **49 offline tests passed, zero skipped**, including 16 new tests using
synthetic assets. Coverage includes deterministic multi-joint interpolation,
column permutations, holds and nonzero final poses, analytic limit violations
hidden between stored samples, malformed input/configuration, output allocation
bounds, CLI round trips/provenance, check-only operation, and overwrite rejection.
Compilation and whitespace checks passed.

The example conversion used the downloaded SO-101 model and produced 481 samples
over eight reference seconds in the unique run directory above. The existing
replay entry point loaded and validated those files successfully. Analytic peak
speed was 0.09375 rad/s and acceleration was 0.07216878365 rad/s²; the replay
sample estimates were 0.09374565981 rad/s and 0.07216324870 rad/s².

No Isaac Sim replay was executed for this converter addition; dynamic tracking
of this new multi-joint example is unverified. Collision, workspace, jerk, and
measured dynamics remain unchecked, and exported candidates are not marked as
validated research data. Earlier simulator results below and above concern the
previously recorded runs, not this new example.

## Scene selection checks

On 2026-10-28, added independent scene selection through `--scene` and
`configs/scenes.json`. The `empty` scene preserves the previous setup; `table`
loads the project-authored static environment in `configs/scenes/table.usda`.
The verified robot asset/revision, origin placement, name mapping, 120 Hz physics,
60 Hz commands, and gripper policy remain the same. See
[scene-selection.md](scene-selection.md) for the supported stage contract.

Executed commands from the repository root:

```sh
python3 scripts/control_so101.py --list-scenes
python3 scripts/control_so101.py --scene table --trajectory runs/my-trajectory --check
python3 scripts/control_so101.py --help
PYTHONPATH=src python3 -m unittest discover -s tests -v
python3 -m compileall -q src scripts tests
git diff --check
ISAAC_SIM_PATH=/home/yifan/isaacsim-6.2.0 timeout -k 10 150 bash scripts/launch_isaacsim.sh \
  --python scripts/control_so101.py --scene table --trajectory runs/my-trajectory --headless
ISAAC_SIM_PATH=/home/yifan/isaacsim-6.2.0 timeout -k 10 150 bash scripts/launch_isaacsim.sh \
  --python tests/simulator/test_scene_loading.py
ISAAC_SIM_PATH=/home/yifan/isaacsim-6.2.0 timeout -k 10 150 bash scripts/launch_isaacsim.sh \
  --python scripts/control_so101.py --trajectory runs/my-trajectory --headless
ISAAC_SIM_PATH=/home/yifan/isaacsim-6.2.0 timeout -k 10 150 bash scripts/launch_isaacsim.sh \
  --python runs/scene-selection-verification/verify_rendered.py
```

All commands exited 0. **65 offline tests passed, zero skipped**; the 16 scene
tests cover catalog validation, file hashes, path resolution from another working
directory, CLI errors, and preserving reference metadata/arrays. **11 real USD
tests passed, zero skipped** through the simulator launcher, including source
preservation after export, relative nested references, and rejecting bad
units/frames, reserved paths, transformed/inactive roots, additional physics
objects (including inside instances), animation, missing references, malformed
USD, and changed source hashes. Ordinary Python invocation of the separate
simulator test script explicitly reported **SKIPPED** without Isaac Sim.

The table headless run is `runs/so101-20261028T220431Z-lxpo8i7b/`; default empty
is `runs/so101-20261028T220625Z-4jxekoi0/`. Both saved 963 measured samples over
8.0166667 simulation seconds and had maximum tracking error 0.0303252544 rad.
Reloading both logs confirmed exact source commands after name mapping, the
120 Hz timestamp schedule, unchanged reference-file hashes, and the selected
scene metadata. The table source hash also remained unchanged. All four raw
arrays were identical between these two particular runs; this is not a general
promise that scene changes leave dynamics unchanged.

The local rendered harness exercised the windowed `RaytracedLighting` path with
only its OS window hidden. The real replay completed with 963 samples; its four
raw arrays matched the table headless run. Initial and midpoint PNGs were visually
inspected and showed the arm above the tabletop and a changed pose. The harness
verified render-only waits did not advance physics and logs existed before the
final viewer, then exited via a handled KeyboardInterrupt. Native window
interactions were not separately retested. Logs, captures, harness, and data
comparison results are preserved under ignored
`runs/scene-selection-verification/`, with `verification.json` summarizing them.

Only the included static environments were replayed. User-created stages,
texture/material dependency completeness, arbitrary robot placement, dynamic
scene objects, collision/workspace validity, and measured acceleration remain
unverified or unsupported. Root USD hashes do not cover referenced dependencies;
those remain explicitly unchecked. No run is marked eligible for a validated
research dataset.

## CSV reader checks

On 2026-11-05, completed direct radian CSV reading and offline SO-101 replay
preparation. The user selected `robot_time_s` for playback. `source_time_s` and
tool XYZ remain separate source context; tool frames and source robot calibration
are unspecified by these files. Named SO-101 columns are mapped by name, the
omitted gripper holds zero, and CSV references declare initialization at the
first pose. The original data and project limits were not changed.

Executed from the repository root:

```sh
python3 scripts/read_trajectory.py data/dataset-1_kick_normal_001_right_so101_option-9.csv
python3 scripts/read_trajectory.py data/dataset-1_kick_normal_001_right_gen3_option-9.csv
python3 scripts/control_so101.py --trajectory data/dataset-1_kick_normal_001_right_so101_option-9.csv --check
python3 scripts/control_so101.py --trajectory runs/my-trajectory --check
PYTHONPATH=src python3 -m unittest discover -s tests -v
python3 -m compileall -q src scripts tests
git diff --check
```

Both reader commands exited 0 and reported 358 source rows, robot duration
110.914143210 seconds, and source duration 11.899988100 seconds. The SO-101 CSV
preflight exited 2 with `CSV source velocity for shoulder_lift reaches 1.22352
rad/s; applied limit is 0.5 rad/s`. This is a motion-limit rejection, not a
parsing failure. The existing `runs/my-trajectory` preflight passed. Compilation
and whitespace checks passed. **85 offline tests passed, zero skipped**.
Synthetic tests cover reordered headers, distinct clocks and tool columns,
nonzero initialization, endpoint padding, malformed input, bounded sampling,
source spikes missed by the control grid, Gen3 replay rejection, archive
round trips, changed-source detection, and retaining existing replay limits.

Simulator checks for this initial CSV addition were **not run**: the supplied
motion failed preflight with the original limits. No claim is made about its dynamic tracking.
Linear interpolation does not provide continuous acceleration or jerk bounds;
the replay acceleration check uses finite differences at the control rate.
Collision/workspace checks and source calibration remain unverified.

## CSV reference limit adjustment

Later on 2026-11-05, at the user's request, raised
`configs/so101-demo.json` project reference velocity from **0.5 to 1.5 rad/s**
and reference acceleration from **1.0 to 40.0 rad/s²**. Values were selected
above the supplied SO-101 CSV's sampled peaks of **1.223521034 rad/s** and
**32.702265660 rad/s²**, using `robot_time_s` and the existing 60 Hz control grid.
The asset's velocity limit remains 10 rad/s for each joint; it is still applied
alongside the project limit. Position limits/margins, tracking thresholds,
drive gains, measured-state checks, raw CSV data, and timing are unchanged.
The separate dummy generation configuration retains its original limits.

Executed:

```sh
python3 scripts/control_so101.py --trajectory data/dataset-1_kick_normal_001_right_so101_option-9.csv --check
PYTHONPATH=src python3 -m unittest discover -s tests -v
git diff --check
```

The command exited 0, validating 6,656 reference samples through
110.916666667 seconds, including endpoint padding. **85 offline tests passed,
zero skipped**; synthetic limit-rejection tests use explicit fixture thresholds
independent of these operating defaults. The whitespace check passed. This confirms offline
reference acceptance only. Simulator replay was not run for this adjustment;
dynamic tracking, continuous acceleration, collisions, and workspace validity
remain unverified. The configured acceleration threshold is a project choice
for sampled reference screening, not a manufacturer limit.

## Interpretation and remaining checks

These runs establish the demonstrated importer, command schedule, file handoff, name mapping, measured-state logging, and windowed visibility for the recorded setup. They do not establish hardware safety, collision freedom, or expressive validity. Gains and reference limits are project-selected demonstration settings rather than calibrated motor specifications. Self-collision is disabled; workspace/collision checks and measured acceleration validation remain unimplemented. Initial/midpoint viewport captures were inspected for the updated separated workflow; full-editor startup remains unresolved.
