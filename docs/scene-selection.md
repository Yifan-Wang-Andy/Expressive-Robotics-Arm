# Selecting scenes and trajectories

Scenes and joint trajectories are independent inputs to the same replay script.
Choose any compatible pair without editing Python:

```sh
python3 scripts/control_so101.py --list-scenes

bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --scene table --trajectory runs/my-trajectory

bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
  --scene empty --trajectory runs/my-trajectory
```

These commands assume the trajectory has already been generated and
`ISAAC_SIM_PATH` points to the installed simulator, as described in the README.
Omitting `--scene` selects `empty` and preserves the original scene setup.
`--headless`, `--playback-speed`, and other existing replay options still apply.

The supplied [scene catalog](../configs/scenes.json) has two entries:

| Name | Environment |
| --- | --- |
| `empty` | Original empty stage. |
| `table` | Project-authored [USD tabletop and visual marker](../configs/scenes/table.usda). |

The table is a simple example with a static collider. Its top is at z = −0.05 m;
the arm remains fixed at the world origin. The marker has no collision API.
This arrangement is not collision-validated or an expressive-state stimulus.

## Add an environment

Start by copying `configs/scenes/table.usda` to another USD file and editing its
environment children, in a text editor or Isaac Sim. Save the environment before
running replay. Add a new entry inside `scenes` in the catalog:

```json
"desk": {
  "id": "so101-desk/1",
  "description": "My desk environment",
  "stage": "scenes/desk.usda"
}
```

Then select it with `--scene desk`. Names use letters, digits, underscores, or
hyphens; IDs must be unique. Each entry requires exactly `id`, `description`, and
`stage`. A null stage selects a built-in empty environment. The top-level schema
is `so101-scenes/1`.

Stage paths resolve relative to the **catalog file**, regardless of the shell's
working directory. Local `.usd`, `.usda`, and `.usdc` files are supported. A
separate catalog can be selected with `--scenes-config configs/scenes.local.json`;
`*.local.json` is ignored by Git and is suitable for machine-specific absolute
paths. Large/downloaded environments belong under ignored `assets/downloaded/`
with their dependencies kept alongside them; record their source and license.

This first implementation accepts **static environment-only stages**:

- Use meters (`metersPerUnit = 1`) and Z-up. Units are checked, never silently converted.
- Keep `/World` as an active, non-instanced container with an identity transform.
  Place and transform your geometry under `/World/Environment` instead. The robot
  is added at the world origin; arbitrary robot placement is not implemented.
- Leave `/World/so101`, `/World/light`, and `/World/physicsScene` unused. Replay
  adds the verified SO-101, its lighting, and its physics scene.
- Do not include robots, rigid bodies, joints, physics scenes, or animated
  attributes. Static collision geometry, materials, lights, and cameras are allowed.
- Keep all USD references and payloads resolvable. Missing composition sources
  fail startup. Complete texture/material dependency validation is not implemented.

Replay composes the selected file as a sublayer of a fresh stage and writes its
own changes to the new root layer. It does not save edits into your source USD.
Relative references inside that USD continue resolving relative to their source
layers. Each run exports `scene.usda` beside its logs, but this file still refers
to the original environment and imported robot files; it is not a standalone
copy of all assets. Preserve those sources with your experiment.

The fixed base, zero-radian gripper target, gravity −9.81 m/s², 120 Hz physics,
and 60 Hz commands remain controlled by the existing replay setup. Joint values
are mapped by name. Scene selection does not rewrite or resample the trajectory.

## Run several combinations

For an explicit list of pairs, a small shell loop can invoke the same script:

```sh
while read -r scene trajectory; do
  bash scripts/launch_isaacsim.sh --python scripts/control_so101.py \
    --scene "$scene" --trajectory "$trajectory" --headless || break
done <<'PAIRS'
empty runs/my-trajectory
table runs/my-trajectory
table runs/wrist-trajectory
PAIRS
```

Generate both trajectory directories first. The example assumes paths without
spaces. Every invocation starts a fresh simulator and creates a unique run
directory; the loop stops at the first failure. For interactive inspection, run
one pair without `--headless` instead.

## Checks and provenance

```sh
python3 scripts/control_so101.py \
  --scene table --trajectory runs/my-trajectory --check
```

This offline check validates the catalog, selected file's existence, model, and
reference motion. It reports the selected scene and hashes. USD content checks
remain **pending simulator inspection** until replay starts. Listing scenes does
not require installed assets, a trajectory, or Isaac Sim, and does not require
every listed stage to exist on the current machine.

Measured log metadata records the actual scene under `scene`, its catalog and
root USD SHA-256 hashes, source paths, units, frame, and composition-check result.
The original reference metadata remains under `trajectory.metadata`, including
the scene recorded when that reference was generated. A root USD hash does not
cover referenced assets: `dependency_hashes` remains `unchecked`.

Changing a scene does not establish collision or workspace validity. Both checks
remain `unchecked`, and logs retain `dataset_eligible: false` even after a
successful replay.

The implementation uses the inspected Isaac Sim 6.2 lifecycle and its
[stage utilities](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/extensions/isaacsim.core.experimental.utils/docs/index.html),
with USD sublayer composition after application startup. Separately selectable
simulator checks are run through the same launcher:

```sh
bash scripts/launch_isaacsim.sh --python tests/simulator/test_scene_loading.py
```

See [verification.md](verification.md) for checks actually executed and remaining
limitations.
