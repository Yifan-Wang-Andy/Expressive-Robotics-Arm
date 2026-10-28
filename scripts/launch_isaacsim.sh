#!/usr/bin/env bash
# Launch the verified Isaac Sim 6.2 workstation build with its own CUDA runtime.
set -euo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    cat <<'EOF'
Usage: ISAAC_SIM_PATH=/absolute/install/path bash scripts/launch_isaacsim.sh [Isaac Sim options]
       ISAAC_SIM_PATH=/absolute/install/path bash scripts/launch_isaacsim.sh --python [Python arguments]

Requires the Isaac Sim 6.2.0 Linux workstation installation. Uses its bundled
CUDA runtime and ignores inherited Python/Conda library overrides for this
process. ROS environment setup and GUI ROS bridge startup are disabled.
No system settings are changed.
EOF
    exit 0
fi

if [[ -z "${ISAAC_SIM_PATH:-}" || "$ISAAC_SIM_PATH" != /* ]]; then
    echo "Set ISAAC_SIM_PATH to the absolute path of the Isaac Sim 6.2.0 installation." >&2
    exit 2
fi
if [[ ! -d "$ISAAC_SIM_PATH" ]]; then
    echo "Isaac Sim installation directory does not exist: $ISAAC_SIM_PATH" >&2
    exit 2
fi
isaac_root=$(cd -- "$ISAAC_SIM_PATH" && pwd -P)
if [[ ! -r "$isaac_root/VERSION" ]]; then
    echo "Missing Isaac Sim VERSION file: $isaac_root/VERSION" >&2
    exit 2
fi
isaac_version=$(<"$isaac_root/VERSION")
case "$isaac_version" in
    6.2.0|6.2.0-*) ;;
    *) echo "This launch workaround was verified for 6.2.0; found: $isaac_version" >&2; exit 2 ;;
esac

cuda_runtime_dir="$isaac_root/exts/isaacsim.pip.nv/pip_prebundle/nvidia/cuda_runtime/lib"
if [[ ! -r "$cuda_runtime_dir/libcudart.so.12" ]]; then
    echo "Missing bundled CUDA runtime: $cuda_runtime_dir/libcudart.so.12" >&2
    exit 2
fi

launcher="$isaac_root/isaac-sim.sh"
launch_options=(--/isaac/startup/ros_bridge_extension= --/isaac/startup/ros_sim_control_extension=false)
if [[ "${1:-}" == "--python" ]]; then
    shift
    launcher="$isaac_root/python.sh"
    launch_options=()
fi
if [[ ! -x "$launcher" ]]; then
    echo "Missing executable Isaac Sim launcher: $launcher" >&2
    exit 2
fi

# A system CUDA 12.0 library can satisfy the soname but miss symbols required
# by bundled torch 2.11/cu128. Select CUDA 12.8 before Kit/PyTorch load it.
# Keep these overrides local to this child process; preserve display variables.
unset LD_PRELOAD PYTHONPATH PYTHONHOME PYTHONEXE CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV
export LD_LIBRARY_PATH="$cuda_runtime_dir"
export PYTHONNOUSERSITE=1
exec "$launcher" --no-ros-env "${launch_options[@]}" "$@"
