"""Small provenance helpers shared by offline generation and simulator replay."""
from pathlib import Path
import subprocess


def git_identity(directory: Path) -> dict:
    """Record local revision and dirty state without requiring a Git checkout."""
    try:
        revision = subprocess.check_output(
            ["git", "-C", str(directory), "rev-parse", "HEAD"],
            stderr=subprocess.DEVNULL, text=True,
        ).strip()
        dirty = bool(subprocess.check_output(
            ["git", "-C", str(directory), "status", "--porcelain"],
            stderr=subprocess.DEVNULL, text=True,
        ).strip())
        return {"revision": revision, "dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"revision": None, "dirty": None}
