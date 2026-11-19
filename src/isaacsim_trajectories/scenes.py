"""Offline selection and provenance for environment-only replay stages.

USD contents, dependencies, units, and physics objects are inspected by the
simulator entry point. A successful resolution here does not validate a stage
for simulation or establish collision freedom.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
from typing import Any
from urllib.parse import urlsplit

SCHEMA_VERSION = "so101-scenes/1"


def _unique_object(pairs: list[tuple[str, Any]]) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key in scene catalog: {key!r}")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ValueError(f"Scene catalog must contain finite JSON data, found {value}")


def _parse_catalog(contents: bytes) -> dict[str, dict]:
    document = json.loads(contents, object_pairs_hook=_unique_object,
                          parse_constant=_reject_constant)
    if not isinstance(document, dict) or set(document) != {"schema_version", "scenes"}:
        raise ValueError("Scene catalog must contain exactly schema_version and scenes")
    if document["schema_version"] != SCHEMA_VERSION:
        raise ValueError("Unsupported scene catalog schema")
    scenes = document["scenes"]
    if not isinstance(scenes, dict) or not scenes:
        raise ValueError("Scene catalog scenes must be a nonempty object")
    ids = set()
    for name, scene in scenes.items():
        if re.fullmatch(r"[A-Za-z0-9_-]+", name) is None:
            raise ValueError(f"Invalid scene name {name!r}; use letters, digits, underscores, or hyphens")
        if not isinstance(scene, dict) or set(scene) != {"id", "description", "stage"}:
            raise ValueError(f"Scene {name!r} must contain exactly id, description, and stage")
        for field in ("id", "description"):
            if not isinstance(scene[field], str) or not scene[field].strip():
                raise ValueError(f"Scene {name!r} {field} must be a nonempty string")
        if scene["id"] in ids:
            raise ValueError(f"Duplicate scene id: {scene['id']!r}")
        ids.add(scene["id"])
        stage = scene["stage"]
        if stage is not None:
            if not isinstance(stage, str) or not stage.strip():
                raise ValueError(f"Scene {name!r} stage must be null or a nonempty local USD path")
            if urlsplit(stage).scheme or stage.startswith("//"):
                raise ValueError(f"Scene {name!r} stage must be a local path; URLs are unsupported")
            if "\x00" in stage or Path(stage).suffix.lower() not in {".usd", ".usda", ".usdc"}:
                raise ValueError(f"Scene {name!r} stage must be a .usd, .usda, or .usdc file")
    return scenes


def load_scene_catalog(path: Path) -> dict[str, dict]:
    """Validate every entry without opening any selected or unselected USD file.

    This lets a portable catalog list scenes unavailable on the current machine.
    Relative stage paths remain relative to the catalog until resolution.
    """
    return _parse_catalog(Path(path).read_bytes())


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_scene(catalog_path: Path, name: str) -> dict:
    """Resolve one scene and record the actual catalog and root-file hashes.

    Only the selected stage must exist. USD validation remains pending until
    simulator startup, and the hash does not cover referenced dependencies.
    """
    catalog_path = Path(catalog_path).resolve()
    contents = catalog_path.read_bytes()
    scenes = _parse_catalog(contents)
    if name not in scenes:
        raise ValueError(f"Unknown scene {name!r}. Available scenes: {', '.join(sorted(scenes))}")
    scene = scenes[name]
    stage = None
    stage_sha256 = None
    if scene["stage"] is not None:
        stage_path = Path(scene["stage"])
        if not stage_path.is_absolute():
            stage_path = catalog_path.parent / stage_path
        stage_path = stage_path.resolve()
        if not stage_path.is_file():
            raise FileNotFoundError(f"Selected scene {name!r} stage file does not exist: {stage_path}")
        stage = str(stage_path)
        stage_sha256 = _file_sha256(stage_path)
    return {
        "id": scene["id"], "name": name, "description": scene["description"],
        "stage": stage, "stage_sha256": stage_sha256,
        "catalog_path": str(catalog_path),
        "catalog_sha256": hashlib.sha256(contents).hexdigest(),
        "dependency_hashes": "unchecked",
        "meters_per_unit": 1.0, "up_axis": "Z", "gravity_m_s2": [0, 0, -9.81],
        "robot_placement": "fixed at world origin; imported SO-101",
        "stage_validation": "pending simulator inspection" if stage else "built-in empty",
    }
