"""Scene selection tests need neither Isaac Sim, NumPy, nor downloaded assets."""
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest

from isaacsim_trajectories.scenes import load_scene_catalog, resolve_scene


class SceneCatalogTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.catalog = self.root / "configuration" / "scenes.json"
        self.catalog.parent.mkdir()
        self.document = {
            "schema_version": "so101-scenes/1",
            "scenes": {
                "empty": {"id": "empty/1", "description": "Built-in empty scene", "stage": None},
                "room-A_2": {"id": "room/1", "description": "Local room", "stage": "../room.usda"},
            },
        }

    def save(self, document=None):
        self.catalog.write_text(json.dumps(self.document if document is None else document))

    def test_empty_scene_and_unavailable_unselected_stage(self):
        self.save()
        self.assertEqual(load_scene_catalog(self.catalog), self.document["scenes"])
        scene = resolve_scene(self.catalog, "empty")
        self.assertEqual(scene["name"], "empty")
        self.assertIsNone(scene["stage"])
        self.assertIsNone(scene["stage_sha256"])
        self.assertEqual(scene["stage_validation"], "built-in empty")
        self.assertEqual(scene["dependency_hashes"], "unchecked")
        with self.assertRaisesRegex(FileNotFoundError, "room-A_2.*stage file"):
            resolve_scene(self.catalog, "room-A_2")

    def test_relative_path_is_catalog_relative_and_hashes_are_of_source_files(self):
        self.save()
        stage = self.root / "room.usda"
        original = b"#usda 1.0\n# offline fixture; no USD parsing claimed\n"
        stage.write_bytes(original)
        elsewhere = self.root / "different-working-directory"
        elsewhere.mkdir()
        previous = Path.cwd()
        try:
            os.chdir(elsewhere)
            scene = resolve_scene(self.catalog, "room-A_2")
        finally:
            os.chdir(previous)
        self.assertEqual(scene["stage"], str(stage.resolve()))
        self.assertEqual(scene["catalog_path"], str(self.catalog.resolve()))
        self.assertEqual(scene["catalog_sha256"], hashlib.sha256(self.catalog.read_bytes()).hexdigest())
        self.assertEqual(scene["stage_sha256"], hashlib.sha256(original).hexdigest())
        self.assertEqual(scene["stage_validation"], "pending simulator inspection")
        self.assertEqual(scene["meters_per_unit"], 1.0)
        self.assertEqual(scene["up_axis"], "Z")
        self.assertEqual(scene["gravity_m_s2"], [0, 0, -9.81])
        stage.write_bytes(original + b"# changed\n")
        changed = resolve_scene(self.catalog, "room-A_2")
        self.assertNotEqual(changed["stage_sha256"], scene["stage_sha256"])
        self.assertEqual(changed["catalog_sha256"], scene["catalog_sha256"])

    def test_absolute_local_paths_and_all_supported_extensions(self):
        for extension in (".usd", ".usda", ".usdc"):
            with self.subTest(extension=extension):
                stage = self.root / f"room with spaces{extension}"
                stage.write_bytes(b"offline fixture")
                self.document["scenes"]["room-A_2"]["stage"] = str(stage)
                self.save()
                self.assertEqual(resolve_scene(self.catalog, "room-A_2")["stage"], str(stage.resolve()))

    def test_unknown_scene_lists_available_names(self):
        self.save()
        with self.assertRaisesRegex(ValueError, "Unknown scene.*Available scenes: empty, room-A_2"):
            resolve_scene(self.catalog, "unknown")

    def test_selected_directory_is_not_a_stage_file(self):
        (self.root / "room.usda").mkdir()
        self.save()
        with self.assertRaisesRegex(FileNotFoundError, "stage file"):
            resolve_scene(self.catalog, "room-A_2")

    def test_malformed_top_level_schema_and_fields(self):
        documents = [None, [], {}, {"schema_version": "so101-scenes/1"},
                     {**self.document, "extra": 1},
                     {**self.document, "schema_version": "so101-scenes/2"}]
        documents.extend({**self.document, "scenes": value} for value in ([], None, {}, "empty"))
        for document in documents:
            with self.subTest(document=document):
                self.catalog.write_text(json.dumps(document))
                with self.assertRaises(ValueError):
                    load_scene_catalog(self.catalog)

    def test_invalid_names_and_duplicate_ids(self):
        for name in ("", "a b", "../room", "room.usd", "room/name", "é"):
            with self.subTest(name=name):
                self.save({**self.document, "scenes": {name: self.document["scenes"]["empty"]}})
                with self.assertRaisesRegex(ValueError, "Invalid scene name"):
                    load_scene_catalog(self.catalog)
        self.document["scenes"]["room-A_2"]["id"] = "empty/1"
        self.save()
        with self.assertRaisesRegex(ValueError, "Duplicate scene id"):
            load_scene_catalog(self.catalog)

    def test_every_entry_is_validated_even_if_not_selected(self):
        invalid_entries = [None, [], {}, {**self.document["scenes"]["room-A_2"], "extra": False}]
        invalid_entries.extend(
            {**self.document["scenes"]["room-A_2"], field: value}
            for field in ("id", "description") for value in (None, False, 1, "", " \t"))
        for entry in invalid_entries:
            with self.subTest(entry=entry):
                document = copy.deepcopy(self.document)
                document["scenes"]["room-A_2"] = entry
                self.save(document)
                with self.assertRaises(ValueError):
                    resolve_scene(self.catalog, "empty")

    def test_nonlocal_or_invalid_stage_paths(self):
        for stage in (False, 0, [], {}, "", " ", "scene.json", "scene", "bad\x00.usd",
                      "https://example.com/room.usd", "omniverse://host/room.usd",
                      "file:///tmp/room.usd", "//example.com/room.usd", "asset:room.usd"):
            with self.subTest(stage=stage):
                document = copy.deepcopy(self.document)
                document["scenes"]["room-A_2"]["stage"] = stage
                self.save(document)
                with self.assertRaises(ValueError):
                    load_scene_catalog(self.catalog)

    def test_duplicate_keys_at_every_level_and_nonfinite_json_are_rejected(self):
        documents = [
            '{"schema_version":"so101-scenes/1","schema_version":"so101-scenes/1","scenes":{}}',
            '{"schema_version":"so101-scenes/1","scenes":{"a":{},"a":{}}}',
            '{"schema_version":"so101-scenes/1","scenes":{"a":{"id":"x","id":"y","description":"d","stage":null}}}',
            '{"schema_version":"so101-scenes/1","scenes":{"a":{"id":"x","description":"d","stage":NaN}}}',
        ]
        for document in documents:
            with self.subTest(document=document):
                self.catalog.write_text(document)
                with self.assertRaises(ValueError):
                    load_scene_catalog(self.catalog)


if __name__ == "__main__":
    unittest.main()
