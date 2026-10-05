"""Saved angular designs cannot silently resize the preserved stop/base."""
import importlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"experiments"))
builder = importlib.import_module("prepare_petal_design")


def design(**changes):
    return dict(schema="petal-guidance-design/v1",
                design_parameters=dict(builder.designer.PRESETS["narrow"], **changes))


def test_user_selected_angle_and_blend_are_actual_generator_inputs():
    p = builder.checked_parameters(design(guide_tip_half_angle_deg=1., guide_blend_fraction=.3))
    assert p["guide_tip_half_angle_deg"] == 1.
    assert p["guide_blend_fraction"] == .3
    assert p["guide_height_mm"] == 18.
    assert p["guide_axial_clearance_mm"] == .4


@pytest.mark.parametrize("changes", [{"guide_height_mm": 20.}, {"guide_axial_clearance_mm": .8}])
def test_stop_reference_change_requires_full_rebuild(changes):
    with pytest.raises(ValueError, match="rebuild"):
        builder.checked_parameters(design(**changes))


def test_private_policy_does_not_change_imported_generator(tmp_path):
    source = tmp_path/"design.json"
    source.write_text(json.dumps(design(guide_tip_half_angle_deg=1., guide_blend_fraction=.3)))
    original_policy = builder.designer.geometry.candidate_parameters
    out = tmp_path/"angle1_blend030"
    meta = builder.build(source, out)
    assert builder.designer.geometry.candidate_parameters is original_policy
    assert original_policy(builder.designer.ORIGINAL["parameters"], "narrow")["guide_tip_half_angle_deg"] == 4.21875
    assert meta["parameters"]["guide_tip_half_angle_deg"] == 1.
    assert meta["parameters"]["guide_blend_fraction"] == .3
    assert meta["nominal_flange_separation_m"] == .0464
    manifest = json.loads((out/"manifest.json").read_text())
    assert manifest["parameter_overrides"] == {"guide_tip_half_angle_deg": 1., "guide_blend_fraction": .3}
    assert manifest["generator_sha256"] != manifest["base_generator_sha256"]
    assert "selected_design.json" in manifest["imported_files"]
    with pytest.raises(ValueError, match="exists"):
        builder.build(source, out)


def test_selected_preview_keeps_default_preset_available(tmp_path, monkeypatch):
    app = builder.designer
    monkeypatch.setattr(app, "PRESETS", dict(app.PRESETS))
    monkeypatch.setattr(app, "PRESET_LABELS", dict(app.PRESET_LABELS))
    monkeypatch.setattr(app, "DEFAULT_PARAMETERS", dict(app.DEFAULT_PARAMETERS))
    path = tmp_path/"design.json"
    path.write_text(json.dumps(design(guide_tip_half_angle_deg=1., guide_blend_fraction=.3)))
    app.select_design(path)
    assert app.DEFAULT_PARAMETERS["guide_tip_half_angle_deg"] == 1.
    assert app.DEFAULT_PARAMETERS["guide_blend_fraction"] == .3
    assert app.PRESETS["narrow"]["guide_tip_half_angle_deg"] == 1.
    assert app.PRESETS["narrow"]["guide_blend_fraction"] == .3
