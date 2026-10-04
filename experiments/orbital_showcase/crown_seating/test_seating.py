"""Reject convincing-looking but unsafe seating states, and audit the free pair."""
import importlib.util
from pathlib import Path

import mujoco
import numpy as np
import pytest

spec = importlib.util.spec_from_file_location("crown_pair", Path(__file__).with_name("run.py"))
study = importlib.util.module_from_spec(spec)
spec.loader.exec_module(study)


@pytest.mark.parametrize("change", [
    {"gap": .002},  # Force present while jammed above seat.
    {"lateral": .001}, {"angle": 2.},
    {"speed": .002}, {"spin": .1},
    {"force": 0.}, {"force": 11.}, {"penetration": .001},
    {"force": float("nan")},
])
def test_reject_unsafe_lock(change):
    values = dict(lateral=0., gap=0., angle=0., speed=0., spin=0., force=1., penetration=0.)
    values.update(change)
    assert not study.gate(**values)


def test_pair_mass_and_clear_approach():
    model = mujoco.MjModel.from_xml_string(study.model_xml(.001))
    data = mujoco.MjData(model)
    data.qpos[:3] = [0, 0, study.HEIGHT + .006]
    data.qpos[3:] = [0, 1, 0, 0]
    mujoco.mj_forward(model, data)
    assert model.nv == 6
    assert model.body_subtreemass[model.body("carrier").id] == pytest.approx(2.)
    assert data.ncon == 0
    assert not data.eq_active[0]
    assert np.all(np.isfinite(data.qacc))
