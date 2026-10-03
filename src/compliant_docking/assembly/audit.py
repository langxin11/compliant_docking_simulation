"""Independent audit of recorded physical seating, handover and provenance."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


def window_n(dt):
    """0.5 s dwell window expressed in contact-trace rows (one per physics step)."""
    n = round(.5/dt)
    assert n >= 1
    return n


def audit(output):
    report = json.loads((output/"validation.json").read_text())
    geometry = json.loads((output/"geometry_check.json").read_text())
    values = np.load(output/"contact_trace.npz")["values"]
    records = np.load(output/"rollout.npz")
    dt = json.loads((output/"runtime.json").read_text())["physics_timestep_s"]
    n = window_n(dt)
    assert report["status"] == geometry["status"] == "PASS"
    assert report["module_asset"] == "hexframe"
    assert abs(report["module_mass_kg"]-geometry["mass_kg"]) < 1e-8
    assert geometry["base_connection"]["status"] == "PASS"
    assert geometry["base_connection"]["prelocked"]
    assert geometry["base_connection"]["module_port"] == "module2_port_4"
    assert geometry["base_connection"]["mating_position_error_m"] < 1e-10
    assert geometry["storage_connection"]["status"] == "PASS"
    assert geometry["storage_connection"]["spare_empty"]
    assert geometry["storage_connection"]["module_port"] == "module1_port_4"
    assert len(values) == round(report["duration_s"]/dt)+1
    assert records["t"][-1] == report["duration_s"]
    assert np.isfinite(values).all() and np.isfinite(records["qpos"]).all()
    assert [e["event"] for e in report["events"]] == ["grip_on", "rack_off", "assembly_on", "grip_off"]
    assert records["locks"][-1].tolist() == [False, False, True]
    index = np.flatnonzero(values[:, 11])[0]
    window = values[index-n+1:index+1]
    assert len(window) == n
    for valid in [(window[:, 1] >= .15) & (window[:, 1] <= .6), window[:, 3] <= .5,
                  window[:, 5] <= .0003, window[:, 7] <= .00075,
                  window[:, 8] <= np.deg2rad(.5), window[:, 9] <= .0005,
                  window[:, 10] <= np.deg2rad(.5)]:
        assert np.all(valid)
    assert report["events"][2]["dwell_s"] >= .5
    assert np.all(window[:, 14] == 1) and np.all(window[:, 15] == 1)
    assert report["events"][3]["gripper_constraint_force_n"] <= .2
    assert report["events"][3]["unloaded_dwell_s"] >= .5
    if values.shape[1] >= 17:
        released = round(report["events"][3]["t"]/dt)
        unload = values[released-n+1:released+1]
        assert len(unload) == n
        assert np.all(unload[:, 12] <= .2) and np.all(unload[:, 16] <= .002)
        assert np.all(unload[:, 11] == 1)
    stored = np.load(output/"storage_trace.npz")["values"]
    assert len(stored) == len(values) and np.isfinite(stored).all()
    np.testing.assert_allclose(stored[:, 0], values[:, 0], atol=1e-12)
    np.testing.assert_array_equal(stored[:, 9], values[:, 11])
    assert stored[0, 7:10].tolist() == [1, 0, 0]
    assert np.all(stored[:, 7:10].sum(axis=1) >= 1)
    release_index = np.flatnonzero(stored[:, 7] == 0)[0]
    assert stored[release_index-1, 7:9].tolist() == [1, 1]
    assert stored[release_index, 7:9].tolist() == [0, 1]
    assert abs(stored[release_index, 0]-report["events"][1]["t"]) < 1e-10
    assert np.all(stored[:release_index, 6] < .001)
    phase_list = json.loads((output/"phases.json").read_text())
    lift_end = sum(p["seconds"] for p in phase_list[:6])
    cleared = stored[stored[:, 0] >= lift_end]
    assert np.all(cleared[:, 6] > .04) and np.all(cleared[:, 5] == 0)
    assert report["storage_interface"]["peak_contact_force_n"] <= 10
    assert report["storage_interface"]["max_penetration_mm"] <= .3
    assert report["peak_axial_force_n"] <= 10
    assert report["max_penetration_mm"] <= .3 and report["final_module_error_mm"] <= 1
    manifest = json.loads((output/"source_manifest.json").read_text())
    assert manifest
    for file, digest in manifest.items():
        assert hashlib.sha256(Path(file).read_bytes()).hexdigest() == digest, file
    retreat = records["phase"] == 12
    assert np.any(retreat)
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(output/"model.xml"))
    data = mujoco.MjData(model)
    for i in np.flatnonzero(retreat):
        data.qpos[:] = records["qpos"][i]
        mujoco.mj_forward(model, data)
        assert np.linalg.norm(data.site("gripper_anchor").xpos-data.site("module1_anchor").xpos) >= .07
    result = dict(final_retreat_clearance_verified=True, status="PASS", module="HexFrame", geometry_mass_and_port_mapping_verified=True,
                  handover_and_unload_verified=True, prelock_physical_samples_recomputed=n,
                  loaded_stop_continuous_dwell_verified=True, prelocked_base_interface_verified=True,
                  storage_unlock_after_grip_verified=True, storage_extraction_clearance_verified=True)
    (output/"audit.json").write_text(json.dumps(result, indent=2)+"\n")
    return result
