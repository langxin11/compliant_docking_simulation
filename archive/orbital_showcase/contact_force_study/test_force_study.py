"""Independent frame-sign and unsafe-lock regression checks."""
import importlib.util
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

spec = importlib.util.spec_from_file_location("force_study", Path(__file__).with_name("run.py"))
study = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = study
spec.loader.exec_module(study)


@pytest.mark.parametrize("reverse", [False, True])
def test_force_on_mobile_is_upward_for_either_pair_order(reverse):
    pair = ('geom1="seat_mobile_0" geom2="seat_fixed"' if reverse else
            'geom1="seat_fixed" geom2="seat_mobile_0"')
    xml = f'''<mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <geom name="seat_fixed" type="cylinder" pos="0 0 -.003" size=".042 .003"/>
        <body name="module1_port_4" pos="0 0 .0035">
          <freejoint/>
          <geom name="seat_mobile_0" type="sphere" size=".004" mass=".2"/>
          <geom name="seat_mobile_1" type="sphere" pos="1 0 0" size=".004" mass="0"/>
          <geom name="seat_mobile_2" type="sphere" pos="2 0 0" size=".004" mass="0"/>
        </body>
      </worldbody>
      <contact><pair {pair}/></contact>
    </mujoco>'''
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    force, _, contacts, unexpected, _ = study.seating_wrench(model, data)
    assert contacts == 1 and not unexpected
    assert force[2] > 0
    np.testing.assert_allclose(force[:2], 0., atol=1e-12)
    # Free mass receives precisely the contact force; no weld or actuator exists.
    np.testing.assert_allclose(data.qacc[:3] * .2, force, rtol=1e-8, atol=1e-10)


@pytest.mark.parametrize("changes", [dict(force=0.), dict(force=14.), dict(contacts=1),
                                      dict(speed=.003), dict(angle_error=np.deg2rad(1.)),
                                      dict(position_error=.01), dict(lateral=3.), dict(moment=1.)])
def test_gate_rejects_unsafe_capture(changes):
    values = dict(force=8., target=8., lateral=.1, moment=.01, speed=0.,
                  position_error=.0001, angle_error=0., contacts=3)
    assert study.lock_ready(**values)
    values.update(changes)
    assert not study.lock_ready(**values)
