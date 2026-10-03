"""Planning and protected handover for one assembly runtime."""
from __future__ import annotations

import contextlib
import io
from dataclasses import dataclass

import mujoco
import numpy as np
import pinocchio as pin
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

from compliant_docking.planning.kinematics import compute_ik


@dataclass
class Phase:
    key: str
    title: str
    seconds: float
    end: np.ndarray
    event: str | None = None

def reference_ik(model, kin, q, position, rotation):
    q = q.copy()
    for _ in range(4):
        kin.qpos[:7] = q
        mujoco.mj_kinematics(model, kin)
        mujoco.mj_comPos(model, kin)
        site = kin.site("gripper_tip")
        error = np.r_[position - site.xpos,
                      Rotation.from_matrix(rotation @ site.xmat.reshape(3, 3).T).as_rotvec()]
        if np.linalg.norm(error) < 1e-8:
            break
        jac = np.zeros((6, model.nv))
        mujoco.mj_jacSite(model, kin, jac[:3], jac[3:], site.id)
        jac = jac[:, :7]
        q += jac.T @ np.linalg.solve(jac @ jac.T + 1e-8 * np.eye(6), error)
    if np.linalg.norm(error) > 1e-5:
        raise RuntimeError(f"Reference IK residual {np.linalg.norm(error)}")
    return q

class Sequence:
    def plan(self, model, initial_q=None, phase_list=None):
        pm = self.pin_model
        pd = pm.createData()
        q = self.initial_q.copy()
        if initial_q is not None:
            q = np.asarray(initial_q).copy()
        data = mujoco.MjData(model)
        if phase_list is None:
            phase_list = self.phases()
        start = phase_list[0].end.copy()
        qs, ts, phase_ids = [], [], []
        time = 0.
        max_fk = 0.
        with contextlib.redirect_stdout(io.StringIO()):
            for k, phase in enumerate(phase_list):
                for tau in np.linspace(0., phase.seconds, max(2, int(phase.seconds*30)+1)):
                    if ts and tau == 0:
                        continue
                    u = tau/phase.seconds
                    s = 10*u**3 - 15*u**4 + 6*u**5
                    p = start + s*(phase.end-start)
                    q, ok = compute_ik(pm, pd, pin.SE3(self.tip_rotation, p), initial_q=q, max_iters=1000, ee_frame="gripper_tip")
                    if not ok:
                        raise ValueError(f"Unreachable trajectory: {phase.key}, {tau}")
                    data.qpos[:7] = q
                    mujoco.mj_forward(model, data)
                    tip = data.site("gripper_tip")
                    max_fk = max(max_fk, float(np.linalg.norm(tip.xpos-p)))
                    qs.append(q.copy())
                    ts.append(time+tau)
                    phase_ids.append(k)
                start = phase.end.copy()
                time += phase.seconds
        assert max_fk < 1e-6
        qs, ts = np.array(qs), np.array(ts)
        spline = CubicSpline(ts, qs, bc_type=((1, np.zeros(7)), (1, np.zeros(7))))
        return spline, phase_list, dict(ik_samples=len(ts), fk_max_error_m=max_fk, spline_joint_limit_margin_rad=float(np.min(np.minimum(spline(np.linspace(0, time, 5000))-pm.lowerPositionLimit, pm.upperPositionLimit-spline(np.linspace(0, time, 5000))))),
                                      joint_limits=[pm.lowerPositionLimit.tolist(), pm.upperPositionLimit.tolist()])

    def lock_error(self, model, data, index):
        a, b = [data.site(name) for name in self.lock_sites[index]]
        dp = float(np.linalg.norm(a.xpos-b.xpos))
        dr = float(np.linalg.norm(pin.log3(a.xmat.reshape(3, 3)@b.xmat.reshape(3, 3).T)))
        jacobians = []
        for site in [a, b]:
            j = np.zeros((6, model.nv))
            mujoco.mj_jacSite(model, data, j[:3], j[3:], site.id)
            jacobians.append(j)
        relative = (jacobians[0]-jacobians[1])@data.qvel
        return dp, dr, float(np.linalg.norm(relative[:3])), float(np.linalg.norm(relative[3:]))

    def handover(self, model, data, event):
        """Enable the receiving latch before releasing the current support."""
        if event not in {"grip_on", "rack_off", "assembly_on", "grip_off"}:
            raise ValueError(event)
        if event == "assembly_on":
            raise RuntimeError("Assembly lock requires continuous physical seating in the simulation gate")
        index = 1 if event.startswith("grip") else 0 if event == "rack_off" else 2
        if event.endswith("on"):
            error = self.lock_error(model, data, index)
            if not (error[0] < .001 and error[1] < np.deg2rad(.5)
                    and error[2] < .003 and error[3] < np.deg2rad(2.)):
                raise RuntimeError(f"Latch gate failed: {event}: {error}")
            data.eq_active[index] = True
        else:
            support = 1 if event == "rack_off" else 2
            error = self.lock_error(model, data, support)
            if not data.eq_active[support] or error[0] > .001 or error[1] > np.deg2rad(.5):
                raise RuntimeError(f"Cannot release {event}: receiving latch unconfirmed")
            data.eq_active[index] = False
        assert np.any(data.eq_active)
        return error

    def verify_interlocks(self, model, initial_q):
        """Reject remote capture and both unsafe release operations."""
        data = mujoco.MjData(model)
        data.qpos[:7] = initial_q
        mujoco.mj_forward(model, data)
        rejected = []
        for event in ["grip_on", "rack_off", "grip_off"]:
            before = data.eq_active.copy()
            try:
                self.handover(model, data, event)
            except RuntimeError:
                np.testing.assert_array_equal(data.eq_active, before)
                rejected.append(event)
            else:
                raise AssertionError(f"Unsafe event accepted: {event}")
        return rejected

