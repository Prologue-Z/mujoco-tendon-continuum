"""Run a fixed-base tendon-driven continuum model, with no RL dependencies."""
from pathlib import Path
import argparse
import json
import time

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
PAIR_NAMES = (('seg1_x_plus', 'seg1_x_minus'),
              ('seg2_x_plus', 'seg2_x_minus'),
              ('seg2_y_plus', 'seg2_y_minus'))


class TendonController:
    """Map three differential displacements in metres to six tendon references.

    A displacement is half the change in opposing tendon-length difference.
    The neutral lengths are evaluated at the model's initial straight pose.
    """
    def __init__(self, model, data):
        self.model, self.data = model, data
        if model.nu != 6:
            raise ValueError('This controller expects the standalone six-actuator component')
        self.pairs = []
        for names in PAIR_NAMES:
            ids = tuple(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in names)
            if min(ids) < 0:
                raise ValueError(f'Missing tendon-actuator pair: {names}')
            self.pairs.append(ids)
        mujoco.mj_forward(model, data)
        self.neutral = data.ten_length[model.actuator_trnid[:, 0]].copy()
        self._gain = model.actuator_gainprm.copy()
        self._bias = model.actuator_biasprm.copy()
        self.enabled = True
        self.command([0, 0, 0])

    def command(self, displacement_m):
        delta = np.asarray(displacement_m, dtype=float)
        if delta.shape != (3,) or not np.isfinite(delta).all():
            raise ValueError('Expected three finite differential displacements in metres')
        applied = delta.copy()
        for i, (plus, minus) in enumerate(self.pairs):
            limits = self.model.actuator_ctrlrange
            low = max(limits[plus, 0]-self.neutral[plus], self.neutral[minus]-limits[minus, 1])
            high = min(limits[plus, 1]-self.neutral[plus], self.neutral[minus]-limits[minus, 0])
            applied[i] = np.clip(applied[i], low, high)
            self.data.ctrl[plus] = self.neutral[plus]+applied[i]
            self.data.ctrl[minus] = self.neutral[minus]-applied[i]
        return applied

    def set_enabled(self, enabled):
        """Disable both servo stiffness and servo damping for unpowered return."""
        self.enabled = bool(enabled)
        self.model.actuator_gainprm[:] = self._gain if enabled else 0
        self.model.actuator_biasprm[:] = self._bias if enabled else 0


def elastic_energy(model, data):
    """Stored hinge-spring energy in joules, excluding viscous dissipation."""
    return float(.5*np.sum(model.jnt_stiffness*(data.qpos-model.qpos_spring)**2))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode', choices=['bend', 'recoil'], default='bend')
    ap.add_argument('--headless', action='store_true', help='Run without an OpenGL viewer')
    ap.add_argument('--duration', type=float, default=4., help='Simulation duration in seconds')
    args = ap.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        ap.error('--duration must be positive and finite')
    model = mujoco.MjModel.from_xml_path(str(HERE/'models/continuum.xml'))
    data = mujoco.MjData(model)
    controller = TendonController(model, data)
    substeps = round(.02/model.opt.timestep)
    max_energy = 0.
    step = 0

    def advance():
        nonlocal step, max_energy
        if step % substeps == 0:
            t = data.time
            if args.mode == 'recoil':
                controller.command([.006*min(t/.6, 1.), 0, 0])
                controller.set_enabled(t < 1.5)
            else:
                envelope = min(t/.4, 1.)
                controller.command(envelope*np.array([.006*np.sin(2*np.pi*.5*t),
                                                      .004*np.sin(2*np.pi*.35*t),
                                                      .004*np.sin(2*np.pi*.25*t)]))
        mujoco.mj_step(model, data)
        step += 1
        max_energy = max(max_energy, elastic_energy(model, data))
        if not np.isfinite(data.qpos).all():
            raise RuntimeError('Non-finite joint state')

    if args.headless:
        while data.time < args.duration-1e-10:
            advance()
    else:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [0, 0, .31]
            viewer.cam.distance = .9
            viewer.cam.azimuth = 125
            viewer.cam.elevation = -15
            while viewer.is_running() and data.time < args.duration-1e-10:
                start = time.perf_counter()
                advance()
                viewer.sync()
                remaining = model.opt.timestep-(time.perf_counter()-start)
                if remaining > 0:
                    time.sleep(remaining)
    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'continuum_tip')
    mujoco.mj_forward(model, data)
    print(json.dumps({'mode': args.mode, 'duration_s': float(data.time),
        'physics_steps': step, 'independent_tendon_drives': 3,
        'peak_elastic_energy_j': max_energy, 'final_elastic_energy_j': elastic_energy(model, data),
        'tip_position_m': data.site_xpos[tip].tolist(), 'actuators_enabled': controller.enabled}, indent=2))


if __name__ == '__main__':
    main()
