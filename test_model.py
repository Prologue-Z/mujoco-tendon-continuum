"""Structural and physical checks for the released continuum component."""
from pathlib import Path
import json
import subprocess
import sys
import unittest

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent


class ContinuumChecks(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(HERE / 'models/continuum.xml'))
        self.data = mujoco.MjData(self.model)
        self.parameters = json.loads((HERE / 'parameters.json').read_text())

    def test_standalone_structure_and_geometry(self):
        m, d = self.model, self.data
        self.assertEqual((m.nq, m.nv, m.ntendon, m.nu), (20, 20, 6, 6))
        self.assertEqual(m.nmesh, 0)
        self.assertAlmostEqual(float(m.body_mass.sum()), 1.0)
        mujoco.mj_forward(m, d)
        base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'continuum_base_site')
        tip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'continuum_tip')
        self.assertAlmostEqual(float(np.linalg.norm(d.site_xpos[tip]-d.site_xpos[base])),
                               self.parameters['straight_base_to_tip_span_m'])
        self.assertAlmostEqual(sum(m.geom_size[mujoco.mj_name2id(m,
                mujoco.mjtObj.mjOBJ_GEOM, f'g{s}_{i}_vis'), 1]*2
                for s in (1, 2) for i in range(1, 11)), .451)
        for segment in (1, 2):
            p = self.parameters['segments'][str(segment)]
            for i in range(1, 11):
                j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'j{segment}_{i}')
                geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f'g{segment}_{i}_vis')
                self.assertAlmostEqual(m.jnt_stiffness[j], p['hinge_stiffness_nm_per_rad'])
                self.assertAlmostEqual(m.dof_damping[m.jnt_dofadr[j]], p['hinge_damping_nms_per_rad'])
                self.assertAlmostEqual(m.geom_size[geom, 0]*2, p['body_diameter_m'])
        self.assertTrue(np.all(m.tendon_stiffness == 0))
        self.assertTrue(np.all(m.tendon_damping == 0))

    def test_three_commands_generate_opposing_references(self):
        from demo import TendonController
        c = TendonController(self.model, self.data)
        request = np.array([.003, -.002, .001])
        applied = c.command(request)
        np.testing.assert_allclose(applied, request)
        for i, (plus, minus) in enumerate(c.pairs):
            self.assertAlmostEqual(self.data.ctrl[plus]-c.neutral[plus], request[i])
            self.assertAlmostEqual(self.data.ctrl[minus]-c.neutral[minus], -request[i])
        c.command([10, -10, 10])
        self.assertTrue(np.all(self.data.ctrl >= self.model.actuator_ctrlrange[:, 0]))
        self.assertTrue(np.all(self.data.ctrl <= self.model.actuator_ctrlrange[:, 1]))
        for pair in c.pairs:
            self.assertAlmostEqual(sum(self.data.ctrl[list(pair)]), sum(c.neutral[list(pair)]))
        for bad in ([1, 2], [0, float('nan'), 0]):
            with self.assertRaises(ValueError):
                c.command(bad)

    def test_each_input_causes_finite_bending(self):
        from demo import TendonController
        for drive in range(3):
            m = self.model
            d = mujoco.MjData(m)
            c = TendonController(m, d)
            q = [0., 0., 0.]
            q[drive] = .004
            c.command(q)
            for _ in range(750):
                mujoco.mj_step(m, d)
            self.assertTrue(np.isfinite(d.qpos).all())
            self.assertGreater(float(np.linalg.norm(d.qpos)), 1e-4)
            self.assertLess(float(np.max(np.abs(d.qpos))), .32)

    def test_unpowered_spring_return(self):
        from demo import TendonController, elastic_energy
        m, d = self.model, self.data
        c = TendonController(m, d)
        c.command([.006, 0, 0])
        for _ in range(750):
            mujoco.mj_step(m, d)
        stored = elastic_energy(m, d)
        self.assertGreater(stored, .001)
        c.set_enabled(False)
        mujoco.mj_forward(m, d)
        np.testing.assert_allclose(d.qfrc_actuator, 0, atol=1e-12)
        for _ in range(1500):
            mujoco.mj_step(m, d)
        self.assertLess(elastic_energy(m, d), .2*stored)
        self.assertTrue(np.isfinite(d.qpos).all())
        c.set_enabled(True)
        self.assertTrue(np.all(m.actuator_gainprm[:, 0] > 0))

    def test_headless_quickstart_commands(self):
        for mode in ('bend', 'recoil'):
            result = subprocess.run([sys.executable, '-B', str(HERE/'demo.py'), '--mode', mode,
                                     '--headless', '--duration', '2'],
                                    capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(result.stdout)
            self.assertAlmostEqual(report['duration_s'], 2.)
            self.assertEqual(report['actuators_enabled'], mode == 'bend')


if __name__ == '__main__':
    unittest.main(verbosity=2)
