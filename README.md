# Tendon-driven continuum model for MuJoCo

A standalone two-segment continuum component represented by rigid links,
compliant hinges and spatial tendons in MuJoCo. The model provides three
independent differential tendon inputs: one for the proximal segment and two
for the distal segment. Six virtual tendon-position actuators implement the
three opposing pairs; they do **not** represent six independent physical motors.

The geometry, local hinge axes, collision shapes, tendon routing and virtual
position actuators are extracted from the continuum component of the associated
rigid–continuum throwing robot. The released XML includes the identified
segment-wise hinge stiffness and damping.

## Quick start

Use Python 3.10–3.12. This release was validated with Python 3.12.4,
MuJoCo 3.2.0 and NumPy 1.26.4.
Install MuJoCo as a dependency; no separate MuJoCo installation, RL framework,
robot driver, external mesh or experimental dataset is needed.

```bash
git clone https://github.com/Prologue-Z/mujoco-tendon-continuum.git
cd mujoco-tendon-continuum
python -m pip install -r requirements.txt

# Interactive bending example
python demo.py --mode bend --duration 20

# Bend, hold, and then disable tendon actuation to observe passive return
python demo.py --mode recoil --duration 4

# Run without a graphics display
python demo.py --mode recoil --headless --duration 4

# Structural and physical checks
python -m unittest -v test_model.py
```

On macOS, use `mjpython demo.py ...` for the interactive viewer; ordinary
`python` works for headless examples and tests. The interactive viewer requires
an OpenGL-capable display.

The XML can also be opened directly in a MuJoCo viewer. Before simulating with
tendon-position actuators, initialize their references to the straight-pose
tendon lengths; `TendonController` in `demo.py` performs this initialization.
The default zero control vector is not a neutral tendon-length reference.

## Model

| Property | Proximal segment | Distal segment |
|---|---:|---:|
| Rigid links and compliant hinges | 10 | 10 |
| Link length | 20.9 mm | 24.2 mm |
| Nominal link-length sum | 209 mm | 242 mm |
| Body diameter | 56 mm | 60 mm |
| Mass per link | 0.05 kg | 0.05 kg |
| Hinge axes | Y | Alternating X and Y |
| Local hinge limits | ±0.157 rad | ±0.314 rad |
| Opposing tendon pairs | 1 | 2 |
| Axis-wise routing offset | 33 mm | 24.7 mm |
| Hinge stiffness | 82.04 N m/rad | 58.10 N m/rad |
| Hinge damping | 10.99 N m s/rad | 19.00 N m s/rad |

Values shown above are rounded for readability; the XML and `parameters.json`
retain the full identified parameter values. Stiffness and damping apply to
each discretized hinge and are shared within each segment. Tendon stiffness
and damping are zero: recoverable elasticity is provided by hinge springs.
The local hinge limits are not limits on the total segment bend.

The nominal link lengths sum to 451 mm. The source model also has a 3.3 mm
serial-interface positioning offset, retained here, so the straight-pose
distance between `continuum_base_site` and `continuum_tip` is 454.3 mm.
The distal routing sites are offset by ±24.7 mm in both cross-sectional axes,
giving a radial offset of approximately 34.93 mm.

The physics timestep is 0.002 s with implicit integration. Examples update
tendon references every 0.02 s (50 Hz). The copied virtual position-actuator
gains are 5000 N/m and 100 N s/m, with a ±500 N virtual force range. These are
simulation actuator settings, not a measured hardware motor specification.

## Three-coordinate actuation

`TendonController.command([d1, d2x, d2y])` takes differential tendon
displacements in **metres**, corresponding to Δℓ₁, Δℓ₂,ₓ and Δℓ₂,ᵧ. Each is
half the change in the corresponding opposing tendon-length difference.
The last two coordinates drive the **same distal segment**; their x/y labels
identify local tendon-drive channels.

For each pair, the references are `L_plus = L0_plus + d` and
`L_minus = L0_minus - d`. The controller clips the shared displacement to the
intersection of both actuator ranges, preserving equal-and-opposite changes.
The 20 individual hinge angles are dynamic states rather than independently
commanded inputs.

The recoil example disables both the position-servo gain and its servo-damping
bias after 1.5 s. Hinge stiffness, hinge damping and gravity remain active.
Thus the subsequent return is unpowered; merely commanding the neutral tendon
lengths would instead produce an actively driven return.

## Scope and reuse

This release contains the **continuum component**, with its base fixed in a
simple scene. It excludes the mobile base, rigid arm, wrist, ball holder, ball,
SAC training code and experimental records. The example illustrates component
actuation and spring return; it is not a reproduction of complete throwing
results or their peak-power comparison.

To integrate the model into another robot, attach the `continuum_base` body to
the desired parent and retain its tendons and actuators. `continuum_tip` is the
distal tube interface. Loads attached there change the component dynamics.
Treat this as a discrete compliant-link approximation with the stated geometry
and parameters, rather than a general continuum constitutive model.

## Files

- `models/continuum.xml`: self-contained MJCF model without mesh assets.
- `parameters.json`: dimensions, identified parameters, units and source hashes.
- `demo.py`: three-coordinate control and bending/passive-return examples.
- `test_model.py`: loading, geometry, parameter, actuation and return checks.
- `requirements.txt`: tested dependency versions.

## Associated work and citation

Associated manuscript: *Learning to Exploit Elastic Energy for Peak Drive
Power Reduction in Robotic Throwing with Soft Actor-Critic*.
Publication details will be added when available. For now, cite the software
repository and version `v1.0.0`; `CITATION.cff` supplies the software metadata.

## License

The model and example code are released under the MIT license. See `LICENSE`.
MuJoCo and other installed dependencies retain their own licenses.
