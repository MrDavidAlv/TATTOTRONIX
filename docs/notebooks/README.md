# Notebooks

The robotics behind TATTOTRONIX, worked through step by step and runnable in the
browser. Every formula is implemented where you can read it and then checked
against the model the repository actually uses — the `assert` lines run on every
build, so a notebook that disagreed with the robot could not have been published.

| Notebook | What it covers | |
|---|---|---|
| **[Kinematics](01_kinematics.ipynb)** | Rotation matrices and Rodrigues' formula, the URDF's roll–pitch–yaw, homogeneous transforms, forward kinematics derived symbolically, the product of exponentials, the space, geometric and task Jacobians, singular values and manipulability ellipsoids, the workspace, damped least-squares inverse kinematics, and the arm drawing the ROS logo in 2D and 3D | [![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/1Lmc4HzBdJf-z6v3CmRNt6g5lY85Nq4Dd?usp=sharing) |
| **[Dynamics](02_dynamics.ipynb)** | Inertia tensors and the spatial inertia matrix, the equations of motion, the mass matrix rebuilt from kinetic energy, Coriolis terms from Christoffel symbols and the skew-symmetry of Ṁ − 2C, gravity from potential energy, energy conservation with the arm falling freely, and the torque each servo has to supply while drawing | [![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/16LlfGKtVIHmqNoxojnTrFz47NQBHQEkX?usp=sharing) |

<div align="center">
<img src="../figures/21_arm_3d.gif" width="70%"/>
<br/>
<sub>From the kinematics notebook: the arm's real meshes, simplified for animation,
replaying the trajectory the simulation runs.</sub>
</div>

---

## Running them

**In Colab**, press the badge. The first cell clones this repository — it carries
the robot's description, meshes and model code — and everything else comes with
Colab: NumPy, SciPy, Matplotlib, SymPy and Plotly. No ROS is needed; the notebooks
read [`tattotronix.urdf`](tattotronix.urdf), the description already expanded
from its xacro.

**Locally**, from a checkout:

```bash
pip install -r docs/scripts/requirements.txt -r docs/notebooks/requirements.txt jupyter
jupyter lab docs/notebooks/
```

---

## How they are made, and why they can be trusted

- **Sources, not notebooks, are edited.** Each notebook is written as a plain
  Python file under [`src/`](src/), in the "percent" format, and
  [`build.py`](build.py) turns it into the `.ipynb`. Sources diff and review like
  code; the notebooks are generated.
- **The model is the repository's, not a copy.** The chain and the dynamic model
  are built by `docs/scripts/kinematics.py` and `dynamics.py`, the same code every
  study uses. What a notebook adds is the mathematics written out, and a check
  that the two agree.
- **They are run on every build.** `tests/test_notebooks.py` rebuilds each
  notebook and compares, then executes its code cells — exactly what a reader
  opens — headless. It also checks that `tattotronix.urdf` is still what the xacro
  expands to.
- **The documentation's concept figures come from here.** With `TATTO_EXPORT=1`,
  the figures a notebook names are written to [`docs/figures`](../figures), so the
  pictures in the documents are produced by the notebook code itself:

```bash
python3 docs/notebooks/build.py                          # rebuild from the sources
python3 docs/notebooks/build.py --export 01_kinematics 02_dynamics   # run and write figures
```

The badges open copies of the notebooks shared from Google Drive: anyone with the
link can open and run them, and *File → Save a copy in Drive* keeps your changes.
Their first cell clones the `humble` branch, the one this repository publishes, so
the model a copy runs is always the current one. The notebook text is a snapshot;
if a copy and the notebook here ever differ, the one here is the reference.
