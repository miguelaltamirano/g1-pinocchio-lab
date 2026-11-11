# G1 Lab — Left-arm IK with Pinocchio

An implementation-first student website for Unitree G1 left-arm inverse kinematics. Students can explore a reach in the browser, follow six short code steps, and download the matching Python solver.

The website is static HTML, CSS and JavaScript with a locally bundled Three.js renderer. There is no build step, backend, API key, or student sign-in.

**Student website:** https://miguelaltamirano.github.io/g1-pinocchio-lab/

**Deployment status:** [Publish student website](https://github.com/miguelaltamirano/g1-pinocchio-lab/actions/workflows/pages.yml)

## What students learn

1. Load the G1 URDF with `pin.buildModelFromUrdf`.
2. Lock non-arm joints with `pin.buildReducedModel`.
3. Add the tool frame with `model.addFrame`.
4. Calculate the current position with `pin.framesForwardKinematics`.
5. Obtain a frame-consistent Jacobian with `pin.computeFrameJacobian`.
6. Apply damped least-squares updates using `pin.integrate`.

The browser preview implements the same position-only DLS update in JavaScript; it does not run native Pinocchio. The downloadable Python example uses actual Pinocchio. The seven left-arm joints move while the base and other joints stay fixed. Collision checking and dynamic balance are outside this exercise.

## 3D robot viewer

The MuJoCo-style viewport displays all 36 official G1 visual meshes, with lighting, shadows and a checkerboard floor. Drag to orbit, scroll or pinch to zoom, or use the zoom buttons. Focus arm brings the reach closer; Reset view restores the full robot. Joint frames shows coordinate axes, and Hand path shows the accepted IK updates. The canvas also supports left/right arrow keys and +/- for keyboard camera control.

This is a Three.js kinematic viewer, not a running MuJoCo simulation. The full lab download includes the native MuJoCo viewer. No software installation is needed to use the website, but the browser needs WebGL 2. If 3D rendering is unavailable, the IK controls, lessons and downloads remain usable.

The mesh payload is about 6.9 MB before HTTP compression, loaded once and cached by the browser. The renderer only draws when the pose or camera changes. Three.js 0.180.0 and OrbitControls are served from this repository rather than a third-party CDN.

## Publish with GitHub Pages

1. Create a **public** repository called `g1-pinocchio-lab` in your personal GitHub account, with `main` as its default branch.
2. Push or upload the contents of this folder, including `.github/workflows/pages.yml`. Keep `dist/`, `example/`, and `qa/` at the repository root; do not add an extra enclosing directory.
3. In the repository, open **Settings → Pages → Build and deployment → Source** and choose **GitHub Actions**.
4. Open **Actions → Publish student website → Run workflow**. If the first automatic run happened before Pages was enabled, rerun it.
5. After deployment succeeds, **Settings → Pages** and the workflow's `github-pages` environment show the actual student URL.

For this repository, the student URL is:

```text
https://miguelaltamirano.github.io/g1-pinocchio-lab/
```

Relative asset and download paths support the repository subpath. Future pushes to `main` automatically check and deploy `dist/`.

GitHub Pages supports public repositories on GitHub Free. Official documentation: [custom Pages workflows](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages).

## Preview locally

From the repository root:

```bash
python3 -m http.server 8000 --directory dist
```

On Windows, use `python` instead of `python3`. Open `http://localhost:8000`. Serving over HTTP is needed for JavaScript modules and the full-code fetch; opening `index.html` directly as a file is not equivalent.

## Check the browser solver

With a current Node.js installation:

```bash
node qa/check.mjs
node qa/check-viewer.mjs
```

This compares browser FK, Jacobians and DLS results to reference outputs generated using Pinocchio. It checks convergence, the supplied higher target, an unreachable target, and joint bounds. The default target converges in nine accepted updates to about 0.0687 mm position error. This is one example, not a universal performance ranking.

The viewer check additionally verifies the rendered robot's tool position, joint origins and joint axes against the solver at 54 poses, and checks all mesh indices. To regenerate the packed mesh assets, an instructor with NumPy can run `python scripts/build_robot_assets.py /path/to/models`, where the source directory contains the pinned `g1_original.urdf` and `meshes/` from the full lab. Students do not need this step.

## Run the native Pinocchio example

The small Python example is in `example/`. It needs only Pinocchio and NumPy:

```bash
cd example
conda env create -f environment.yml
conda activate g1-left-arm
python left_arm_ik.py
```

Use [Miniforge](https://conda-forge.org/download/) if Conda is not already installed. The code has been tested on Linux. The Conda recipe targets mixed operating systems, but Windows/macOS still need a pilot installation. The pinned `pin==3.9.0` pip distribution is Linux-only; use the supplied Conda environment for the classroom route.

The site offers both the minimal Python ZIP and the complete MuJoCo/notebook teaching bundle. MuJoCo and Jupyter are optional extensions, not requirements for using the website.

## Repository layout

```text
dist/                     GitHub Pages website, assets and student downloads
example/                  Minimal native Pinocchio solver and original URDF
qa/check.mjs              Numerical browser-versus-Pinocchio checks
qa/reference.json         Reference results generated with Pinocchio
.github/workflows/        Automatic GitHub Pages deployment
```

## Model attribution

The G1 geometry comes from [Unitree's official URDF at the pinned revision](https://github.com/unitreerobotics/unitree_ros/blob/9926cc2f179ae3b86f4f74087bd32ef0c8b6fd90/robots/g1_description/g1_29dof.urdf). The 8 cm wrist-to-tool offset is a classroom choice, not a calibrated fingertip.

Unitree model assets retain their BSD 3-Clause license, included in `example/models/UNITREE_LICENSE.txt` and the download archives. This repository copy contains no Sites account identity, hosting credentials, or `.openai` configuration.

The same Unitree license covers the packed browser meshes; see `dist/assets/UNITREE_LICENSE.txt`. Third-party rendering code retains the MIT license at `dist/vendor/THREE_LICENSE.txt`.
