G1 LEFT-ARM IK — MINIMAL PINOCCHIO EXAMPLE

Install Miniforge (or use an existing Conda installation):
https://conda-forge.org/download/

From this extracted directory:
    conda env create -f environment.yml
    conda activate g1-left-arm
    python left_arm_ik.py

The script uses only Pinocchio and NumPy. MuJoCo, meshes and Jupyter are not
needed for computing joint configurations. The URDF references mesh files,
but buildModelFromUrdf loads kinematics without building visual geometry.

Targets use torso_link coordinates, in metres:
    python left_arm_ik.py --target 0.30 0.25 0.10

The script solves only tool position, for the left arm, with every other joint
locked at zero/neutral. The tool is a teaching point 8 cm along the wrist local
x axis. It does not check collisions, dynamics, balance or send robot commands.

The Python implementation is tested with Pinocchio 3.9.0 / Python 3.10 on Linux.
The Conda recipe is the recommended cross-platform distribution route but has
not been pilot-tested on Windows or macOS in this session. The pinned pip
release of Pinocchio 3.9.0 is Linux-only; use Conda for a mixed-OS class.

Model source:
https://github.com/unitreerobotics/unitree_ros/blob/9926cc2f179ae3b86f4f74087bd32ef0c8b6fd90/robots/g1_description/g1_29dof.urdf
Upstream license: models/UNITREE_LICENSE.txt.
