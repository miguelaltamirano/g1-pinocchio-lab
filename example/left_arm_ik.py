"""Position IK for the Unitree G1 left arm using Pinocchio + NumPy only.

Run: python left_arm_ik.py
Custom target in torso_link [m]: python left_arm_ik.py --target 0.30 0.25 0.10
The base, waist, legs and right arm are fixed. No dynamics or hardware commands.
"""
from pathlib import Path
import argparse
import numpy as np
import pinocchio as pin

# 01 — Load the original G1 model. No free-flyer joint: the pelvis is fixed.
URDF = Path(__file__).resolve().parent / 'models' / 'g1_29dof.urdf'
full_model = pin.buildModelFromUrdf(str(URDF))
ARM_JOINTS = [f'left_{name}_joint' for name in (
    'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
    'wrist_roll', 'wrist_pitch', 'wrist_yaw')]

# 02 — Lock every joint except the left arm at the neutral configuration.
locked_ids = [j for j in range(1, full_model.njoints)
              if full_model.names[j] not in ARM_JOINTS]
model = pin.buildReducedModel(full_model, locked_ids, pin.neutral(full_model))
q_indices = np.array([model.joints[model.getJointId(n)].idx_q for n in ARM_JOINTS])
v_indices = np.array([model.joints[model.getJointId(n)].idx_v for n in ARM_JOINTS])

# 03 — Define the point to reach: 8 cm along the final wrist frame's local x.
# This is a classroom tool offset, not a calibrated palm/fingertip.
wrist = model.frames[model.getFrameId('left_wrist_yaw_link')]
wrist_to_tool = pin.SE3(np.eye(3), np.array([0.08, 0.0, 0.0]))
tool_id = model.addFrame(pin.Frame(
    'left_tool', wrist.parentJoint, wrist.placement * wrist_to_tool,
    pin.FrameType.OP_FRAME))
data = model.createData()  # create after adding the tool frame
q_initial = pin.neutral(model)
q_initial[q_indices] = [0.0, 0.20, 0.0, 0.60, 0.0, 0.0, 0.0]
pin.framesForwardKinematics(model, data, q_initial)
torso_id = model.getFrameId('torso_link')
world_T_torso = data.oMf[torso_id].copy()  # torso remains fixed in this exercise


def tool_position(q):
    # 04 — Compute the current tool position, expressed in the torso frame.
    pin.framesForwardKinematics(model, data, q)
    torso_T_tool = world_T_torso.actInv(data.oMf[tool_id])
    return torso_T_tool.translation.copy()


def position_jacobian(q):
    # 05 — Tool-origin Jacobian, first in world axes, then in torso axes.
    J6 = pin.computeFrameJacobian(
        model, data, q, tool_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    # Pinocchio's first 3 rows are linear; last 3 are angular.
    return world_T_torso.rotation.T @ J6[:3, v_indices]


def solve_ik(target, damping=0.03, max_iterations=400):
    # 06 — Repeat DLS updates, with a step cap, joint bounds and backtracking.
    target = np.asarray(target, dtype=float)
    if target.shape != (3,) or not np.isfinite(target).all():
        raise ValueError('Target must contain three finite coordinates in metres')
    if not np.isfinite(damping) or damping <= 0:
        raise ValueError('Damping must be finite and positive')
    q = q_initial.copy()
    status = 'iteration_limit'
    history = []
    for iteration in range(max_iterations + 1):
        error = target - tool_position(q)
        residual = np.linalg.norm(error)
        history.append(residual)
        if residual < 1e-4:  # 0.1 mm position tolerance
            status = 'converged'
            break
        if iteration == max_iterations:
            break
        J = position_jacobian(q)
        dq_arm = J.T @ np.linalg.solve(J @ J.T + damping**2*np.eye(3), error)
        dq_arm *= min(1.0, 0.12 / max(np.linalg.norm(dq_arm), 1e-15))
        velocity = np.zeros(model.nv)
        velocity[v_indices] = dq_arm
        for scale in 0.5 ** np.arange(15):
            candidate = pin.integrate(model, q, scale * velocity)
            candidate = np.clip(candidate, model.lowerPositionLimit, model.upperPositionLimit)
            if np.linalg.norm(target - tool_position(candidate)) < residual - 1e-13:
                q = candidate
                break
        else:
            status = 'stalled'
            break
    # q is in Pinocchio's order; map by name when displaying/exporting values.
    return dict(q=q, status=status, iterations=len(history)-1,
                position_error_m=history[-1], history=np.array(history))


def default_target():
    q_reference = pin.neutral(model)
    q_reference[q_indices] = [-0.60, 0.45, 0.25, 1.20, 0.20, -0.30, 0.15]
    return tool_position(q_reference)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
    args = parser.parse_args()
    target = default_target() if args.target is None else np.array(args.target)
    result = solve_ik(target)
    print('Target in torso_link [m]:', target)
    print(f"{result['status']}: {result['iterations']} updates; "
          f"position error {result['position_error_m']*1000:.4f} mm")
    for name, index in zip(ARM_JOINTS, q_indices):
        print(f'{name:30s} {result["q"][index]: .6f} rad')
    # Success certifies only this position task, not collision freedom or balance.
