export const LESSONS = [
{title:'Load the robot',label:'Read the URDF',function:'buildModelFromUrdf',heading:'Start with a model, not a simulator.',description:'Pinocchio reads the joint names, transforms and limits from the G1 URDF. For this exercise, the pelvis is fixed. You do not need MuJoCo or mesh files to calculate joint configurations.',code:`from pathlib import Path
import numpy as np
import pinocchio as pin

urdf = Path("models/g1_29dof.urdf")
full_model = pin.buildModelFromUrdf(str(urdf))`,notes:[['What you get','A Model describes the robot: its joints, frames and limits.'],['Why the base stays fixed','We do not pass a free-flyer joint to the model builder.']],check:'The original model has 29 actuated joints. Next, keep only the seven we need.'},
{title:'Keep the left arm',label:'Reduce the model',function:'buildReducedModel',heading:'Seven active joints. Everything else stays still.',description:'Lock the legs, waist and right arm at the neutral configuration. Use joint names to select the arm; never assume two libraries store their joints in the same order.',code:`ARM_JOINTS = [f"left_{name}_joint" for name in (
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
    "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"
)]

locked_ids = [j for j in range(1, full_model.njoints)
              if full_model.names[j] not in ARM_JOINTS]
model = pin.buildReducedModel(
    full_model, locked_ids, pin.neutral(full_model)
)`,notes:[['Lock at a known posture','pin.neutral(full_model) is the configuration used for the locked joints.'],['Map coordinates by name','The full script stores idx_q and idx_v separately for each arm joint.']],check:'For this reduced arm, model.nq and model.nv are both 7.'},
{title:'Define the tool',label:'Choose the reaching point',function:'addFrame',heading:'Tell Pinocchio which point should reach.',description:'A wrist joint is not the same as the point you want to place. Add a tool frame 8 cm along the final wrist frame’s local x axis. This is a teaching offset, not a calibrated fingertip.',code:`wrist = model.frames[
    model.getFrameId("left_wrist_yaw_link")
]
offset = pin.SE3(np.eye(3), np.array([0.08, 0, 0]))

tool_id = model.addFrame(pin.Frame(
    "left_tool", wrist.parentJoint,
    wrist.placement * offset, pin.FrameType.OP_FRAME
))
data = model.createData()`,notes:[['Compose the transforms','wrist.placement locates the wrist frame relative to its parent joint. Multiply by the local tool offset.'],['Create Data afterwards','Data holds calculated results. Create it after adding the frame so its storage matches Model.']],check:'tool_id is the frame index used for both the position and the Jacobian.'},
{title:'Measure the error',label:'Run forward kinematics',function:'framesForwardKinematics',heading:'Where is the tool right now?',description:'Run forward kinematics at the current q. The requested target uses torso_link coordinates, so express the current tool position in that same frame before subtracting.',code:`pin.framesForwardKinematics(model, data, q)

torso_id = model.getFrameId("torso_link")
world_T_torso = data.oMf[torso_id].copy()
torso_T_tool = world_T_torso.actInv(
    data.oMf[tool_id]
)
position = torso_T_tool.translation
error = target - position`,notes:[['One coordinate frame','target and position are both three-element vectors in torso_link, in metres.'],['The torso is fixed here','The full script caches world_T_torso once. A moving torso would need a relative-motion formulation.']],check:'Stop successfully when np.linalg.norm(error) < 1e-4: a 0.1 mm position tolerance.'},
{title:'Ask for the Jacobian',label:'Get the linear rows',function:'computeFrameJacobian',heading:'Use the Jacobian of the tool frame.',description:'LOCAL_WORLD_ALIGNED gives the Jacobian at the tool origin, with world-aligned axes. Take the linear rows and rotate them into torso axes. Select velocity columns by their named joint indices.',code:`J6 = pin.computeFrameJacobian(
    model, data, q, tool_id,
    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
)

J = world_T_torso.rotation.T @ J6[:3, v_indices]`,notes:[['Which rows?','Pinocchio orders motion as linear first, angular second. Position-only IK needs the first three rows.'],['Which columns?','v_indices maps the seven arm joints into Pinocchio’s velocity coordinates; it is not assumed to equal idx_q.']],check:'J is 3 × 7. Its rows and the error vector now describe the same axes.'},
{title:'Update and repeat',label:'Solve the position task',function:'integrate',heading:'One small update, then measure again.',description:'Compute a damped least-squares step, put it into Pinocchio’s velocity coordinates, and integrate it. Repeat until the tool reaches the target or the search stops making progress.',code:`dq_arm = J.T @ np.linalg.solve(
    J @ J.T + damping**2 * np.eye(3), error
)
dq_arm *= min(1.0, 0.12 / max(np.linalg.norm(dq_arm), 1e-15))

velocity = np.zeros(model.nv)
velocity[v_indices] = dq_arm
candidate = pin.integrate(model, q, scale * velocity)
candidate = np.clip(candidate,
    model.lowerPositionLimit, model.upperPositionLimit)`,notes:[['Accept an improvement','The full loop backtracks scale until the position error decreases. Clipping is a simple safeguard, not a constrained QP.'],['Report the actual outcome','Return converged, stalled or iteration_limit together with the residual. A local failure does not prove a target is unreachable.']],check:'The supplied target converges in 9 updates with damping = 0.03. Now run the Python file.'}
];
