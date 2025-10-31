# grasp_sequence_refactored.py
# Purpose: A refactored and clean version of the Phase 0 script.
# Recording is now controlled by a simple boolean flag and handled by a dedicated function.

import pybullet as p
import pybullet_data
import time
import math
import os
import imageio
import numpy as np

# --- 0. Configuration ---
RECORD_GIF = True

output_dir = "out/phase0"
os.makedirs(output_dir, exist_ok=True)
frames = []
capture_every_n_steps = 10  # Capture a frame every 8 steps for a 30fps GIF (240Hz / 8 = 30fps)


def capture_frame():
    """
    If recording is enabled, captures a single frame from the simulation
    and appends it to the global 'frames' list.
    """
    if not RECORD_GIF:
        return  # Do nothing if recording is disabled

    camera_info = p.getDebugVisualizerCamera()
    width, height, view_matrix, proj_matrix = camera_info[0], camera_info[1], camera_info[2], camera_info[3]
    width = 480
    height = 360
    img = p.getCameraImage(width, height, view_matrix, proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    # Convert RGBA to RGB and ensure the correct data type for image saving
    rgb_image = np.array(img[2], dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
    frames.append(rgb_image)


# --- 1. Setup the simulation environment ---
print("Connecting to PyBullet...")
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0.5])

print("Loading scene...")
# Load assets
p.loadURDF("plane.urdf")
p.loadURDF("table/table.urdf", useFixedBase=True)
cubeId = p.loadURDF("cube_small.urdf", basePosition=[0.4, 0.1, 0.65])
robotId = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0.62], useFixedBase=True)

# --- 2. Prepare for control ---
print("Setting initial robot pose...")
initial_joint_positions = [0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.7854]
num_arm_joints = 7
for i in range(num_arm_joints):
    p.resetJointState(robotId, i, initial_joint_positions[i])

end_effector_link_index = 6
finger_joint_indices = []
for i in range(p.getNumJoints(robotId)):
    info = p.getJointInfo(robotId, i)
    if info[1].decode('UTF-8') in ['panda_finger_joint1', 'panda_finger_joint2']:
        finger_joint_indices.append(i)

lower_limits = [p.getJointInfo(robotId, i)[8] for i in range(num_arm_joints)]
upper_limits = [p.getJointInfo(robotId, i)[9] for i in range(num_arm_joints)]
joint_ranges = [u - l for l, u in zip(lower_limits, upper_limits)]
for i in range(num_arm_joints):
    p.changeDynamics(robotId, i, linearDamping=0, angularDamping=0.1)

print(f"Using End effector link index: {end_effector_link_index}")
print(f"Found Finger joint indices: {finger_joint_indices}")

# --- 3. Define control parameters ---
GRIPPER_OPEN_POSITION = 0.04
GRIPPER_CLOSED_POSITION = 0.0


# --- 4. Define a FINAL, SUPERIOR move_arm_to function using CONTEXTUAL IK ---
def move_arm_to(target_pos, target_orn_q, duration=1, tolerance=0.005):
    current_joint_poses = [p.getJointState(robotId, i)[0] for i in range(num_arm_joints)]
    target_joint_poses = p.calculateInverseKinematics(
        robotId, end_effector_link_index, target_pos, target_orn_q,
        lowerLimits=lower_limits, upperLimits=upper_limits, jointRanges=joint_ranges,
        restPoses=current_joint_poses, maxNumIterations=100)

    kp = 0.02
    kd = 0.5
    num_steps = int(duration * 240)

    for step in range(num_steps):
        p.setJointMotorControlArray(
            robotId, range(num_arm_joints), p.POSITION_CONTROL,
            targetPositions=target_joint_poses[:num_arm_joints],
            positionGains=[kp] * num_arm_joints, velocityGains=[kd] * num_arm_joints,
            forces=[500] * num_arm_joints)

        p.stepSimulation()
        time.sleep(1. / 240.)

        if step % capture_every_n_steps == 0:
            capture_frame()

        current_pose = p.getLinkState(robotId, end_effector_link_index)
        distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(current_pose[0], target_pos)]))
        if distance < tolerance:
            break

# --- 5. Define the set_gripper function ---
def set_gripper(position):
    p.setJointMotorControl2(robotId, finger_joint_indices[0], p.POSITION_CONTROL, targetPosition=position, force=30)
    p.setJointMotorControl2(robotId, finger_joint_indices[1], p.POSITION_CONTROL, targetPosition=position, force=30)
    for step in range(120):
        p.stepSimulation()
        time.sleep(1. / 240.)

        if step % capture_every_n_steps == 0:
            capture_frame()


# --- 6. Execute the grasp sequence ---
down_orientation = p.getQuaternionFromEuler([math.pi, 0, math.pi / 4])
print("Opening gripper...")
set_gripper(GRIPPER_OPEN_POSITION)
print("Moving to pre-grasp position...")
move_arm_to([0.4, 0.1, 1.0], down_orientation)
print("Moving to grasp position...")
move_arm_to([0.4, 0.1, 0.86], down_orientation)
print("Closing gripper...")
set_gripper(GRIPPER_CLOSED_POSITION)
print("Lifting the cube...")
move_arm_to([0.4, 0.1, 1.5], down_orientation)
print("Grasp sequence finished. Holding for a few seconds before saving.")

for step in range(60):
    p.stepSimulation()
    time.sleep(1. / 240.)

    if step % capture_every_n_steps == 0:
        capture_frame()

# --- Save the GIF ---
if RECORD_GIF:
    print("Saving GIF...")
    gif_path = os.path.join(output_dir, "phase0_grasp_sequence.gif")
    imageio.mimsave(gif_path, frames, fps=30)
    print(f"GIF saved to: {gif_path}")

print("Simulation finished.")
p.disconnect()