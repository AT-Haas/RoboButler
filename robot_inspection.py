# inspect_robot.py
# Purpose: To load the Franka Panda robot and print a detailed, formatted
# report of all its joints and their properties. This is a crucial
# debugging tool to understand the robot's structure.

import pybullet as p
import pybullet_data

# --- 1. Setup the simulation environment (no GUI needed) ---
print("Connecting to PyBullet in DIRECT mode...")
physicsClient = p.connect(p.DIRECT)  # We don't need to see the robot, just inspect it
p.setAdditionalSearchPath(pybullet_data.getDataPath())

print("Loading robot URDF...")
# Load the robot at a standard position
robotId = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
print(f"Robot loaded with ID: {robotId}")

# --- 2. Get and print the joint information ---
num_joints = p.getNumJoints(robotId)
print(f"The robot has a total of {num_joints} joints.")
print("-" * 80)

# Create a mapping from joint type numbers to human-readable names
joint_type_mapping = {
    p.JOINT_REVOLUTE: "REVOLUTE",
    p.JOINT_PRISMATIC: "PRISMATIC",
    p.JOINT_SPHERICAL: "SPHERICAL",
    p.JOINT_PLANAR: "PLANAR",
    p.JOINT_FIXED: "FIXED"
}

# Print a formatted header for our table
print(f"{'Index':<7} | {'Joint Name':<25} | {'Joint Type':<12} | {'Link Name':<20} | {'Limits (L/U)':<15}")
print("-" * 80)

# --- 3. Iterate through all joints and print their properties ---
for i in range(num_joints):
    # Get all information for the current joint
    info = p.getJointInfo(robotId, i)

    # Extract the relevant properties
    joint_index = info[0]
    joint_name = info[1].decode('UTF-8')
    joint_type_num = info[2]
    link_name = info[12].decode('UTF-8')
    lower_limit = round(info[8], 2)
    upper_limit = round(info[9], 2)

    # Convert joint type number to a string
    joint_type_str = joint_type_mapping.get(joint_type_num, f"UNKNOWN({joint_type_num})")

    # Format and print the row for our table
    print(
        f"{joint_index:<7} | {joint_name:<25} | {joint_type_str:<12} | {link_name:<20} | [{lower_limit}, {upper_limit}]")

print("-" * 80)
print("Inspection complete.")
p.disconnect()

"""OUTPUT:
Robot loaded with ID: 0
The robot has a total of 12 joints.
--------------------------------------------------------------------------------
Index   | Joint Name                | Joint Type   | Link Name            | Limits (L/U)   
--------------------------------------------------------------------------------
0       | panda_joint1              | REVOLUTE     | panda_link1          | [-2.97, 2.97]
1       | panda_joint2              | REVOLUTE     | panda_link2          | [-1.83, 1.83]
2       | panda_joint3              | REVOLUTE     | panda_link3          | [-2.97, 2.97]
3       | panda_joint4              | REVOLUTE     | panda_link4          | [-3.14, 0.0]
4       | panda_joint5              | REVOLUTE     | panda_link5          | [-2.97, 2.97]
5       | panda_joint6              | REVOLUTE     | panda_link6          | [-0.09, 3.82]
6       | panda_joint7              | REVOLUTE     | panda_link7          | [-2.97, 2.97]
7       | panda_joint8              | FIXED        | panda_link8          | [0.0, -1.0]
8       | panda_hand_joint          | FIXED        | panda_hand           | [0.0, -1.0]
9       | panda_finger_joint1       | PRISMATIC    | panda_leftfinger     | [0.0, 0.04]
10      | panda_finger_joint2       | PRISMATIC    | panda_rightfinger    | [0.0, 0.04]
11      | panda_grasptarget_hand    | FIXED        | panda_grasptarget    | [0.0, -1.0]
--------------------------------------------------------------------------------
Inspection complete.
"""