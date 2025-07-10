import pybullet as p
import pybullet_data
import time
import math
from robot_descriptions.loaders.pybullet import load_robot_description

# Start PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load ground plane
p.loadURDF("plane.urdf")

# Load Valkyrie
robot_id = load_robot_description("valkyrie_description")

# Find right arm joints
right_arm_joints = []
num_joints = p.getNumJoints(robot_id)

for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode('utf-8')
    
    # Look for right arm joints (shoulder, elbow, wrist)
    if any(keyword in joint_name.lower() for keyword in ['rightarm', 'right_arm', 'rightshoulder', 'rightelbow', 'rightwrist']):
        right_arm_joints.append(i)
        print(f"Found right arm joint: {joint_name} (ID: {i})")

# If no specific right arm joints found, use general arm joints
if not right_arm_joints:
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        if 'arm' in joint_name.lower() or 'shoulder' in joint_name.lower():
            right_arm_joints.append(i)
            print(f"Found arm joint: {joint_name} (ID: {i})")

print(f"Using joints: {right_arm_joints}")

# Wave animation parameters
wave_frequency = 2.0  # Hz
wave_amplitude = 0.8  # radians
step_count = 0

# Initial arm position (raised)
initial_positions = [0.5, -0.3, 0.0]  # shoulder up, elbow bent, wrist neutral

while True:
    # Calculate wave motion
    t = step_count * 0.01
    wave_offset = math.sin(t * wave_frequency * 2 * math.pi) * wave_amplitude
    
    # Apply positions to joints
    for i, joint_id in enumerate(right_arm_joints[:3]):  # Use first 3 joints
        if i == 0:  # Shoulder - main wave motion
            target_pos = initial_positions[i] + wave_offset
        elif i == 1:  # Elbow - slight counter motion
            target_pos = initial_positions[i] - wave_offset * 0.3
        else:  # Wrist - minimal motion
            target_pos = initial_positions[i] + wave_offset * 0.1
            
        p.setJointMotorControl2(robot_id, joint_id, p.POSITION_CONTROL, target_pos, force=50)
    
    p.stepSimulation()
    time.sleep(1./240.)
    step_count += 1