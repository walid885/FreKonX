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

# Load Valkyrie at proper height
robot_id = load_robot_description("valkyrie_description")
# Reset position to 1m above ground
p.resetBasePositionAndOrientation(robot_id, [0, 0, 1.0], [0, 0, 0, 1])

print(f"Robot loaded with ID: {robot_id}")

# Get robot info
num_joints = p.getNumJoints(robot_id)
print(f"Total joints: {num_joints}")

# Print all joint names for debugging
print("\nAll joints:")
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    print(f"  {i}: {joint_name} (type: {joint_type})")

# Find right arm joints
right_arm_joints = []
arm_keywords = ['rightarm', 'right_arm', 'rightshoulder', 'rightelbow', 'rightwrist', 'r_arm', 'r_shoulder', 'r_elbow']

for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode('utf-8').lower()
    
    if any(keyword in joint_name for keyword in arm_keywords):
        right_arm_joints.append(i)
        print(f"Found right arm joint: {joint_info[1].decode('utf-8')} (ID: {i})")

# If no specific right arm joints found, look for general arm joints
if not right_arm_joints:
    print("No specific right arm joints found, searching for general arm joints...")
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8').lower()
        if 'arm' in joint_name or 'shoulder' in joint_name:
            right_arm_joints.append(i)
            print(f"Found arm joint: {joint_info[1].decode('utf-8')} (ID: {i})")

print(f"\nUsing joints: {right_arm_joints}")

if not right_arm_joints:
    print("WARNING: No arm joints found! Using first 3 joints as fallback.")
    right_arm_joints = [0, 1, 2]

# Stabilize robot first
print("\nStabilizing robot...")
for i in range(500):  # More stabilization time
    p.stepSimulation()
    if i % 100 == 0:
        pos, orient = p.getBasePositionAndOrientation(robot_id)
        print(f"Robot position at step {i}: {pos}")
    time.sleep(1./240.)

# Get current joint positions
current_positions = []
joint_limits = []
for joint_id in right_arm_joints[:3]:
    joint_state = p.getJointState(robot_id, joint_id)
    joint_info = p.getJointInfo(robot_id, joint_id)
    current_positions.append(joint_state[0])
    joint_limits.append((joint_info[8], joint_info[9]))  # lower, upper limits
    print(f"Joint {joint_id}: current={joint_state[0]:.3f}, limits={joint_info[8]:.3f} to {joint_info[9]:.3f}")

print(f"Current joint positions: {current_positions}")
print(f"Joint limits: {joint_limits}")

# Set conservative target positions within joint limits
target_positions = []
for i, (current, (lower, upper)) in enumerate(zip(current_positions, joint_limits)):
    if lower < upper:  # Valid joint limits
        safe_range = (upper - lower) * 0.3  # Use 30% of range
        target = current + safe_range * 0.5  # Small offset
        target = max(lower, min(upper, target))  # Clamp to limits
    else:
        target = current + 0.1  # Small offset for unlimited joints
    target_positions.append(target)
    print(f"Joint {i}: target position = {target:.3f}")

# Wave animation parameters
wave_frequency = 1.0  # Slower frequency
wave_amplitude = 0.2  # Smaller amplitude
step_count = 0

print("\nStarting wave animation...")

while True:
    # Calculate wave motion
    t = step_count * 0.01
    wave_offset = math.sin(t * wave_frequency * 2 * math.pi) * wave_amplitude
    
    # Apply positions to joints with safety checks
    for i, joint_id in enumerate(right_arm_joints[:3]):
        if i < len(target_positions):
            if i == 0:  # Primary wave joint
                target_pos = target_positions[i] + wave_offset
            elif i == 1:  # Secondary joint
                target_pos = target_positions[i] - wave_offset * 0.3
            else:  # Tertiary joint
                target_pos = target_positions[i] + wave_offset * 0.1
            
            # Clamp to joint limits
            if i < len(joint_limits):
                lower, upper = joint_limits[i]
                if lower < upper:
                    target_pos = max(lower, min(upper, target_pos))
            
            p.setJointMotorControl2(robot_id, joint_id, p.POSITION_CONTROL, target_pos, force=30)
    
    p.stepSimulation()
    time.sleep(1./240.)
    step_count += 1
    
    # Debug output every 1000 steps
    if step_count % 1000 == 0:
        pos, orient = p.getBasePositionAndOrientation(robot_id)
        print(f"Step {step_count}: Robot position = {pos}")