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

print(f"Robot loaded with ID: {robot_id}")

# Get robot info
num_joints = p.getNumJoints(robot_id)
print(f"Total joints: {num_joints}")

# Find all movable joints and set them to position control
movable_joints = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    
    # Only control revolute and prismatic joints
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        movable_joints.append(i)
        
        # Get current position
        joint_state = p.getJointState(robot_id, i)
        current_pos = joint_state[0]
        
        # Set initial position control to hold current position
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, 
                              targetPosition=current_pos, 
                              force=1000)  # High force to maintain position

print(f"Controlling {len(movable_joints)} movable joints")

# Position robot at proper height (adjust based on robot's feet)
# Start higher to account for robot height
p.resetBasePositionAndOrientation(robot_id, [0, 0, 1.2], [0, 0, 0, 1])

# Stabilize robot with all joints held in position
print("Stabilizing robot...")
for i in range(1000):
    p.stepSimulation()
    if i % 200 == 0:
        pos, orient = p.getBasePositionAndOrientation(robot_id)
        print(f"Stabilization step {i}: Robot position = {pos[2]:.3f}")
    time.sleep(1./240.)

# Find right arm joints specifically
right_arm_joints = []
arm_keywords = ['rightarm', 'right_arm', 'rightshoulder', 'rightelbow', 'rightwrist', 'r_arm', 'r_shoulder', 'r_elbow']

for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode('utf-8').lower()
    joint_type = joint_info[2]
    
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        if any(keyword in joint_name for keyword in arm_keywords):
            right_arm_joints.append(i)
            print(f"Found right arm joint: {joint_info[1].decode('utf-8')} (ID: {i})")

# If no specific right arm joints found, look for general arm joints
if not right_arm_joints:
    print("No specific right arm joints found, searching for general arm joints...")
    for i in movable_joints:
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8').lower()
        if 'arm' in joint_name or 'shoulder' in joint_name:
            right_arm_joints.append(i)
            print(f"Found arm joint: {joint_info[1].decode('utf-8')} (ID: {i})")

print(f"Using arm joints: {right_arm_joints}")

if not right_arm_joints:
    print("WARNING: No arm joints found! Using first 3 movable joints as fallback.")
    right_arm_joints = movable_joints[:3]

# Get current joint positions and limits for arm joints
current_positions = []
joint_limits = []
for joint_id in right_arm_joints[:3]:
    joint_state = p.getJointState(robot_id, joint_id)
    joint_info = p.getJointInfo(robot_id, joint_id)
    current_positions.append(joint_state[0])
    joint_limits.append((joint_info[8], joint_info[9]))
    print(f"Arm joint {joint_id}: current={joint_state[0]:.3f}, limits={joint_info[8]:.3f} to {joint_info[9]:.3f}")

# Set conservative target positions for arm movement
target_positions = []
for i, (current, (lower, upper)) in enumerate(zip(current_positions, joint_limits)):
    if lower < upper:
        safe_range = (upper - lower) * 0.2
        target = current + safe_range * 0.5
        target = max(lower, min(upper, target))
    else:
        target = current + 0.1
    target_positions.append(target)

# Wave animation parameters
wave_frequency = 0.8
wave_amplitude = 0.15
step_count = 0

print("Starting wave animation while maintaining standing posture...")

while True:
    # Calculate wave motion
    t = step_count * 0.01
    wave_offset = math.sin(t * wave_frequency * 2 * math.pi) * wave_amplitude
    
    # Keep all non-arm joints in their current positions (maintain standing)
    for joint_id in movable_joints:
        if joint_id not in right_arm_joints:
            joint_state = p.getJointState(robot_id, joint_id)
            p.setJointMotorControl2(robot_id, joint_id, p.POSITION_CONTROL, 
                                  targetPosition=joint_state[0], 
                                  force=1000)
    
    # Apply wave motion to arm joints
    for i, joint_id in enumerate(right_arm_joints[:3]):
        if i < len(target_positions):
            if i == 0:
                target_pos = target_positions[i] + wave_offset
            elif i == 1:
                target_pos = target_positions[i] - wave_offset * 0.3
            else:
                target_pos = target_positions[i] + wave_offset * 0.1
            
            # Clamp to joint limits
            if i < len(joint_limits):
                lower, upper = joint_limits[i]
                if lower < upper:
                    target_pos = max(lower, min(upper, target_pos))
            
            p.setJointMotorControl2(robot_id, joint_id, p.POSITION_CONTROL, 
                                  targetPosition=target_pos, 
                                  force=500)
    
    p.stepSimulation()
    time.sleep(1./240.)
    step_count += 1
    
    # Debug output
    if step_count % 1000 == 0:
        pos, orient = p.getBasePositionAndOrientation(robot_id)
        print(f"Step {step_count}: Robot Z position = {pos[2]:.3f}")