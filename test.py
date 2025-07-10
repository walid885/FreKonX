import pybullet as p
import pybullet_data
import time

# Connect to the PyBullet GUI
p.connect(p.GUI)

# Set the path to PyBullet's data directory
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# --- Simulation Setup ---

# Load a flat ground plane
# This provides a surface for objects to interact with
ground_plane_id = p.loadURDF("plane.urdf")

# Load the R2D2 robot model
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5]) # Start slightly above ground
# The basePosition is adjusted to prevent it from spawning inside the ground

# Set the gravity for the simulation
p.setGravity(0, 0, -9.8)

# --- R2D2 Movement Setup ---

# Get information about the R2D2 robot's joints
# We need to find the joint IDs for the wheels to control them
num_joints = p.getNumJoints(robot_id)
print(f"R2D2 has {num_joints} joints.")

# Initialize lists to store wheel joint IDs
wheel_joints = []
# Common joint names for R2D2's main drive wheels
# These might vary slightly depending on the specific URDF,
# but 'left_wheel_joint' and 'right_wheel_joint' are typical.
# We'll iterate and print joint info to help identify them if needed.
expected_wheel_names = ["left_wheel_joint", "right_wheel_joint"]

# Iterate through all joints to find the wheel joints
for i in range(num_joints):
    # Get joint info: jointIndex, jointName, jointType, qIndex, uIndex,
    # flags, jointDamping, jointFriction, jointLowerLimit, jointUpperLimit,
    # jointMaxForce, jointMaxVelocity, linkName, jointAxis, parentFramePos,
    # parentFrameOrn, parentIndex
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode("utf-8") # Joint name is a byte string, decode it

    print(f"Joint {i}: Name = {joint_name}, Type = {joint_info[2]}")

    # Check if the joint name matches our expected wheel names
    # Or if it's a revolute joint (for wheels) and its name suggests it's a wheel
    if joint_name in expected_wheel_names or \
       (joint_info[2] == p.JOINT_REVOLUTE and "wheel" in joint_name.lower()):
        wheel_joints.append(i)
        print(f"Identified wheel joint: {joint_name} (ID: {i})")

if not wheel_joints:
    print("Warning: No wheel joints identified. R2D2 may not move.")
    print("Please inspect the printed joint names and adjust 'expected_wheel_names' if necessary.")
else:
    print(f"Found {len(wheel_joints)} wheel joints: {wheel_joints}")

# Set a target velocity for the wheels
# Positive velocity will make the robot move forward (assuming correct wheel orientation)
wheel_velocity = 25 # radians per second

# --- Simulation Loop ---

print("PyBullet simulation started. R2D2 will move forward. Close the GUI window to stop.")

# Run the simulation indefinitely until the GUI connection is lost
while p.isConnected():
    # Apply motor control to each identified wheel joint
    for joint_id in wheel_joints:
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_id,
            controlMode=p.VELOCITY_CONTROL, # Control the joint's velocity
            targetVelocity=wheel_velocity,  # Set the desired velocity
            force=50 # Maximum force the motor can apply (adjust as needed)
        )

    # Advance the simulation by one step
    p.stepSimulation()

    # Add a small delay for visualization
    time.sleep(1./240.) # Aim for 240 frames per second

# Disconnect from the PyBullet physics server
p.disconnect()

print("PyBullet simulation stopped.")
