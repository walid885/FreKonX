import pybullet as p
import pybullet_data
import time
import math
import os
from robot_descriptions.loaders.pybullet import load_robot_description
from typing import List, Dict, Tuple, Optional
from functools import partial
from dataclasses import dataclass

@dataclass
class JointInfo:
    id: int
    name: str
    type: int
    limits: Tuple[float, float]
    current_position: float

@dataclass
class SimulationConfig:
    gravity: float = -9.81
    robot_height: float = 1.0
    table_height: float = 0.75  # Table height
    stabilization_steps: int = 2000
    simulation_rate: float = 240.0
    wave_frequency: float = 0.2
    wave_amplitude: float = 0.05

@dataclass
class SceneObjects:
    robot_id: int
    table_bottle_id: int
    bottle_link_id: int

def initialize_physics_engine() -> None:
    """Initialize PyBullet physics engine with default settings."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    p.changeDynamics(0, -1, lateralFriction=1.0, spinningFriction=0.1, rollingFriction=0.1)

def load_robot(description: str, position: List[float]) -> int:
    """Load robot from description and set initial position."""
    robot_id = load_robot_description(description)
    p.resetBasePositionAndOrientation(robot_id, position, [0, 0, 0, 1])
    
    # Set robot dynamics for stability
    for i in range(p.getNumJoints(robot_id)):
        p.changeDynamics(robot_id, i, 
                        linearDamping=0.1, 
                        angularDamping=0.1,
                        maxJointVelocity=1.0)
    
    return robot_id

def load_table_bottle_scene(urdf_path: str, config: SimulationConfig) -> int:
    """Load table and bottle from URDF file."""
    table_position = [1.0, 0.0, config.table_height]  # Position table in front of robot
    table_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    table_bottle_id = p.loadURDF(urdf_path, table_position, table_orientation)
    
    # Set table dynamics
    p.changeDynamics(table_bottle_id, -1, lateralFriction=1.0, spinningFriction=0.1, rollingFriction=0.1)
    
    # Set bottle dynamics (assuming bottle is link 5 based on URDF structure)
    bottle_link_id = 5  # This is the bottle link in our URDF
    p.changeDynamics(table_bottle_id, bottle_link_id, 
                    mass=0.5, 
                    lateralFriction=0.5, 
                    spinningFriction=0.1, 
                    rollingFriction=0.1)
    
    return table_bottle_id

def get_bottle_position(table_bottle_id: int, bottle_link_id: int = 5) -> Tuple[float, float, float]:
    """Get bottle position in world coordinates."""
    link_state = p.getLinkState(table_bottle_id, bottle_link_id)
    return link_state[0]  # World position

def get_bottle_relative_to_robot(robot_id: int, bottle_position: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Get bottle position relative to robot base."""
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot_id)
    relative_pos = (
        bottle_position[0] - robot_pos[0],
        bottle_position[1] - robot_pos[1],
        bottle_position[2] - robot_pos[2]
    )
    return relative_pos

def create_urdf_file(filename: str = "table_bottle_scene.urdf") -> str:
    """Create URDF file in current directory."""
    urdf_content = '''<?xml version="1.0"?>
<robot name="table_bottle_scene">
  
  <!-- Table -->
  <link name="table_base">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.2 0.8 0.05"/>
      </geometry>
      <material name="wood">
        <color rgba="0.6 0.4 0.2 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.2 0.8 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="20"/>
      <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Table Legs -->
  <link name="table_leg_1">
    <visual>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.75"/>
      </geometry>
      <material name="wood">
        <color rgba="0.6 0.4 0.2 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.75"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <link name="table_leg_2">
    <visual>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.75"/>
      </geometry>
      <material name="wood">
        <color rgba="0.6 0.4 0.2 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.75"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <link name="table_leg_3">
    <visual>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.75"/>
      </geometry>
      <material name="wood">
        <color rgba="0.6 0.4 0.2 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.75"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <link name="table_leg_4">
    <visual>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.75"/>
      </geometry>
      <material name="wood">
        <color rgba="0.6 0.4 0.2 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.75"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.375" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Bottle -->
  <link name="bottle">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <material name="plastic">
        <color rgba="0.2 0.6 0.8 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="table_leg_1_joint" type="fixed">
    <parent link="table_base"/>
    <child link="table_leg_1"/>
    <origin xyz="0.5 0.35 0" rpy="0 0 0"/>
  </joint>

  <joint name="table_leg_2_joint" type="fixed">
    <parent link="table_base"/>
    <child link="table_leg_2"/>
    <origin xyz="0.5 -0.35 0" rpy="0 0 0"/>
  </joint>

  <joint name="table_leg_3_joint" type="fixed">
    <parent link="table_base"/>
    <child link="table_leg_3"/>
    <origin xyz="-0.5 0.35 0" rpy="0 0 0"/>
  </joint>

  <joint name="table_leg_4_joint" type="fixed">
    <parent link="table_base"/>
    <child link="table_leg_4"/>
    <origin xyz="-0.5 -0.35 0" rpy="0 0 0"/>
  </joint>

  <joint name="bottle_joint" type="fixed">
    <parent link="table_base"/>
    <child link="bottle"/>
    <origin xyz="0.3 0.2 0.125" rpy="0 0 0"/>
  </joint>

</robot>'''
    
    with open(filename, 'w') as f:
        f.write(urdf_content)
    
    return os.path.abspath(filename)

def get_joint_info(robot_id: int, joint_id: int) -> JointInfo:
    """Extract joint information from robot."""
    joint_info = p.getJointInfo(robot_id, joint_id)
    joint_state = p.getJointState(robot_id, joint_id)
    
    return JointInfo(
        id=joint_id,
        name=joint_info[1].decode('utf-8'),
        type=joint_info[2],
        limits=(joint_info[8], joint_info[9]),
        current_position=joint_state[0]
    )

def get_movable_joints(robot_id: int) -> List[JointInfo]:
    """Get all movable joints from robot."""
    num_joints = p.getNumJoints(robot_id)
    movable_types = [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]
    
    return [
        get_joint_info(robot_id, i) 
        for i in range(num_joints)
        if get_joint_info(robot_id, i).type in movable_types
    ]

def filter_joints_by_keywords(joints: List[JointInfo], keywords: List[str]) -> List[JointInfo]:
    """Filter joints by name keywords."""
    return [
        joint for joint in joints
        if any(keyword in joint.name.lower() for keyword in keywords)
    ]

def find_joint_groups(joints: List[JointInfo]) -> Tuple[List[JointInfo], List[JointInfo]]:
    """Find arm and balance joint groups."""
    arm_keywords = ['rightarm', 'right_arm', 'rightshoulder', 'rightelbow', 'rightwrist', 'r_arm', 'r_shoulder', 'r_elbow']
    balance_keywords = ['leg', 'ankle', 'hip', 'knee', 'foot', 'torso', 'waist']
    
    arm_joints = filter_joints_by_keywords(joints, arm_keywords)
    if not arm_joints:
        arm_joints = filter_joints_by_keywords(joints, ['arm', 'shoulder'])[:3]
    
    balance_joints = filter_joints_by_keywords(joints, balance_keywords)
    
    return arm_joints, balance_joints

def calculate_safe_target_position(joint: JointInfo, offset: float = 0.05) -> float:
    """Calculate safe target position within joint limits."""
    current, (lower, upper) = joint.current_position, joint.limits
    
    if lower < upper and abs(lower) < 100 and abs(upper) < 100:
        safe_range = min(0.2, (upper - lower) * 0.1)
        target = current + safe_range * 0.3
        return max(lower, min(upper, target))
    else:
        return current

def set_joint_position_control(robot_id: int, joint_id: int, target_position: float, force: float = 500, 
                             position_gain: float = 0.3, velocity_gain: float = 0.1) -> None:
    """Set position control for a single joint with gentler defaults."""
    p.setJointMotorControl2(
        robot_id, joint_id, p.POSITION_CONTROL,
        targetPosition=target_position,
        force=force,
        positionGain=position_gain,
        velocityGain=velocity_gain
    )

def run_simulation(scene_objects: SceneObjects, config: SimulationConfig) -> None:
    """Main simulation loop with bottle tracking."""
    robot_id = scene_objects.robot_id
    table_bottle_id = scene_objects.table_bottle_id
    bottle_link_id = scene_objects.bottle_link_id
    
    # Get joint information
    movable_joints = get_movable_joints(robot_id)
    arm_joints, balance_joints = find_joint_groups(movable_joints)
    
    print(f"Controlling {len(movable_joints)} movable joints")
    print(f"Found {len(arm_joints)} arm joints")
    print(f"Found {len(balance_joints)} balance joints")
    
    # Initialize joint positions
    for joint in movable_joints:
        set_joint_position_control(robot_id, joint.id, joint.current_position, force=200)
    
    # Stabilization
    print("Stabilizing robot...")
    for i in range(config.stabilization_steps):
        p.stepSimulation()
        if i % 500 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot_id)
            print(f"Stabilization step {i}: Robot Z position = {pos[2]:.3f}")
        time.sleep(1.0 / config.simulation_rate)
    
    print("Starting simulation with bottle tracking...")
    
    step_count = 0
    while True:
        # Get bottle position and relative position to robot
        bottle_pos = get_bottle_position(table_bottle_id, bottle_link_id)
        relative_pos = get_bottle_relative_to_robot(robot_id, bottle_pos)
        
        # Apply basic joint control (for now, keep original wave motion)
        for joint in movable_joints:
            set_joint_position_control(robot_id, joint.id, joint.current_position, force=200)
        
        # Step simulation
        p.stepSimulation()
        time.sleep(1.0 / config.simulation_rate)
        step_count += 1
        
        # Debug output
        if step_count % 1000 == 0:
            robot_pos, _ = p.getBasePositionAndOrientation(robot_id)
            distance = math.sqrt(relative_pos[0]**2 + relative_pos[1]**2 + relative_pos[2]**2)
            print(f"Step {step_count}: Robot Z={robot_pos[2]:.3f}, Bottle pos={bottle_pos}, Distance={distance:.3f}")

def main():
    """Main entry point."""
    config = SimulationConfig()
    
    # Create URDF file
    urdf_path = create_urdf_file()
    print(f"Created URDF file at: {urdf_path}")
    
    # Initialize simulation
    initialize_physics_engine()
    
    # Load robot and scene
    robot_id = load_robot("valkyrie_description", [0, 0, config.robot_height])
    table_bottle_id = load_table_bottle_scene(urdf_path, config)
    
    scene_objects = SceneObjects(
        robot_id=robot_id,
        table_bottle_id=table_bottle_id,
        bottle_link_id=5  # Bottle link in URDF
    )
    
    print(f"Robot loaded with ID: {robot_id}")
    print(f"Table-bottle scene loaded with ID: {table_bottle_id}")
    
    # Run simulation
    run_simulation(scene_objects, config)

if __name__ == "__main__":
    main()