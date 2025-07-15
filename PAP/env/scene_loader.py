import pybullet as p
import os
from ..core.simulation_config import SimulationConfig

def create_urdf_file(filename: str = "table_bottle_scene.urdf") -> str:
    """Create table-bottle URDF file."""
    urdf_content = '''<?xml version="1.0"?>
<robot name="table_bottle_scene">
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

def load_table_bottle_scene(urdf_path: str, config: SimulationConfig) -> int:
    """Load table and bottle scene from URDF."""
    table_orientation = p.getQuaternionFromEuler([0, 0, 0])
    table_bottle_id = p.loadURDF(urdf_path, config.table_position, table_orientation)
    return table_bottle_id

def configure_table_dynamics(table_bottle_id: int) -> None:
    """Configure table dynamics."""
    p.changeDynamics(table_bottle_id, -1, 
                    lateralFriction=1.0, 
                    spinningFriction=0.1, 
                    rollingFriction=0.1)

def configure_bottle_dynamics(table_bottle_id: int, bottle_link_id: int = 5) -> None:
    """Configure bottle dynamics."""
    p.changeDynamics(table_bottle_id, bottle_link_id, 
                    mass=0.5, 
                    lateralFriction=0.5, 
                    spinningFriction=0.1, 
                    rollingFriction=0.1)

def setup_scene(config: SimulationConfig) -> int:
    """Complete scene setup."""
    urdf_path = create_urdf_file()
    table_bottle_id = load_table_bottle_scene(urdf_path, config)
    configure_table_dynamics(table_bottle_id)
    configure_bottle_dynamics(table_bottle_id)
    return table_bottle_id