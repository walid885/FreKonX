import pybullet as p
import os
from typing import Tuple
from ..core.simulation_config import SimulationConfig

class SceneLoader:
    """Handles scene loading and URDF file creation."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
        
    def create_urdf_file(self, filename: str = "table_bottle_scene.urdf") -> str:
        """Create URDF file for table and bottle scene."""
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
    
    def load_table_bottle_scene(self, urdf_path: str) -> int:
        """Load table and bottle from URDF file."""
        table_position = list(self.config.table_position)
        table_orientation = p.getQuaternionFromEuler([0, 0, 0])
        
        table_bottle_id = p.loadURDF(urdf_path, table_position, table_orientation)
        
        # Set table dynamics
        p.changeDynamics(
            table_bottle_id, -1,
            lateralFriction=self.config.lateral_friction,
            spinningFriction=self.config.spinning_friction,
            rollingFriction=self.config.rolling_friction
        )
        
        # Set bottle dynamics (link 5 is the bottle)
        bottle_link_id = 5
        p.changeDynamics(
            table_bottle_id, bottle_link_id,
            mass=self.config.bottle_mass,
            lateralFriction=self.config.bottle_lateral_friction,
            spinningFriction=self.config.spinning_friction,
            rollingFriction=self.config.rolling_friction
        )
        
        return table_bottle_id