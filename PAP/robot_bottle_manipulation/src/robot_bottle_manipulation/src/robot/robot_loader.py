import pybullet as p
from robot_descriptions.loaders.pybullet import load_robot_description
from typing import List
from ..core.simulation_config import SimulationConfig

class RobotLoader:
    """Handles robot loading and initialization."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
        
    def load_robot(self, description: str, position: List[float]) -> int:
        """Load robot from description and set initial position."""
        robot_id = load_robot_description(description)
        p.resetBasePositionAndOrientation(robot_id, position, [0, 0, 0, 1])
        
        # Set robot dynamics for stability
        num_joints = p.getNumJoints(robot_id)
        for i in range(num_joints):
            p.changeDynamics(
                robot_id, i,
                linearDamping=self.config.linear_damping,
                angularDamping=self.config.angular_damping,
                maxJointVelocity=self.config.max_joint_velocity
            )
        
        return robot_id
        
    def get_robot_info(self, robot_id: int) -> dict:
        """Get basic robot information."""
        base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
        num_joints = p.getNumJoints(robot_id)
        
        return {
            'id': robot_id,
            'base_position': base_pos,
            'base_orientation': base_orn,
            'num_joints': num_joints
        }