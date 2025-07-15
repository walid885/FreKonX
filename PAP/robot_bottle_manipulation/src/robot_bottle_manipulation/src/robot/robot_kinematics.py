import pybullet as p
import math
from typing import Tuple, List

class RobotKinematics:
    """Handles robot kinematics calculations."""
    
    def __init__(self, robot_id: int):
        self.robot_id = robot_id
        
    def get_base_position(self) -> Tuple[float, float, float]:
        """Get robot base position."""
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        return pos
        
    def get_base_orientation(self) -> Tuple[float, float, float, float]:
        """Get robot base orientation as quaternion."""
        _, orn = p.getBasePositionAndOrientation(self.robot_id)
        return orn
        
    def get_link_pose(self, link_id: int) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        """Get link position and orientation."""
        link_state = p.getLinkState(self.robot_id, link_id)
        return link_state[0], link_state[1]  # position, orientation
        
    def calculate_distance_to_target(self, target_position: Tuple[float, float, float]) -> float:
        """Calculate distance from robot base to target."""
        robot_pos = self.get_base_position()
        return math.sqrt(
            (target_position[0] - robot_pos[0])**2 +
            (target_position[1] - robot_pos[1])**2 +
            (target_position[2] - robot_pos[2])**2
        )
        
    def get_relative_position(self, target_position: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Get target position relative to robot base."""
        robot_pos = self.get_base_position()
        return (
            target_position[0] - robot_pos[0],
            target_position[1] - robot_pos[1],
            target_position[2] - robot_pos[2]
        )