import pybullet as p
from typing import Tuple

class ObjectTracker:
    """Tracks objects in the simulation environment."""
    
    def __init__(self, table_bottle_id: int, bottle_link_id: int = 5):
        self.table_bottle_id = table_bottle_id
        self.bottle_link_id = bottle_link_id
        
    def get_bottle_position(self) -> Tuple[float, float, float]:
        """Get bottle position in world coordinates."""
        link_state = p.getLinkState(self.table_bottle_id, self.bottle_link_id)
        return link_state[0]  # World position
        
    def get_bottle_orientation(self) -> Tuple[float, float, float, float]:
        """Get bottle orientation as quaternion."""
        link_state = p.getLinkState(self.table_bottle_id, self.bottle_link_id)
        return link_state[1]  # World orientation
        
    def get_bottle_state(self) -> dict:
        """Get complete bottle state information."""
        link_state = p.getLinkState(self.table_bottle_id, self.bottle_link_id)
        return {
            'position': link_state[0],
            'orientation': link_state[1],
            'linear_velocity': link_state[6] if len(link_state) > 6 else None,
            'angular_velocity': link_state[7] if len(link_state) > 7 else None
        }
        
    def get_bottle_relative_to_robot(self, robot_id: int) -> Tuple[float, float, float]:
        """Get bottle position relative to robot base."""
        bottle_pos = self.get_bottle_position()
        robot_pos, _ = p.getBasePositionAndOrientation(robot_id)
        
        return (
            bottle_pos[0] - robot_pos[0],
            bottle_pos[1] - robot_pos[1],
            bottle_pos[2] - robot_pos[2]
        )