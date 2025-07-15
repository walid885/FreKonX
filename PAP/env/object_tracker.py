import pybullet as p
import math
from typing import Tuple

def get_bottle_position(table_bottle_id: int, bottle_link_id: int = 5) -> Tuple[float, float, float]:
    """Get bottle position in world coordinates."""
    link_state = p.getLinkState(table_bottle_id, bottle_link_id)
    return link_state[0]

def get_bottle_orientation(table_bottle_id: int, bottle_link_id: int = 5) -> Tuple[float, float, float, float]:
    """Get bottle orientation in world coordinates."""
    link_state = p.getLinkState(table_bottle_id, bottle_link_id)
    return link_state[1]

def get_relative_position(robot_id: int, target_position: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Get target position relative to robot base."""
    robot_pos, _ = p.getBasePositionAndOrientation(robot_id)
    return (
        target_position[0] - robot_pos[0],
        target_position[1] - robot_pos[1],
        target_position[2] - robot_pos[2]
    )

def calculate_distance(pos1: Tuple[float, float, float], pos2: Tuple[float, float, float]) -> float:
    """Calculate Euclidean distance between two positions."""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))

def get_robot_to_bottle_distance(robot_id: int, bottle_position: Tuple[float, float, float]) -> float:
    """Get distance from robot to bottle."""
    robot_pos, _ = p.getBasePositionAndOrientation(robot_id)
    return calculate_distance(robot_pos, bottle_position)

def is_bottle_in_reach(robot_id: int, bottle_position: Tuple[float, float, float], 
                      max_reach: float = 1.5) -> bool:
    """Check if bottle is within robot's reach."""
    distance = get_robot_to_bottle_distance(robot_id, bottle_position)
    return distance <= max_reach

def get_bottle_info(table_bottle_id: int, robot_id: int, bottle_link_id: int = 5) -> dict:
    """Get comprehensive bottle information."""
    bottle_pos = get_bottle_position(table_bottle_id, bottle_link_id)
    bottle_orn = get_bottle_orientation(table_bottle_id, bottle_link_id)
    relative_pos = get_relative_position(robot_id, bottle_pos)
    distance = get_robot_to_bottle_distance(robot_id, bottle_pos)
    in_reach = is_bottle_in_reach(robot_id, bottle_pos)
    
    return {
        'world_position': bottle_pos,
        'world_orientation': bottle_orn,
        'relative_position': relative_pos,
        'distance': distance,
        'in_reach': in_reach
    }