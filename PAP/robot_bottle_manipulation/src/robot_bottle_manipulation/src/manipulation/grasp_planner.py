import math
from typing import Tuple, List, Optional
from ..robot.joint_controller import JointInfo

class GraspPlanner:
    """Plans grasping motions for manipulation tasks."""
    
    def __init__(self):
        self.grasp_threshold = 0.05  # Distance threshold for grasping
        
    def plan_approach_to_bottle(self, current_position: Tuple[float, float, float],
                               bottle_position: Tuple[float, float, float],
                               arm_joints: List[JointInfo]) -> List[float]:
        """Plan approach motion to bottle."""
        # Calculate approach vector
        approach_offset = 0.1  # Approach from 10cm away
        dx = bottle_position[0] - current_position[0]
        dy = bottle_position[1] - current_position[1]
        dz = bottle_position[2] - current_position[2]
        
        # Normalize approach vector
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance > 0:
            approach_pos = (
                bottle_position[0] - (dx/distance) * approach_offset,
                bottle_position[1] - (dy/distance) * approach_offset,
                bottle_position[2] - (dz/distance) * approach_offset
            )
        else:
            approach_pos = bottle_position
            
        # Convert to joint angles (simplified)
        return self._position_to_joint_angles(approach_pos, arm_joints)
    
    def plan_grasp_motion(self, bottle_position: Tuple[float, float, float],
                         arm_joints: List[JointInfo]) -> List[float]:
        """Plan final grasping motion."""
        # Move directly to bottle position
        return self._position_to_joint_angles(bottle_position, arm_joints)
    
    def is_within_grasp_range(self, hand_position: Tuple[float, float, float],
                             bottle_position: Tuple[float, float, float]) -> bool:
        """Check if hand is within grasping range of bottle."""
        distance = math.sqrt(
            (hand_position[0] - bottle_position[0])**2 +
            (hand_position[1] - bottle_position[1])**2 +
            (hand_position[2] - bottle_position[2])**2
        )
        return distance <= self.grasp_threshold
    
    def _position_to_joint_angles(self, target_position: Tuple[float, float, float],
                                 arm_joints: List[JointInfo]) -> List[float]:
        """Convert target position to joint angles (simplified inverse kinematics)."""
        target_angles = []
        
        for joint in arm_joints:
            if 'shoulder' in joint.name.lower():
                # Shoulder pitch/yaw based on target position
                if 'pitch' in joint.name.lower():
                    angle = math.atan2(target_position[2], target_position[0])
                else:  # yaw
                    angle = math.atan2(target_position[1], target_position[0])
                target_angles.append(self._clamp_to_limits(angle, joint.limits))
            elif 'elbow' in joint.name.lower():
                # Elbow angle based on distance
                distance = math.sqrt(target_position[0]**2 + target_position[1]**2)
                angle = math.pi/3 * min(1.0, distance / 0.5)  # Scale for arm length
                target_angles.append(self._clamp_to_limits(angle, joint.limits))
            else:
                # Keep other joints at current position
                target_angles.append(joint.current_position)
                
        return target_angles
    
    def _clamp_to_limits(self, value: float, limits: Tuple[float, float]) -> float:
        """Clamp value to joint limits."""
        lower, upper = limits
        if lower < upper and abs(lower) < 100 and abs(upper) < 100:
            return max(lower, min(upper, value))
        return value