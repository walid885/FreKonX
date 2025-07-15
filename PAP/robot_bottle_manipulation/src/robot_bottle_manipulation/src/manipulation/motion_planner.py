import math
from typing import List, Tuple
from ..robot.joint_controller import JointInfo

class MotionPlanner:
    """Plans robot motion for manipulation tasks."""
    
    def __init__(self):
        pass
        
    def plan_reaching_motion(self, arm_joints: List[JointInfo], 
                           target_position: Tuple[float, float, float]) -> List[float]:
        """Plan arm motion to reach target position."""
        # Simple implementation - can be extended with inverse kinematics
        target_angles = []
        
        for joint in arm_joints:
            # Simple heuristic based on target position
            if 'shoulder' in joint.name.lower():
                # Shoulder joint - adjust based on target x, y
                angle = math.atan2(target_position[1], target_position[0])
                target_angles.append(self._clamp_to_limits(angle, joint.limits))
            elif 'elbow' in joint.name.lower():
                # Elbow joint - adjust based on distance
                distance = math.sqrt(target_position[0]**2 + target_position[1]**2)
                angle = math.pi/4 * min(1.0, distance)
                target_angles.append(self._clamp_to_limits(angle, joint.limits))
            else:
                # Other joints - keep current position
                target_angles.append(joint.current_position)
                
        return target_angles
    
    def plan_stabilization_motion(self, balance_joints: List[JointInfo]) -> List[float]:
        """Plan motion to maintain robot balance."""
        target_angles = []
        
        for joint in balance_joints:
            # Simple stabilization - keep joints near neutral position
            neutral_pos = (joint.limits[0] + joint.limits[1]) / 2
            if abs(joint.limits[0]) < 100 and abs(joint.limits[1]) < 100:
                target_angles.append(neutral_pos)
            else:
                target_angles.append(joint.current_position)
                
        return target_angles
    
    def _clamp_to_limits(self, value: float, limits: Tuple[float, float]) -> float:
        """Clamp value to joint limits."""
        lower, upper = limits
        if lower < upper and abs(lower) < 100 and abs(upper) < 100:
            return max(lower, min(upper, value))
        return value
    
    def interpolate_motion(self, current_angles: List[float], target_angles: List[float], 
                          step_size: float = 0.01) -> List[float]:
        """Interpolate between current and target angles."""
        interpolated = []
        
        for current, target in zip(current_angles, target_angles):
            diff = target - current
            if abs(diff) < step_size:
                interpolated.append(target)
            else:
                step = step_size if diff > 0 else -step_size
                interpolated.append(current + step)
                
        return interpolated