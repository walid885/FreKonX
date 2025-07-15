import pybullet as p
from typing import List, Dict
from ..core.simulation_config import SimulationConfig, JointInfo

def set_joint_position_control(robot_id: int, joint_id: int, target_position: float, 
                             force: float = 500.0, position_gain: float = 0.3, 
                             velocity_gain: float = 0.1) -> None:
    """Set position control for a single joint."""
    p.setJointMotorControl2(
        robot_id, joint_id, p.POSITION_CONTROL,
        targetPosition=target_position,
        force=force,
        positionGain=position_gain,
        velocityGain=velocity_gain
    )

def set_multiple_joint_positions(robot_id: int, joint_targets: Dict[int, float], 
                               config: SimulationConfig) -> None:
    """Set position control for multiple joints."""
    for joint_id, target_position in joint_targets.items():
        set_joint_position_control(robot_id, joint_id, target_position, 
                                 config.joint_force, config.position_gain, 
                                 config.velocity_gain)

def initialize_joint_positions(robot_id: int, joints: List[JointInfo], 
                             config: SimulationConfig) -> None:
    """Initialize all joints to their current positions."""
    for joint in joints:
        set_joint_position_control(robot_id, joint.id, joint.current_position, 
                                 force=200.0, position_gain=config.position_gain,
                                 velocity_gain=config.velocity_gain)

def calculate_safe_target_position(joint: JointInfo, offset: float = 0.05) -> float:
    """Calculate safe target position within joint limits."""
    current, (lower, upper) = joint.current_position, joint.limits
    
    if lower < upper and abs(lower) < 100 and abs(upper) < 100:
        safe_range = min(0.2, (upper - lower) * 0.1)
        target = current + safe_range * 0.3
        return max(lower, min(upper, target))
    else:
        return current

def apply_balance_control(robot_id: int, balance_joints: List[JointInfo], 
                        initial_positions: Dict[int, float], config: SimulationConfig) -> None:
    """Apply strong position control to balance joints."""
    for joint in balance_joints:
        set_joint_position_control(
            robot_id, joint.id, initial_positions[joint.id], 
            force=1000.0, position_gain=0.5, velocity_gain=0.2
        )

def apply_stabilization_control(robot_id: int, joints: List[JointInfo], 
                              arm_joints: List[JointInfo], balance_joints: List[JointInfo],
                              initial_positions: Dict[int, float]) -> None:
    """Apply stabilization control to non-arm, non-balance joints."""
    arm_ids = {j.id for j in arm_joints}
    balance_ids = {j.id for j in balance_joints}
    
    for joint in joints:
        if joint.id not in arm_ids and joint.id not in balance_ids:
            set_joint_position_control(robot_id, joint.id, initial_positions[joint.id], force=800.0)