import pybullet as p
from dataclasses import dataclass
from typing import List, Tuple
from ..core.simulation_config import SimulationConfig

@dataclass
class JointInfo:
    """Information about a robot joint."""
    id: int
    name: str
    type: int
    limits: Tuple[float, float]
    current_position: float

class JointController:
    """Handles robot joint control and information."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
        
    def get_joint_info(self, robot_id: int, joint_id: int) -> JointInfo:
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
    
    def get_movable_joints(self, robot_id: int) -> List[JointInfo]:
        """Get all movable joints from robot."""
        num_joints = p.getNumJoints(robot_id)
        movable_types = [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]
        
        return [
            self.get_joint_info(robot_id, i) 
            for i in range(num_joints)
            if self.get_joint_info(robot_id, i).type in movable_types
        ]
    
    def filter_joints_by_keywords(self, joints: List[JointInfo], keywords: List[str]) -> List[JointInfo]:
        """Filter joints by name keywords."""
        return [
            joint for joint in joints
            if any(keyword in joint.name.lower() for keyword in keywords)
        ]
    
    def find_joint_groups(self, joints: List[JointInfo]) -> Tuple[List[JointInfo], List[JointInfo]]:
        """Find arm and balance joint groups."""
        arm_keywords = ['rightarm', 'right_arm', 'rightshoulder', 'rightelbow', 'rightwrist', 'r_arm', 'r_shoulder', 'r_elbow']
        balance_keywords = ['leg', 'ankle', 'hip', 'knee', 'foot', 'torso', 'waist']
        
        arm_joints = self.filter_joints_by_keywords(joints, arm_keywords)
        if not arm_joints:
            arm_joints = self.filter_joints_by_keywords(joints, ['arm', 'shoulder'])[:3]
        
        balance_joints = self.filter_joints_by_keywords(joints, balance_keywords)
        return arm_joints, balance_joints
    
    def calculate_safe_target_position(self, joint: JointInfo, offset: float = 0.05) -> float:
        """Calculate safe target position within joint limits."""
        current, (lower, upper) = joint.current_position, joint.limits
        
        if lower < upper and abs(lower) < 100 and abs(upper) < 100:
            safe_range = min(0.2, (upper - lower) * 0.1)
            target = current + safe_range * 0.3
            return max(lower, min(upper, target))
        else:
            return current
    
    def set_joint_position_control(self, robot_id: int, joint_id: int, target_position: float, 
                                 force: float = None, position_gain: float = None, 
                                 velocity_gain: float = None) -> None:
        """Set position control for a single joint."""
        force = force or self.config.joint_force
        position_gain = position_gain or self.config.position_gain
        velocity_gain = velocity_gain or self.config.velocity_gain
        
        p.setJointMotorControl2(
            robot_id, joint_id, p.POSITION_CONTROL,
            targetPosition=target_position,
            force=force,
            positionGain=position_gain,
            velocityGain=velocity_gain
        )
    
    def initialize_joint_positions(self, robot_id: int, joints: List[JointInfo]) -> None:
        """Initialize joint positions with current values."""
        for joint in joints:
            # Use higher force for balance-critical joints
            force = 1000 if any(kw in joint.name.lower() for kw in ['leg', 'ankle', 'hip']) else 500
            self.set_joint_position_control(robot_id, joint.id, joint.current_position, force=force)
    
    def maintain_balance(self, robot_id: int, balance_joints: List[JointInfo]) -> None:
        """Actively maintain robot balance."""
        base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
        base_euler = p.getEulerFromQuaternion(base_orn)
        
        # Simple balance controller - keep robot upright
        for joint in balance_joints:
            current_joint_info = self.get_joint_info(robot_id, joint.id)
            joint_name = joint.name.lower()
            
            if 'ankle' in joint_name or 'foot' in joint_name:
                # Ankle control for pitch/roll balance
                pitch_correction = -base_euler[0] * 0.3  # Pitch (forward/backward)
                roll_correction = -base_euler[1] * 0.3   # Roll (left/right)
                
                if 'pitch' in joint_name or 'y' in joint_name:
                    target = current_joint_info.current_position + pitch_correction
                elif 'roll' in joint_name or 'x' in joint_name:
                    target = current_joint_info.current_position + roll_correction
                else:
                    target = current_joint_info.current_position
                    
                self.set_joint_position_control(robot_id, joint.id, target, force=1500)
            
            elif 'hip' in joint_name or 'leg' in joint_name:
                # Hip stabilization
                target = current_joint_info.current_position
                self.set_joint_position_control(robot_id, joint.id, target, force=1200)
            
            elif 'knee' in joint_name:
                # Knee stabilization - slight bend for stability
                target = current_joint_info.current_position + 0.1
                self.set_joint_position_control(robot_id, joint.id, target, force=1000)
            
            else:
                # Default stabilization
                self.set_joint_position_control(robot_id, joint.id, current_joint_info.current_position, force=800)