import pybullet as p
from robot_descriptions.loaders.pybullet import load_robot_description
from typing import List
from ..core.simulation_config import SimulationConfig, JointInfo

def load_robot(config: SimulationConfig) -> int:
    """Load robot from description."""
    robot_id = load_robot_description(config.robot_description)
    p.resetBasePositionAndOrientation(robot_id, config.robot_position, [0, 0, 0, 1])
    return robot_id

def configure_robot_dynamics(robot_id: int) -> None:
    """Configure robot joint dynamics for stability."""
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        p.changeDynamics(robot_id, i, 
                        linearDamping=0.1, 
                        angularDamping=0.1,
                        maxJointVelocity=1.0)

def get_joint_info(robot_id: int, joint_id: int) -> JointInfo:
    """Get information for a specific joint."""
    joint_info = p.getJointInfo(robot_id, joint_id)
    joint_state = p.getJointState(robot_id, joint_id)
    
    return JointInfo(
        id=joint_id,
        name=joint_info[1].decode('utf-8'),
        type=joint_info[2],
        limits=(joint_info[8], joint_info[9]),
        current_position=joint_state[0]
    )

def get_all_joints(robot_id: int) -> List[JointInfo]:
    """Get information for all joints."""
    num_joints = p.getNumJoints(robot_id)
    return [get_joint_info(robot_id, i) for i in range(num_joints)]

def get_movable_joints(robot_id: int) -> List[JointInfo]:
    """Get only movable joints (revolute and prismatic)."""
    all_joints = get_all_joints(robot_id)
    movable_types = [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]
    return [joint for joint in all_joints if joint.type in movable_types]

def filter_joints_by_keywords(joints: List[JointInfo], keywords: List[str]) -> List[JointInfo]:
    """Filter joints by name keywords."""
    return [
        joint for joint in joints
        if any(keyword in joint.name.lower() for keyword in keywords)
    ]

def find_arm_joints(joints: List[JointInfo]) -> List[JointInfo]:
    """Find arm joints from joint list."""
    arm_keywords = ['rightarm', 'right_arm', 'rightshoulder', 'rightelbow', 'rightwrist', 'r_arm', 'r_shoulder', 'r_elbow']
    arm_joints = filter_joints_by_keywords(joints, arm_keywords)
    if not arm_joints:
        arm_joints = filter_joints_by_keywords(joints, ['arm', 'shoulder'])[:3]
    return arm_joints

def find_balance_joints(joints: List[JointInfo]) -> List[JointInfo]:
    """Find balance-related joints."""
    balance_keywords = ['leg', 'ankle', 'hip', 'knee', 'foot', 'torso', 'waist']
    return filter_joints_by_keywords(joints, balance_keywords)

def get_robot_base_position(robot_id: int) -> tuple:
    """Get robot base position and orientation."""
    return p.getBasePositionAndOrientation(robot_id)