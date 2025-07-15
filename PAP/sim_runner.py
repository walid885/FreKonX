import time
from typing import Dict
from .core.simulation_config import SimulationConfig, SceneObjects
from .core.physics_engine import step_simulation
from .robot.robot_loader import get_robot_base_position
from .robot.joint_controller import (
    initialize_joint_positions, apply_balance_control, 
    apply_stabilization_control
)
from .environment.object_tracker import get_bottle_info

def stabilize_robot(robot_id: int, config: SimulationConfig) -> None:
    """Stabilize robot for specified number of steps."""
    print("Stabilizing robot...")
    for i in range(config.stabilization_steps):
        step_simulation(config)
        if i % 500 == 0:
            pos, _ = get_robot_base_position(robot_id)
            print(f"Stabilization step {i}: Robot Z position = {pos[2]:.3f}")
        time.sleep(1.0 / config.simulation_rate)

def run_simulation_loop(scene_objects: SceneObjects, config: SimulationConfig,
                       movable_joints, arm_joints, balance_joints, 
                       initial_positions: Dict[int, float]) -> None:
    """Main simulation loop."""
    print("Starting simulation with bottle tracking...")
    
    step_count = 0
    while True:
        # Get bottle information
        bottle_info = get_bottle_info(scene_objects.table_bottle_id, 
                                    scene_objects.robot_id, 
                                    scene_objects.bottle_link_id)
        
        # Apply joint controls
        apply_balance_control(scene_objects.robot_id, balance_joints, 
                            initial_positions, config)
        apply_stabilization_control(scene_objects.robot_id, movable_joints, 
                                  arm_joints, balance_joints, initial_positions)
        
        # Step simulation
        step_simulation(config)
        time.sleep(1.0 / config.simulation_rate)
        step_count += 1
        
        # Debug output
        if step_count % 1000 == 0:
            robot_pos, _ = get_robot_base_position(scene_objects.robot_id)
            print(f"Step {step_count}: Robot Z={robot_pos[2]:.3f}, "
                  f"Bottle distance={bottle_info['distance']:.3f}, "
                  f"In reach={bottle_info['in_reach']}")

def initialize_simulation(scene_objects: SceneObjects, config: SimulationConfig, 
                         movable_joints) -> Dict[int, float]:
    """Initialize simulation with joint positions."""
    # Initialize joint positions
    initialize_joint_positions(scene_objects.robot_id, movable_joints, config)
    
    # Stabilize robot
    stabilize_robot(scene_objects.robot_id, config)
    
    # Store initial positions for balance reference
    initial_positions = {joint.id: joint.current_position for joint in movable_joints}
    
    return initial_positions