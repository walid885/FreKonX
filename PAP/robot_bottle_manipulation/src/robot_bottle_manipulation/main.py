#!/usr/bin/env python3
"""
Main entry point for the robot bottle manipulation simulation.
"""

import time
import math
from dataclasses import dataclass
from typing import Tuple

from src.core.physics_engine import PhysicsEngine
from src.core.simulation_config import SimulationConfig
from src.robot.robot_loader import RobotLoader
from src.robot.joint_controller import JointController
from src.robot.robot_kinematics import RobotKinematics
from src.environment.scene_loader import SceneLoader
from src.environment.object_tracker import ObjectTracker
from src.manipulation.motion_planner import MotionPlanner
from src.manipulation.grasp_planner import GraspPlanner

@dataclass
class SimulationState:
    """Holds the current state of the simulation."""
    robot_id: int
    table_bottle_id: int
    bottle_link_id: int = 5

class RobotBottleSimulation:
    """Main simulation class for robot bottle manipulation."""
    
    def __init__(self):
        self.config = SimulationConfig()
        self.physics_engine = PhysicsEngine(self.config)
        self.robot_loader = RobotLoader(self.config)
        self.joint_controller = JointController(self.config)
        self.scene_loader = SceneLoader(self.config)
        self.motion_planner = MotionPlanner()
        self.grasp_planner = GraspPlanner()
        
        # Simulation state
        self.state = None
        self.object_tracker = None
        self.robot_kinematics = None
        
    def setup_simulation(self) -> None:
        """Set up the complete simulation environment."""
        print("Setting up simulation...")
        
        # Initialize physics
        self.physics_engine.initialize()
        
        # Create and load scene
        urdf_path = self.scene_loader.create_urdf_file("assets/table_bottle_scene.urdf")
        print(f"Created URDF file at: {urdf_path}")
        
        # Load robot
        robot_id = self.robot_loader.load_robot(
            "valkyrie_description", 
            [0, 0, self.config.robot_height]
        )
        
        # Load scene
        table_bottle_id = self.scene_loader.load_table_bottle_scene(urdf_path)
        
        # Create simulation state
        self.state = SimulationState(
            robot_id=robot_id,
            table_bottle_id=table_bottle_id
        )
        
        # Initialize trackers
        self.object_tracker = ObjectTracker(table_bottle_id, self.state.bottle_link_id)
        self.robot_kinematics = RobotKinematics(robot_id)
        
        print(f"Robot loaded with ID: {robot_id}")
        print(f"Table-bottle scene loaded with ID: {table_bottle_id}")
        
    def stabilize_robot(self) -> None:
        """Stabilize robot before starting main simulation."""
        print("Stabilizing robot...")
        
        # Get joint information
        movable_joints = self.joint_controller.get_movable_joints(self.state.robot_id)
        
        # Initialize joint positions
        self.joint_controller.initialize_joint_positions(self.state.robot_id, movable_joints)
        
        # Run stabilization steps
        for i in range(self.config.stabilization_steps):
            self.physics_engine.step()
            
            if i % 500 == 0:
                robot_pos = self.robot_kinematics.get_base_position()
                print(f"Stabilization step {i}: Robot Z position = {robot_pos[2]:.3f}")
                
            time.sleep(1.0 / self.config.simulation_rate)

        
        print("Robot stabilized.")
        
    def run_simulation(self) -> None:
        """Run the main simulation loop."""
        print("Starting main simulation...")
        
        # Get joint information
        movable_joints = self.joint_controller.get_movable_joints(self.state.robot_id)
        arm_joints, balance_joints = self.joint_controller.find_joint_groups(movable_joints)
        
        print(f"Controlling {len(movable_joints)} movable joints")
        print(f"Found {len(arm_joints)} arm joints: {[j.name for j in arm_joints]}")
        print(f"Found {len(balance_joints)} balance joints: {[j.name for j in balance_joints]}")
        
        step_count = 0
        
        try:
            while True:
                # Get current positions
                bottle_pos = self.object_tracker.get_bottle_position()
                robot_pos = self.robot_kinematics.get_base_position()
                relative_pos = self.robot_kinematics.get_relative_position(bottle_pos)
                
                # Plan motion
                if len(arm_joints) > 0:
                    target_angles = self.motion_planner.plan_reaching_motion(arm_joints, relative_pos)
                    
                    # Apply motion to arm joints
                    for joint, target in zip(arm_joints, target_angles):
                        self.joint_controller.set_joint_position_control(
                            self.state.robot_id, joint.id, target
                        )
                
                # Maintain balance using active controller
                if len(balance_joints) > 0:
                    self.joint_controller.maintain_balance(self.state.robot_id, balance_joints)
                                
                # Step physics
                self.physics_engine.step()
                time.sleep(1.0 / self.config.simulation_rate)
                step_count += 1
                
                # Debug output
                if step_count % 1000 == 0:
                    distance = self.robot_kinematics.calculate_distance_to_target(bottle_pos)
                    print(f"Step {step_count}: Robot pos={robot_pos}, "
                          f"Bottle pos={bottle_pos}, Distance={distance:.3f}")
                    
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user.")
        finally:
            self.cleanup()
    
    def cleanup(self) -> None:
        """Clean up simulation resources."""
        print("Cleaning up simulation...")
        self.physics_engine.disconnect()
        print("Simulation ended.")

def main():
    """Main entry point."""
    simulation = RobotBottleSimulation()
    
    try:
        simulation.setup_simulation()
        simulation.stabilize_robot()
        simulation.run_simulation()
    except Exception as e:
        print(f"Error during simulation: {e}")
        simulation.cleanup()

if __name__ == "__main__":
    main()