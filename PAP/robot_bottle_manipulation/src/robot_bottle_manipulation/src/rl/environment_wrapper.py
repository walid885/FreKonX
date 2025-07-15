import numpy as np
from typing import Dict, List, Tuple, Any
from ..core.simulation_config import SimulationConfig
from ..robot.joint_controller import JointController, JointInfo
from ..environment.object_tracker import ObjectTracker

class RLEnvironmentWrapper:
    """Wrapper for reinforcement learning environment."""
    
    def __init__(self, config: SimulationConfig, robot_id: int, 
                 joint_controller: JointController, object_tracker: ObjectTracker):
        self.config = config
        self.robot_id = robot_id
        self.joint_controller = joint_controller
        self.object_tracker = object_tracker
        
        # Get movable joints
        self.movable_joints = self.joint_controller.get_movable_joints(robot_id)
        self.arm_joints, self.balance_joints = self.joint_controller.find_joint_groups(self.movable_joints)
        
        # Environment parameters
        self.max_episode_steps = 1000
        self.current_step = 0
        
    def get_observation(self) -> np.ndarray:
        """Get current observation state."""
        # Robot joint positions
        joint_positions = [joint.current_position for joint in self.movable_joints]
        
        # Robot base position
        robot_pos = self.joint_controller.get_joint_info(self.robot_id, -1).current_position
        
        # Bottle position relative to robot
        bottle_relative_pos = self.object_tracker.get_bottle_relative_to_robot(self.robot_id)
        
        # Combine all observations
        observation = np.array(joint_positions + list(bottle_relative_pos))
        return observation
    
    def get_observation_space_size(self) -> int:
        """Get size of observation space."""
        return len(self.movable_joints) + 3  # joints + relative bottle position
    
    def get_action_space_size(self) -> int:
        """Get size of action space."""
        return len(self.movable_joints)
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Step the environment with given action."""
        # Apply action to joints
        for i, joint in enumerate(self.movable_joints):
            if i < len(action):
                target_position = self.joint_controller.calculate_safe_target_position(joint)
                # Scale action and apply
                scaled_action = action[i] * 0.1  # Scale factor
                new_target = target_position + scaled_action
                self.joint_controller.set_joint_position_control(
                    self.robot_id, joint.id, new_target
                )
        
        # Get new observation
        observation = self.get_observation()
        
        # Calculate reward
        reward = self._calculate_reward()
        
        # Check if episode is done
        done = self._is_episode_done()
        
        # Update step count
        self.current_step += 1
        
        # Info dictionary
        info = {
            'step': self.current_step,
            'bottle_position': self.object_tracker.get_bottle_position(),
            'robot_base_position': self.joint_controller.get_joint_info(self.robot_id, -1).current_position
        }
        
        return observation, reward, done, info
    
    def reset(self) -> np.ndarray:
        """Reset the environment."""
        self.current_step = 0
        
        # Reset joint positions
        self.joint_controller.initialize_joint_positions(self.robot_id, self.movable_joints)
        
        return self.get_observation()
    
    def _calculate_reward(self) -> float:
        """Calculate reward for current state."""
        # Distance to bottle (negative reward for distance)
        bottle_relative_pos = self.object_tracker.get_bottle_relative_to_robot(self.robot_id)
        distance = np.linalg.norm(bottle_relative_pos)
        distance_reward = -distance
        
        # Stability reward (penalize large joint velocities)
        stability_penalty = 0.0
        for joint in self.movable_joints:
            joint_info = self.joint_controller.get_joint_info(self.robot_id, joint.id)
            # Simple stability check based on position limits
            if abs(joint_info.current_position) > 2.0:
                stability_penalty -= 0.1
        
        return distance_reward + stability_penalty
    
    def _is_episode_done(self) -> bool:
        """Check if episode should end."""
        # Episode ends if max steps reached
        if self.current_step >= self.max_episode_steps:
            return True
        
        # Episode ends if robot falls (z position too low)
        robot_pos = self.joint_controller.get_joint_info(self.robot_id, -1).current_position
        if hasattr(robot_pos, '__len__') and len(robot_pos) > 2:
            if robot_pos[2] < 0.5:  # Robot fell
                return True
        
        # Episode ends if bottle is grasped (simplified check)
        bottle_relative_pos = self.object_tracker.get_bottle_relative_to_robot(self.robot_id)
        distance = np.linalg.norm(bottle_relative_pos)
        if distance < 0.05:  # Very close to bottle
            return True
        
        return False