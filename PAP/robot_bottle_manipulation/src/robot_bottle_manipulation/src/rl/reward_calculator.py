import numpy as np
import math
from typing import Dict, Any, Tuple

class RewardCalculator:
    """Calculates rewards for reinforcement learning."""
    
    def __init__(self):
        # Reward weights
        self.distance_weight = -1.0
        self.stability_weight = -0.1
        self.progress_weight = 0.5
        self.grasp_reward = 10.0
        self.collision_penalty = -5.0
        
        # Previous state for progress calculation
        self.prev_distance = None
        
    def calculate_reward(self, robot_pos: Tuple[float, float, float],
                        bottle_pos: Tuple[float, float, float],
                        joint_positions: list,
                        joint_limits: list,
                        episode_info: Dict[str, Any]) -> float:
        """Calculate total reward for current state."""
        
        # Distance reward
        distance = self._calculate_distance(robot_pos, bottle_pos)
        distance_reward = self.distance_weight * distance
        
        # Progress reward
        progress_reward = self._calculate_progress_reward(distance)
        
        # Stability reward
        stability_reward = self._calculate_stability_reward(joint_positions, joint_limits)
        
        # Grasp reward
        grasp_reward = self._calculate_grasp_reward(distance)
        
        # Collision penalty
        collision_penalty = self._calculate_collision_penalty(robot_pos)
        
        total_reward = (distance_reward + progress_reward + stability_reward + 
                       grasp_reward + collision_penalty)
        
        return total_reward
    
    def _calculate_distance(self, robot_pos: Tuple[float, float, float],
                           bottle_pos: Tuple[float, float, float]) -> float:
        """Calculate distance between robot and bottle."""
        return math.sqrt(
            (robot_pos[0] - bottle_pos[0])**2 +
            (robot_pos[1] - bottle_pos[1])**2 +
            (robot_pos[2] - bottle_pos[2])**2
        )
    
    def _calculate_progress_reward(self, current_distance: float) -> float:
        """Calculate reward based on progress towards bottle."""
        if self.prev_distance is None:
            self.prev_distance = current_distance
            return 0.0
        
        progress = self.prev_distance - current_distance
        self.prev_distance = current_distance
        
        return self.progress_weight * progress
    
    def _calculate_stability_reward(self, joint_positions: list, joint_limits: list) -> float:
        """Calculate reward based on robot stability."""
        penalty = 0.0
        
        for pos, (lower, upper) in zip(joint_positions, joint_limits):
            # Penalize joints near limits
            if lower < upper and abs(lower) < 100 and abs(upper) < 100:
                range_size = upper - lower
                center = (upper + lower) / 2
                deviation = abs(pos - center) / (range_size / 2)
                if deviation > 0.8:  # Near limits
                    penalty += 0.1
        
        return self.stability_weight * penalty
    
    def _calculate_grasp_reward(self, distance: float) -> float:
        """Calculate reward for successful grasping."""
        if distance < 0.05:  # Very close to bottle
            return self.grasp_reward
        elif distance < 0.1:  # Close to bottle
            return self.grasp_reward * 0.5
        return 0.0
    
    def _calculate_collision_penalty(self, robot_pos: Tuple[float, float, float]) -> float:
        """Calculate penalty for collisions or falls."""
        # Simple fall detection
        if robot_pos[2] < 0.5:  # Robot too low
            return self.collision_penalty
        return 0.0
    
    def reset(self):
        """Reset calculator state."""
        self.prev_distance = None