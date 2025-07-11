import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from robot_descriptions.loaders.pybullet import load_robot_description
from typing import List, Dict, Tuple, Optional, Any
from functools import partial
from dataclasses import dataclass
import json
import os
import pickle
from collections import defaultdict
import random

@dataclass
class JointInfo:
    id: int
    name: str
    type: int
    limits: Tuple[float, float]
    current_position: float

@dataclass
class SimulationConfig:
    gravity: float = -9.81
    robot_height: float = 1.0
    stabilization_steps: int = 5000
    simulation_rate: float = 240.0
    wave_frequency: float = 0.4
    wave_amplitude: float = 0.2
    elbow_wave_amplitude: float = 0.3
    wrist_wave_amplitude: float = 0.15

@dataclass
class WaveMotionConfig:
    shoulder_lift: float = 0.2
    elbow_bend_base: float = 0.15
    wrist_wave_speed: float = 0.8
    compensation_force: float = 5000
    base_stabilization_force: float = 3000
    leg_force: float = 8000
    torso_force: float = 6000

@dataclass
class StabilityMetrics:
    upright_time: float = 0.0
    max_tilt: float = 0.0
    avg_velocity: float = 0.0
    fell_down: bool = False
    success_rate: float = 0.0
    joint_config: Dict[str, float] = None

class ParameterExplorer:
    """Brute force parameter exploration for stability mapping."""
    
    def __init__(self, robot_id: int, joint_categories: Dict[str, List[JointInfo]]):
        self.robot_id = robot_id
        self.joint_categories = joint_categories
        self.results = []
        self.stable_configs = []
        
    def generate_parameter_ranges(self) -> Dict[str, List[float]]:
        """Generate parameter ranges for exploration."""
        ranges = {}
        
        # Hip joint ranges
        ranges['hip_pitch'] = np.linspace(-0.3, 0.1, 10)
        ranges['hip_roll'] = np.linspace(-0.1, 0.1, 5)
        
        # Knee joint ranges
        ranges['knee_pitch'] = np.linspace(0.0, 0.3, 8)
        
        # Ankle joint ranges
        ranges['ankle_pitch'] = np.linspace(-0.1, 0.1, 5)
        ranges['ankle_roll'] = np.linspace(-0.05, 0.05, 3)
        
        # Torso ranges
        ranges['torso_pitch'] = np.linspace(-0.05, 0.05, 3)
        
        # Arm ranges
        ranges['shoulder_pitch'] = np.linspace(-0.2, 0.5, 8)
        ranges['elbow_pitch'] = np.linspace(0.0, 0.5, 6)
        
        return ranges
    
    def test_configuration(self, config: Dict[str, float], test_duration: int = 2000) -> StabilityMetrics:
        """Test a specific joint configuration."""
        metrics = StabilityMetrics(joint_config=config.copy())
        
        # Apply configuration
        stable_positions = self.config_to_joint_positions(config)
        
        # Reset robot
        p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 1.0], [0, 0, 0, 1])
        
        # Test stability
        upright_count = 0
        max_tilt = 0.0
        velocities = []
        
        for step in range(test_duration):
            # Apply joint positions
            for joint_id, target_pos in stable_positions.items():
                p.setJointMotorControl2(
                    self.robot_id, joint_id, p.POSITION_CONTROL,
                    targetPosition=target_pos,
                    force=5000,
                    positionGain=0.8,
                    velocityGain=0.5
                )
            
            p.stepSimulation()
            
            # Check stability
            pos, orn = p.getBasePositionAndOrientation(self.robot_id)
            linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
            euler = p.getEulerFromQuaternion(orn)
            
            tilt = max(abs(euler[0]), abs(euler[1]))
            max_tilt = max(max_tilt, tilt)
            
            vel_magnitude = sum(v**2 for v in linear_vel)**0.5
            velocities.append(vel_magnitude)
            
            if pos[2] > 0.8 and tilt < 0.3 and vel_magnitude < 1.0:
                upright_count += 1
            
            if pos[2] < 0.5:  # Fell down
                metrics.fell_down = True
                break
        
        metrics.upright_time = upright_count / test_duration
        metrics.max_tilt = max_tilt
        metrics.avg_velocity = np.mean(velocities) if velocities else 0.0
        metrics.success_rate = metrics.upright_time
        
        return metrics
    
    def config_to_joint_positions(self, config: Dict[str, float]) -> Dict[int, float]:
        """Convert parameter config to joint positions."""
        positions = {}
        
        # Map config parameters to actual joints
        for joint_type, joints in self.joint_categories.items():
            for joint in joints:
                name = joint.name.lower()
                
                if 'hip' in name:
                    if 'pitch' in name:
                        positions[joint.id] = config.get('hip_pitch', 0.0)
                    elif 'roll' in name:
                        positions[joint.id] = config.get('hip_roll', 0.0)
                elif 'knee' in name:
                    positions[joint.id] = config.get('knee_pitch', 0.0)
                elif 'ankle' in name:
                    if 'pitch' in name:
                        positions[joint.id] = config.get('ankle_pitch', 0.0)
                    elif 'roll' in name:
                        positions[joint.id] = config.get('ankle_roll', 0.0)
                elif 'torso' in name:
                    positions[joint.id] = config.get('torso_pitch', 0.0)
                elif 'shoulder' in name and 'right' in name:
                    positions[joint.id] = config.get('shoulder_pitch', 0.0)
                elif 'elbow' in name and 'right' in name:
                    positions[joint.id] = config.get('elbow_pitch', 0.1)
                else:
                    positions[joint.id] = joint.current_position
        
        return positions
    
    def explore_parameter_space(self, max_iterations: int = 1000) -> List[StabilityMetrics]:
        """Explore parameter space with random sampling."""
        print(f"Starting parameter exploration with {max_iterations} iterations...")
        
        ranges = self.generate_parameter_ranges()
        
        for iteration in range(max_iterations):
            # Random sample from parameter ranges
            config = {}
            for param_name, param_range in ranges.items():
                config[param_name] = random.choice(param_range)
            
            # Test configuration
            metrics = self.test_configuration(config)
            self.results.append(metrics)
            
            # Store stable configs
            if metrics.success_rate > 0.8 and not metrics.fell_down:
                self.stable_configs.append(metrics)
            
            if iteration % 100 == 0:
                success_count = len(self.stable_configs)
                print(f"Iteration {iteration}: {success_count} stable configs found")
        
        return self.results
    
    def save_results(self, filename: str = "stability_results.pkl"):
        """Save exploration results."""
        with open(filename, 'wb') as f:
            pickle.dump({
                'results': self.results,
                'stable_configs': self.stable_configs
            }, f)
        print(f"Results saved to {filename}")
    
    def get_best_configs(self, top_n: int = 10) -> List[StabilityMetrics]:
        """Get top N most stable configurations."""
        sorted_configs = sorted(self.stable_configs, 
                              key=lambda x: x.success_rate, 
                              reverse=True)
        return sorted_configs[:top_n]

class WavingRobotEnv(gym.Env):
    """Gymnasium environment for robot waving task."""
    
    def __init__(self, stable_configs: List[StabilityMetrics] = None):
        super().__init__()
        
        self.stable_configs = stable_configs or []
        self.robot_id = None
        self.joint_categories = None
        self.current_step = 0
        self.max_steps = 5000
        
        # Action space: joint position adjustments
        self.action_space = spaces.Box(
            low=-0.1, high=0.1, shape=(8,), dtype=np.float32
        )
        
        # Observation space: joint angles + base orientation + velocities
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(20,), dtype=np.float32
        )
        
        self.initialize_physics()
    
    def initialize_physics(self):
        """Initialize PyBullet physics."""
        if not p.isConnected():
            p.connect(p.DIRECT)  # Headless for training
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            
            # Load ground
            p.loadURDF("plane.urdf")
            
            # Physics settings
            p.setPhysicsEngineParameter(
                fixedTimeStep=1.0/240.0,
                numSolverIterations=50,
                numSubSteps=2
            )
    
    def reset(self, seed=None, options=None):
        """Reset environment."""
        super().reset(seed=seed)
        
        # Load robot if not loaded
        if self.robot_id is None:
            self.robot_id = load_robot_description("valkyrie_description")
            movable_joints = self.get_movable_joints()
            self.joint_categories = self.find_critical_balance_joints(movable_joints)
        
        # Reset robot position
        if self.stable_configs:
            # Start from a known stable configuration
            config = random.choice(self.stable_configs)
            initial_positions = self.config_to_joint_positions(config.joint_config)
        else:
            # Default position
            initial_positions = self.get_default_positions()
        
        p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 1.0], [0, 0, 0, 1])
        
        # Set initial joint positions
        for joint_id, pos in initial_positions.items():
            p.resetJointState(self.robot_id, joint_id, pos)
        
        self.current_step = 0
        return self.get_observation(), {}
    
    def step(self, action):
        """Execute one step."""
        # Apply action (joint position adjustments)
        self.apply_action(action)
        
        # Step simulation
        p.stepSimulation()
        
        # Get observation
        obs = self.get_observation()
        
        # Calculate reward
        reward = self.calculate_reward()
        
        # Check if done
        done = self.is_done()
        
        self.current_step += 1
        
        return obs, reward, done, False, {}
    
    def apply_action(self, action):
        """Apply action to robot joints."""
        # Map action to key joints
        joint_indices = [
            'hip', 'knee', 'ankle', 'torso', 
            'shoulder_right', 'elbow_right', 'wrist_right'
        ]
        
        action_idx = 0
        for joint_type in joint_indices:
            if joint_type in self.joint_categories:
                for joint in self.joint_categories[joint_type]:
                    if action_idx < len(action):
                        # Get current position and apply adjustment
                        current_pos = p.getJointState(self.robot_id, joint.id)[0]
                        target_pos = current_pos + action[action_idx] * 0.1
                        
                        # Clamp to joint limits
                        target_pos = max(joint.limits[0], min(joint.limits[1], target_pos))
                        
                        p.setJointMotorControl2(
                            self.robot_id, joint.id, p.POSITION_CONTROL,
                            targetPosition=target_pos,
                            force=3000,
                            positionGain=0.5,
                            velocityGain=0.3
                        )
                        action_idx += 1
    
    def get_observation(self):
        """Get current observation."""
        obs = []
        
        # Base position and orientation
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        obs.extend([pos[2], euler[0], euler[1], euler[2]])
        
        # Base velocity
        linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
        obs.extend(linear_vel)
        obs.extend(angular_vel)
        
        # Joint states (first 10 joints)
        for i in range(min(10, p.getNumJoints(self.robot_id))):
            joint_state = p.getJointState(self.robot_id, i)
            obs.extend([joint_state[0], joint_state[1]])  # position, velocity
        
        # Pad to fixed size
        while len(obs) < 20:
            obs.append(0.0)
        
        return np.array(obs[:20], dtype=np.float32)
    
    def calculate_reward(self):
        """Calculate reward for current state."""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
        
        # Base rewards
        height_reward = max(0, pos[2] - 0.5)  # Stay upright
        tilt_penalty = -(abs(euler[0]) + abs(euler[1]))  # Minimize tilt
        stability_reward = -sum(v**2 for v in linear_vel)  # Minimize movement
        
        # Waving reward (encourage right arm movement)
        waving_reward = 0.0
        if self.current_step > 1000:  # Start waving after stabilization
            right_shoulder_joints = self.joint_categories.get('shoulder_right', [])
            if right_shoulder_joints:
                shoulder_pos = p.getJointState(self.robot_id, right_shoulder_joints[0].id)[0]
                waving_reward = max(0, shoulder_pos - 0.1)  # Reward lifting arm
        
        total_reward = height_reward + tilt_penalty * 0.5 + stability_reward * 0.1 + waving_reward
        
        return total_reward
    
    def is_done(self):
        """Check if episode is done."""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        
        # Episode ends if robot falls or max steps reached
        fell_down = pos[2] < 0.5
        too_tilted = abs(euler[0]) > 0.5 or abs(euler[1]) > 0.5
        max_steps_reached = self.current_step >= self.max_steps
        
        return fell_down or too_tilted or max_steps_reached
    
    def get_movable_joints(self):
        """Get movable joints from robot."""
        num_joints = p.getNumJoints(self.robot_id)
        movable_types = [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]
        
        joints = []
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_state = p.getJointState(self.robot_id, i)
            
            if joint_info[2] in movable_types:
                joints.append(JointInfo(
                    id=i,
                    name=joint_info[1].decode('utf-8'),
                    type=joint_info[2],
                    limits=(joint_info[8], joint_info[9]),
                    current_position=joint_state[0]
                ))
        
        return joints
    
    def find_critical_balance_joints(self, joints):
        """Find and categorize critical balance joints."""
        joint_categories = {
            'hip': [], 'knee': [], 'ankle': [], 'torso': [],
            'shoulder_left': [], 'elbow_left': [], 'shoulder_right': [],
            'elbow_right': [], 'wrist_right': [], 'other': []
        }
        
        for joint in joints:
            name = joint.name.lower()
            
            if 'hip' in name:
                joint_categories['hip'].append(joint)
            elif 'knee' in name:
                joint_categories['knee'].append(joint)
            elif 'ankle' in name:
                joint_categories['ankle'].append(joint)
            elif 'torso' in name or 'spine' in name:
                joint_categories['torso'].append(joint)
            elif ('shoulder' in name) and ('left' in name or 'l_' in name):
                joint_categories['shoulder_left'].append(joint)
            elif ('elbow' in name) and ('left' in name or 'l_' in name):
                joint_categories['elbow_left'].append(joint)
            elif ('shoulder' in name) and ('right' in name or 'r_' in name):
                joint_categories['shoulder_right'].append(joint)
            elif ('elbow' in name) and ('right' in name or 'r_' in name):
                joint_categories['elbow_right'].append(joint)
            elif ('wrist' in name) and ('right' in name or 'r_' in name):
                joint_categories['wrist_right'].append(joint)
            else:
                joint_categories['other'].append(joint)
        
        return joint_categories
    
    def config_to_joint_positions(self, config):
        """Convert config to joint positions."""
        positions = {}
        
        for joint_type, joints in self.joint_categories.items():
            for joint in joints:
                name = joint.name.lower()
                
                if 'hip' in name:
                    if 'pitch' in name:
                        positions[joint.id] = config.get('hip_pitch', 0.0)
                    elif 'roll' in name:
                        positions[joint.id] = config.get('hip_roll', 0.0)
                elif 'knee' in name:
                    positions[joint.id] = config.get('knee_pitch', 0.0)
                elif 'ankle' in name:
                    if 'pitch' in name:
                        positions[joint.id] = config.get('ankle_pitch', 0.0)
                    elif 'roll' in name:
                        positions[joint.id] = config.get('ankle_roll', 0.0)
                elif 'torso' in name:
                    positions[joint.id] = config.get('torso_pitch', 0.0)
                elif 'shoulder' in name and 'right' in name:
                    positions[joint.id] = config.get('shoulder_pitch', 0.0)
                elif 'elbow' in name and 'right' in name:
                    positions[joint.id] = config.get('elbow_pitch', 0.1)
                else:
                    positions[joint.id] = joint.current_position
        
        return positions
    
    def get_default_positions(self):
        """Get default joint positions."""
        positions = {}
        for joint_type, joints in self.joint_categories.items():
            for joint in joints:
                positions[joint.id] = 0.0
        return positions

def main():
    """Main execution function."""
    print("Starting Robot RL Training Pipeline...")
    
    # Initialize physics
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    
    # Load robot
    robot_id = load_robot_description("valkyrie_description")
    p.resetBasePositionAndOrientation(robot_id, [0, 0, 1.0], [0, 0, 0, 1])
    
    # Get joint info
    num_joints = p.getNumJoints(robot_id)
    movable_joints = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        if joint_info[2] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            joint_state = p.getJointState(robot_id, i)
            movable_joints.append(JointInfo(
                id=i,
                name=joint_info[1].decode('utf-8'),
                type=joint_info[2],
                limits=(joint_info[8], joint_info[9]),
                current_position=joint_state[0]
            ))
    
    # Categorize joints
    joint_categories = {
        'hip': [], 'knee': [], 'ankle': [], 'torso': [],
        'shoulder_left': [], 'elbow_left': [], 'shoulder_right': [],
        'elbow_right': [], 'wrist_right': [], 'other': []
    }
    
    for joint in movable_joints:
        name = joint.name.lower()
        if 'hip' in name:
            joint_categories['hip'].append(joint)
        elif 'knee' in name:
            joint_categories['knee'].append(joint)
        elif 'ankle' in name:
            joint_categories['ankle'].append(joint)
        elif 'torso' in name or 'spine' in name:
            joint_categories['torso'].append(joint)
        elif ('shoulder' in name) and ('left' in name or 'l_' in name):
            joint_categories['shoulder_left'].append(joint)
        elif ('elbow' in name) and ('left' in name or 'l_' in name):
            joint_categories['elbow_left'].append(joint)
        elif ('shoulder' in name) and ('right' in name or 'r_' in name):
            joint_categories['shoulder_right'].append(joint)
        elif ('elbow' in name) and ('right' in name or 'r_' in name):
            joint_categories['elbow_right'].append(joint)
        elif ('wrist' in name) and ('right' in name or 'r_' in name):
            joint_categories['wrist_right'].append(joint)
        else:
            joint_categories['other'].append(joint)
    
    print(f"Found {len(movable_joints)} movable joints")
    
    # Step 1: Parameter exploration
    print("\n=== PHASE 1: Parameter Exploration ===")
    explorer = ParameterExplorer(robot_id, joint_categories)
    results = explorer.explore_parameter_space(max_iterations=500)
    explorer.save_results()
    
    best_configs = explorer.get_best_configs(top_n=10)
    print(f"Found {len(best_configs)} stable configurations")
    
    # Step 2: Create Gymnasium environment
    print("\n=== PHASE 2: Gymnasium Environment Setup ===")
    p.disconnect()
    
    env = WavingRobotEnv(stable_configs=best_configs)
    
    # Test environment
    print("Testing environment...")
    obs, _ = env.reset()
    print(f"Observation shape: {obs.shape}")
    print(f"Action space: {env.action_space}")
    
    # Run a few test steps
    for i in range(100):
        action = env.action_space.sample()
        obs, reward, done, truncated, info = env.step(action)
        
        if i % 20 == 0:
            print(f"Step {i}: Reward={reward:.3f}, Done={done}")
        
        if done:
            obs, _ = env.reset()
    
    print("\n=== Setup Complete ===")
    print("Ready for RL training!")
    print("Best stable configurations saved for training initialization")

if __name__ == "__main__":
    main()