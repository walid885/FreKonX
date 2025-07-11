class GPUAcceleratedWavingRobotEnv(gym.Env):
    """GPU-accelerated Gymnasium environment for robot waving task."""
    
    def __init__(self, stable_configs: List[StabilityMetrics] = None, use_gpu: bool = True):
        super().__init__()
        
        self.stable_configs = stable_configs or []
        self.use_gpu = use_gpu and torch.cuda.is_available()
        self.device = torch.device('cuda' if self.use_gpu else 'cpu')
        
        self.robot_id = None
        self.joint_categories = None
        self.current_step = 0
        self.max_steps = 3000  # Reduced for faster episodes
        
        # Action space: joint position adjustments
        self.action_space = spaces.Box(
            low=-0.1, high=0.1, shape=(8,), dtype=np.float32
        )
        
        # Observation space: joint angles + base orientation + velocities
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(20,), dtype=np.float32
        )
        
        # GPU buffers for fast computation
        if self.use_gpu:
            self.obs_buffer = torch.zeros(20, device=self.device)
            self.reward_buffer = torch.zeros(1, device=self.device)
        
        self.initialize_physics()
        
    def initialize_physics(self):
        """Initialize PyBullet physics with GPU optimization."""
        if not p.isConnected():
            p.connect(p.DIRECT)  # Always headless for speed
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            
            # Load ground
            p.loadURDF("plane.urdf")
            
            # Optimized physics settings for speed
            p.setPhysicsEngineParameter(
                fixedTimeStep=1.0/120.0,  # Reduced from 240Hz
                numSolverIterations=30,   # Reduced from 50
                numSubSteps=1,           # Reduced from 2
                enableConeFriction=False, # Disable for speed
                useSplitImpulse=False    # Disable for speed
            )
    
    def step(self, action):
        """Execute one step with GPU acceleration."""
        # Convert action to tensor if using GPU
        if self.use_gpu:
            action_tensor = torch.from_numpy(action).to(self.device)
        
        # Apply action
        self.apply_action(action)
        
        # Step simulation
        p.stepSimulation()
        
        # Get observation
        obs = self.get_observation()
        
        # Calculate reward (GPU accelerated if available)
        reward = self.calculate_reward_gpu() if self.use_gpu else self.calculate_reward()
        
        # Check if done
        done = self.is_done()
        
        self.current_step += 1
        
        return obs, reward, done, False, {}
    
    def calculate_reward_gpu(self):
        """GPU-accelerated reward calculation."""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
        
        # Convert to tensors
        pos_tensor = torch.tensor(pos, device=self.device)
        euler_tensor = torch.tensor(euler, device=self.device)
        vel_tensor = torch.tensor(linear_vel, device=self.device)
        
        # Vectorized reward computation
        height_reward = torch.clamp(pos_tensor[2] - 0.5, min=0)
        tilt_penalty = -(torch.abs(euler_tensor[0]) + torch.abs(euler_tensor[1]))
        stability_reward = -torch.sum(vel_tensor**2)
        
        # Waving reward
        waving_reward = torch.tensor(0.0, device=self.device)
        if self.current_step > 1000:
            right_shoulder_joints = self.joint_categories.get('shoulder_right', [])
            if right_shoulder_joints:
                shoulder_pos = p.getJointState(self.robot_id, right_shoulder_joints[0].id)[0]
                waving_reward = torch.clamp(torch.tensor(shoulder_pos - 0.1, device=self.device), min=0)
        
        total_reward = height_reward + tilt_penalty * 0.5 + stability_reward * 0.1 + waving_reward
        
        return total_reward.cpu().numpy()
    
    def apply_action(self, action):
        """Apply action to robot joints with batch control."""
        # Collect joint info for batch processing
        joint_indices = ['hip', 'knee', 'ankle', 'torso', 'shoulder_right', 'elbow_right', 'wrist_right']
        
        joint_ids = []
        current_positions = []
        target_positions = []
        forces = []
        
        action_idx = 0
        for joint_type in joint_indices:
            if joint_type in self.joint_categories:
                for joint in self.joint_categories[joint_type]:
                    if action_idx < len(action):
                        current_pos = p.getJointState(self.robot_id, joint.id)[0]
                        target_pos = current_pos + action[action_idx] * 0.1
                        target_pos = max(joint.limits[0], min(joint.limits[1], target_pos))
                        
                        joint_ids.append(joint.id)
                        current_positions.append(current_pos)
                        target_positions.append(target_pos)
                        forces.append(3000)
                        action_idx += 1
        
        # Batch joint control (if supported by PyBullet version)
        for i, joint_id in enumerate(joint_ids):
            p.setJointMotorControl2(
                self.robot_id, joint_id, p.POSITION_CONTROL,
                targetPosition=target_positions[i],
                force=forces[i],
                positionGain=0.5,
                velocityGain=0.3
            )
    
    # Keep the rest of the methods the same...
    def get_observation(self):
        """Get current observation with GPU optimization."""
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
            obs.extend([joint_state[0], joint_state[1]])
        
        # Pad to fixed size
        while len(obs) < 20:
            obs.append(0.0)
        
        obs_array = np.array(obs[:20], dtype=np.float32)
        
        # Transfer to GPU if available
        if self.use_gpu:
            self.obs_buffer[:] = torch.from_numpy(obs_array).to(self.device)
            return self.obs_buffer.cpu().numpy()
        
        return obs_array
    
    def calculate_reward(self):
        """Standard CPU reward calculation."""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
        
        # Base rewards
        height_reward = max(0, pos[2] - 0.5)
        tilt_penalty = -(abs(euler[0]) + abs(euler[1]))
        stability_reward = -sum(v**2 for v in linear_vel)
        
        # Waving reward
        waving_reward = 0.0
        if self.current_step > 1000:
            right_shoulder_joints = self.joint_categories.get('shoulder_right', [])
            if right_shoulder_joints:
                shoulder_pos = p.getJointState(self.robot_id, right_shoulder_joints[0].id)[0]
                waving_reward = max(0, shoulder_pos - 0.1)
        
        total_reward = height_reward + tilt_penalty * 0.5 + stability_reward * 0.1 + waving_reward
        
        return total_reward
    
    def is_done(self):
        """Check if episode is done."""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        
        fell_down = pos[2] < 0.5
        too_tilted = abs(euler[0]) > 0.5 or abs(euler[1]) > 0.5
        max_steps_reached = self.current_step >= self.max_steps
        
        return fell_down or too_tilted or max_steps_reached
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
import torch
import torch.nn as nn
import torch.multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import cupy as cp
from numba import cuda, jit
import multiprocessing as mp_std

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

# CUDA kernels for fast stability computation
@cuda.jit
def compute_stability_metrics_cuda(positions, orientations, velocities, results):
    """CUDA kernel for parallel stability computation."""
    idx = cuda.grid(1)
    if idx < positions.shape[0]:
        # Compute stability metrics
        height = positions[idx, 2]
        tilt = max(abs(orientations[idx, 0]), abs(orientations[idx, 1]))
        vel_mag = (velocities[idx, 0]**2 + velocities[idx, 1]**2 + velocities[idx, 2]**2)**0.5
        
        # Stability criteria
        height_ok = height > 0.8
        tilt_ok = tilt < 0.3
        vel_ok = vel_mag < 1.0
        
        results[idx] = height_ok and tilt_ok and vel_ok

@jit(nopython=True)
def batch_joint_control_numba(joint_positions, target_positions, forces, gains):
    """Numba-optimized joint control computation."""
    control_signals = np.zeros_like(joint_positions)
    for i in range(len(joint_positions)):
        error = target_positions[i] - joint_positions[i]
        control_signals[i] = gains[i] * error
    return control_signals

class GPUAcceleratedParameterExplorer:
    """GPU-accelerated parameter exploration for massive parallel testing."""
    
    def __init__(self, robot_id: int, joint_categories: Dict[str, List[JointInfo]], use_gpu: bool = True):
        self.robot_id = robot_id
        self.joint_categories = joint_categories
        self.use_gpu = use_gpu and torch.cuda.is_available()
        self.device = torch.device('cuda' if self.use_gpu else 'cpu')
        self.results = []
        self.stable_configs = []
        
        # Initialize CUDA context if available
        if self.use_gpu:
            try:
                import cupy as cp
                self.cp = cp
                print(f"GPU acceleration enabled: {torch.cuda.get_device_name()}")
            except ImportError:
                print("CuPy not available, falling back to PyTorch CUDA")
                self.use_gpu = False
        
        # Pre-compile simulation environments for parallel execution
        self.sim_environments = []
        self.setup_parallel_environments()
    
    def setup_parallel_environments(self, num_envs: int = None):
        """Setup multiple PyBullet instances for parallel simulation."""
        if num_envs is None:
            num_envs = min(mp_std.cpu_count(), 16)  # Limit to prevent memory issues
        
        print(f"Setting up {num_envs} parallel simulation environments...")
        
        # Create multiple physics clients
        for i in range(num_envs):
            client_id = p.connect(p.DIRECT)  # Headless
            p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
            p.setGravity(0, 0, -9.81, physicsClientId=client_id)
            p.loadURDF("plane.urdf", physicsClientId=client_id)
            
            # Load robot in this environment
            robot_id = load_robot_description("valkyrie_description", physicsClientId=client_id)
            
            # Store environment info
            self.sim_environments.append({
                'client_id': client_id,
                'robot_id': robot_id,
                'busy': False
            })
    
    def generate_batch_parameters(self, batch_size: int = 1000) -> torch.Tensor:
        """Generate large batches of parameters on GPU."""
        if self.use_gpu:
            # Generate parameters directly on GPU
            device = self.device
            
            # Parameter ranges as tensors
            hip_pitch = torch.linspace(-0.3, 0.1, 100, device=device)
            knee_pitch = torch.linspace(0.0, 0.3, 80, device=device)
            ankle_pitch = torch.linspace(-0.1, 0.1, 50, device=device)
            shoulder_pitch = torch.linspace(-0.2, 0.5, 80, device=device)
            
            # Random sampling
            batch_params = torch.zeros(batch_size, 8, device=device)
            batch_params[:, 0] = hip_pitch[torch.randint(0, len(hip_pitch), (batch_size,), device=device)]
            batch_params[:, 1] = knee_pitch[torch.randint(0, len(knee_pitch), (batch_size,), device=device)]
            batch_params[:, 2] = ankle_pitch[torch.randint(0, len(ankle_pitch), (batch_size,), device=device)]
            batch_params[:, 3] = shoulder_pitch[torch.randint(0, len(shoulder_pitch), (batch_size,), device=device)]
            
            return batch_params
        else:
            # CPU fallback
            batch_params = np.random.rand(batch_size, 8)
            return torch.from_numpy(batch_params).to(self.device)
    
    def parallel_simulation_worker(self, env_info: Dict, param_batch: List[Dict]) -> List[StabilityMetrics]:
        """Worker function for parallel simulation."""
        client_id = env_info['client_id']
        robot_id = env_info['robot_id']
        results = []
        
        for params in param_batch:
            # Reset robot
            p.resetBasePositionAndOrientation(robot_id, [0, 0, 1.0], [0, 0, 0, 1], 
                                            physicsClientId=client_id)
            
            # Apply parameters
            stable_positions = self.config_to_joint_positions(params)
            
            # Quick stability test (reduced duration for speed)
            stability_score = 0
            test_steps = 500  # Reduced from 2000
            
            for step in range(test_steps):
                # Apply joint control
                for joint_id, target_pos in stable_positions.items():
                    p.setJointMotorControl2(
                        robot_id, joint_id, p.POSITION_CONTROL,
                        targetPosition=target_pos,
                        force=3000,
                        physicsClientId=client_id
                    )
                
                p.stepSimulation(physicsClientId=client_id)
                
                # Check stability every 10 steps
                if step % 10 == 0:
                    pos, orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=client_id)
                    euler = p.getEulerFromQuaternion(orn)
                    
                    if pos[2] > 0.8 and abs(euler[0]) < 0.3 and abs(euler[1]) < 0.3:
                        stability_score += 1
                    
                    if pos[2] < 0.5:  # Fell down
                        break
            
            # Create metrics
            metrics = StabilityMetrics(
                upright_time=stability_score / (test_steps / 10),
                success_rate=stability_score / (test_steps / 10),
                joint_config=params.copy()
            )
            
            results.append(metrics)
        
        return results
    
    def explore_parameter_space_parallel(self, max_iterations: int = 10000) -> List[StabilityMetrics]:
        """Massively parallel parameter exploration."""
        print(f"Starting GPU-accelerated exploration with {max_iterations} iterations...")
        
        # Generate parameter batches
        batch_size = 100
        num_batches = max_iterations // batch_size
        
        # Split work across environments
        num_envs = len(self.sim_environments)
        batches_per_env = num_batches // num_envs
        
        # Use ThreadPoolExecutor for I/O bound PyBullet operations
        with ThreadPoolExecutor(max_workers=num_envs) as executor:
            futures = []
            
            for env_idx, env_info in enumerate(self.sim_environments):
                # Generate parameter batch for this environment
                param_batches = []
                for batch_idx in range(batches_per_env):
                    batch_params = []
                    for _ in range(batch_size):
                        params = {
                            'hip_pitch': np.random.uniform(-0.3, 0.1),
                            'knee_pitch': np.random.uniform(0.0, 0.3),
                            'ankle_pitch': np.random.uniform(-0.1, 0.1),
                            'shoulder_pitch': np.random.uniform(-0.2, 0.5),
                            'elbow_pitch': np.random.uniform(0.0, 0.5),
                            'hip_roll': 0.0,
                            'ankle_roll': 0.0,
                            'torso_pitch': 0.0
                        }
                        batch_params.append(params)
                    param_batches.extend(batch_params)
                
                # Submit work to executor
                future = executor.submit(self.parallel_simulation_worker, env_info, param_batches)
                futures.append(future)
            
            # Collect results
            all_results = []
            for future in futures:
                results = future.result()
                all_results.extend(results)
                
                # Count stable configs
                stable_count = sum(1 for r in results if r.success_rate > 0.7)
                print(f"Batch complete: {stable_count}/{len(results)} stable configs")
        
        # Filter stable configurations
        self.results = all_results
        self.stable_configs = [r for r in all_results if r.success_rate > 0.7]
        
        print(f"Exploration complete: {len(self.stable_configs)} stable configs found")
        return self.results
    
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
    
    def save_results(self, filename: str = "gpu_stability_results.pkl"):
        """Save exploration results."""
        with open(filename, 'wb') as f:
            pickle.dump({
                'results': self.results,
                'stable_configs': self.stable_configs
            }, f)
        print(f"Results saved to {filename}")
    
    def cleanup(self):
        """Clean up physics clients."""
        for env_info in self.sim_environments:
            p.disconnect(physicsClientId=env_info['client_id'])
        print("Physics environments cleaned up")

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