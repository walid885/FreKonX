from dataclasses import dataclass
from typing import Tuple

@dataclass
class SimulationConfig:
    """Configuration parameters for the simulation."""
    gravity: float = -9.81
    simulation_rate: float = 240.0
    stabilization_steps: int = 2000
    
    # Robot configuration
    robot_height: float = 1.0
    robot_description: str = "valkyrie_description"
    robot_position: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    
    # Environment configuration
    table_height: float = 0.75
    table_position: Tuple[float, float, float] = (1.0, 0.0, 0.75)
    
    # Motion parameters
    wave_frequency: float = 0.2
    wave_amplitude: float = 0.05
    
    # Control parameters
    joint_force: float = 500.0
    position_gain: float = 0.3
    velocity_gain: float = 0.1

@dataclass
class JointInfo:
    """Information about a robot joint."""
    id: int
    name: str
    type: int
    limits: Tuple[float, float]
    current_position: float

@dataclass
class SceneObjects:
    """Container for simulation object IDs."""
    robot_id: int
    table_bottle_id: int
    bottle_link_id: int = 5