from dataclasses import dataclass
from typing import Tuple

@dataclass
class SimulationConfig:
    """Configuration parameters for the simulation."""
    gravity: float = -9.81
    robot_height: float = 1.0
    table_height: float = 0.75
    stabilization_steps: int = 2000
    simulation_rate: float = 240.0
    wave_frequency: float = 0.2
    wave_amplitude: float = 0.05
    
    # Table positioning
    table_position: Tuple[float, float, float] = (1.0, 0.0, 0.75)
    
    # Physics parameters
    lateral_friction: float = 1.0
    spinning_friction: float = 0.1
    rolling_friction: float = 0.1
    
    # Bottle parameters
    bottle_mass: float = 0.5
    bottle_lateral_friction: float = 0.5
    
    # Joint control parameters - UPDATED FOR BETTER STABILITY
    joint_force: float = 2000  # Increased from 500
    position_gain: float = 0.8  # Increased from 0.3
    velocity_gain: float = 0.3  # Increased from 0.1
    max_joint_velocity: float = 1.0
    
    # Damping parameters
    linear_damping: float = 0.1
    angular_damping: float = 0.1