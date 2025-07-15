import pybullet as p
import pybullet_data
from .simulation_config import SimulationConfig

def initialize_physics_engine(config: SimulationConfig) -> None:
    """Initialize PyBullet physics engine."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, config.gravity)

def load_ground_plane() -> int:
    """Load ground plane with friction."""
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, 
                    lateralFriction=1.0, 
                    spinningFriction=0.1, 
                    rollingFriction=0.1)
    return plane_id

def step_simulation(config: SimulationConfig) -> None:
    """Step simulation once."""
    p.stepSimulation()

def disconnect_physics() -> None:
    """Disconnect from physics engine."""
    p.disconnect()