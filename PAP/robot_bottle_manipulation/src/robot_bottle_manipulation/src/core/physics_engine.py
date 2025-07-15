import pybullet as p
import pybullet_data
from .simulation_config import SimulationConfig

class PhysicsEngine:
    """Handles PyBullet physics engine initialization and configuration."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
        self.physics_client = None
        
    def initialize(self) -> None:
        """Initialize PyBullet physics engine with GUI."""
        self.physics_client = p.connect(p.DIRECT) 
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, self.config.gravity)
        
        # Load ground plane
        plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(
            plane_id, -1,
            lateralFriction=self.config.lateral_friction,
            spinningFriction=self.config.spinning_friction,
            rollingFriction=self.config.rolling_friction
        )
        
        # Set simulation timestep
        p.setTimeStep(1.0 / self.config.simulation_rate)
        
    def step(self) -> None:
        """Step the physics simulation."""
        p.stepSimulation()
        
    def disconnect(self) -> None:
        """Disconnect from physics engine."""
        if self.physics_client is not None:
            p.disconnect(self.physics_client)
            self.physics_client = None