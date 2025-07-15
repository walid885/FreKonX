# Robot Bottle Manipulation Simulation

A modular PyBullet-based simulation for robot bottle manipulation tasks with reinforcement learning support.

## Project Structure

```
robot_bottle_manipulation/
├── src/
│   ├── core/                    # Core simulation components
│   │   ├── physics_engine.py    # PyBullet physics engine wrapper
│   │   └── simulation_config.py # Configuration parameters
│   ├── robot/                   # Robot control and kinematics
│   │   ├── robot_loader.py      # Robot loading and initialization
│   │   ├── joint_controller.py  # Joint control and information
│   │   └── robot_kinematics.py  # Robot kinematics calculations
│   ├── environment/             # Environment and scene management
│   │   ├── scene_loader.py      # Scene loading and URDF creation
│   │   └── object_tracker.py    # Object position tracking
│   ├── manipulation/            # Motion planning and grasping
│   │   ├── motion_planner.py    # Motion planning algorithms
│   │   └── grasp_planner.py     # Grasping motion planning
│   └── rl/                      # Reinforcement learning components
│       ├── environment_wrapper.py # RL environment wrapper
│       └── reward_calculator.py   # Reward calculation
├── assets/                      # Asset files
│   └── table_bottle_scene.urdf
├── main.py                      # Main simulation entry point
└── requirements.txt             # Python dependencies
```

## Features

- **Modular Architecture**: Clean separation of concerns with dedicated modules for physics, robot control, environment, and RL
- **Robot Control**: Complete joint control system with safety limits and motion planning
- **Scene Management**: Automatic URDF generation for table and bottle scenes
- **Object Tracking**: Real-time tracking of bottle position relative to robot
- **Motion Planning**: Basic motion planning for reaching and grasping tasks
- **RL Integration**: Environment wrapper and reward calculation for reinforcement learning
- **Configurable**: Centralized configuration system for easy parameter tuning

## Installation

1. Create the project structure:
```bash
chmod +x setup_project.sh
./setup_project.sh
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

### Basic Simulation

Run the main simulation:
```bash
python main.py
```

This will:
1. Initialize the physics engine
2. Load the Valkyrie robot
3. Create and load the table-bottle scene
4. Stabilize the robot
5. Run the main simulation loop with motion planning

### Using Individual Components

```python
from src.core import PhysicsEngine, SimulationConfig
from src.robot import RobotLoader, JointController
from src.environment import SceneLoader, ObjectTracker

# Initialize components
config = SimulationConfig()
physics = PhysicsEngine(config)
physics.initialize()

# Load robot and scene
robot_loader = RobotLoader(config)
robot_id = robot_loader.load_robot("valkyrie_description", [0, 0, 1.0])

# Create scene
scene_loader = SceneLoader(config)
urdf_path = scene_loader.create_urdf_file()
table_id = scene_loader.load_table_bottle_scene(urdf_path)

# Track objects
tracker = ObjectTracker(table_id)
bottle_pos = tracker.get_bottle_position()
```

### Reinforcement Learning

```python
from src.rl import RLEnvironmentWrapper, RewardCalculator

# Create RL environment
rl_env = RLEnvironmentWrapper(config, robot_id, joint_controller, object_tracker)

# Training loop
observation = rl_env.reset()
for step in range(1000):
    action = your_policy(observation)  # Your RL policy
    observation, reward, done, info = rl_env.step(action)
    if done:
        observation = rl_env.reset()
```

## Configuration

All simulation parameters are centralized in `SimulationConfig`:

```python
@dataclass
class SimulationConfig:
    gravity: float = -9.81
    robot_height: float = 1.0
    table_height: float = 0.75
    simulation_rate: float = 240.0
    # ... more parameters
```

## Key Components

### PhysicsEngine
- Initializes PyBullet physics simulation
- Handles simulation stepping and cleanup
- Configurable physics parameters

### RobotLoader
- Loads robots from robot_descriptions
- Sets up robot dynamics and properties
- Provides robot information

### JointController
- Manages robot joint control
- Filters joints by type and name
- Provides safe position control

### ObjectTracker
- Tracks bottle position and orientation
- Calculates relative positions
- Provides object state information

### MotionPlanner
- Plans reaching motions for manipulation
- Handles robot balance and stability
- Provides motion interpolation

### GraspPlanner
- Plans approach and grasping motions
- Calculates grasp feasibility
- Provides inverse kinematics (simplified)

## Extending the System

### Adding New Robots
1. Add robot description to `robot_loader.py`
2. Update joint filtering in `joint_controller.py`
3. Adjust motion planning in `motion_planner.py`

### Adding New Objects
1. Update URDF in `scene_loader.py`
2. Add tracking methods in `object_tracker.py`
3. Update reward calculation in `reward_calculator.py`

### Custom Motion Planning
1. Extend `MotionPlanner` class
2. Implement custom planning algorithms
3. Add new motion primitives

### Custom Rewards
1. Extend `RewardCalculator` class
2. Add new reward components
3. Update environment wrapper

## Dependencies

- PyBullet 3.2.5
- robot-descriptions 1.12.0
- NumPy >= 1.21.0
- dataclasses (Python 3.7+)

## License

This project is provided as-is for educational and research purposes.