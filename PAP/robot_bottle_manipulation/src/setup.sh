#!/bin/bash

# Create project structure
echo "Creating project structure..."

# Create main directories
mkdir -p robot_bottle_manipulation/src/core
mkdir -p robot_bottle_manipulation/src/robot
mkdir -p robot_bottle_manipulation/src/environment
mkdir -p robot_bottle_manipulation/src/manipulation
mkdir -p robot_bottle_manipulation/src/rl
mkdir -p robot_bottle_manipulation/assets

# Create __init__.py files
touch robot_bottle_manipulation/src/__init__.py
touch robot_bottle_manipulation/src/core/__init__.py
touch robot_bottle_manipulation/src/robot/__init__.py
touch robot_bottle_manipulation/src/environment/__init__.py
touch robot_bottle_manipulation/src/manipulation/__init__.py
touch robot_bottle_manipulation/src/rl/__init__.py

# Create core files
touch robot_bottle_manipulation/src/core/physics_engine.py
touch robot_bottle_manipulation/src/core/simulation_config.py

# Create robot files
touch robot_bottle_manipulation/src/robot/robot_loader.py
touch robot_bottle_manipulation/src/robot/joint_controller.py
touch robot_bottle_manipulation/src/robot/robot_kinematics.py

# Create environment files
touch robot_bottle_manipulation/src/environment/scene_loader.py
touch robot_bottle_manipulation/src/environment/object_tracker.py

# Create manipulation files
touch robot_bottle_manipulation/src/manipulation/motion_planner.py
touch robot_bottle_manipulation/src/manipulation/grasp_planner.py

# Create RL files
touch robot_bottle_manipulation/src/rl/environment_wrapper.py
touch robot_bottle_manipulation/src/rl/reward_calculator.py

# Create main files
touch robot_bottle_manipulation/main.py
touch robot_bottle_manipulation/requirements.txt

echo "Project structure created successfully!"
echo "Directory structure:"
find robot_bottle_manipulation -type d | sort
echo ""
echo "Files created:"
find robot_bottle_manipulation -name "*.py" | sort