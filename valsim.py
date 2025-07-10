import pybullet as p
import pybullet_data
import time
from robot_descriptions.loaders.pybullet import load_robot_description

# Start PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load ground plane
p.loadURDF("plane.urdf")

# Load Valkyrie
robot_id = load_robot_description("valkyrie_description")

# Run simulation
while True:
    p.stepSimulation()
    time.sleep(1./240.)