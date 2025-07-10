import pybullet as p
import pybullet_data

# Test PyBullet installation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("r2d2.urdf")
p.setGravity(0, 0, -9.8)

for i in range(1000):
    p.stepSimulation()

print("PyBullet setup successful!")
