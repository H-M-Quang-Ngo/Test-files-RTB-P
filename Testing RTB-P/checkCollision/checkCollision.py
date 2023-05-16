import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import spatialgeometry.geom as collisionObj

robot = rtb.models.UR3()
robot.q[1] = -np.pi/2 
obstacle = collisionObj.Cuboid(scale = [0.1,0.1,0.1], pose = SE3(0.3,0.1,0.5), color = (0.8,0.5,0.5,1))

env = swift.Swift()
env.launch(realtime="True")
env.add(robot)
env.add(obstacle)

iscollision = robot.iscollided(robot.q, shape = obstacle, skip=True)
print(iscollision)

iscollision = robot.links[0].iscollided(shape = obstacle, skip=True)
print(iscollision)

env.hold()