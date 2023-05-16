import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi


robot = rtb.models.UR3()
robot.q = [0, -pi/2, pi/3, 0, pi/2, 0]

# robot_2 = rtb.models.UR3()
# robot_2.base = SE3.Tx(1)*robot_2.base
# robot_2.q = [0, -pi/2, pi/3, 0, pi/2, 0]

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)
# env.add(robot_2)

path_q = []

# move 1
final_pose_1 = SE3.Tz(-0.3)*robot.fkine(robot.q)
arrived = False
dt = 0.05
while not arrived:
    v, arrived = rtb.p_servo(robot.fkine(robot.q), final_pose_1, 0.8)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    path_q.append(robot.q)
    env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.1,0.5,0.1,1)))
    env.step(dt)

# move 2
final_pose_2 = SE3.Ty(-0.3)*robot.fkine(robot.q)
arrived = False
while not arrived:
    v, arrived = rtb.p_servo(robot.fkine(robot.q), final_pose_2, 0.8)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.5,0.1,0.1,1)))
    env.step(dt)

# move 3
final_pose_3 = SE3.Tz(0.3)*robot.fkine(robot.q)
arrived = False
while not arrived:
    v, arrived = rtb.p_servo(robot.fkine(robot.q), final_pose_3, 0.8)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.1,0.1,0.5,1)))
    env.step(dt)

# move 4
final_pose_4 = SE3.Ty(0.3)*robot.fkine(robot.q)
arrived = False
while not arrived:
    v, arrived = rtb.p_servo(robot.fkine(robot.q), final_pose_4, 0.8)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.5,0.1,0.1,1)))
    env.step(dt)
# for q in path_q:
#     robot_2.q = q
#     env.step(dt)

env.hold()
