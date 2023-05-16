import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi

# def Move_J(robot, q):
#     # print('Unsafe movement!')
#     qpath = rtb.jtraj(robot.q,q,50)
#     for q in qpath.q:
#         robot.q = q
#         env.step(0.05)

robot = rtb.models.UR3()
# print(type(robot.links[0]))
# a = type(robot.links[0].collision)
# print(a)

env = swift.Swift()
env.launch(realtime=True)
# env.add(robot)
col =[]

for link in robot.links:
    col.append(link.collision)

# print(col)
# print(len(robot.links)) #11
# print(type(col[1][0]))
# print(col[1][0].T)
print(type(col[1][0].T))
z = 0
for col_obj in col:
    if len(col_obj)>=1:
        for obj in col_obj:
            # obj.T = SE3(0,0,z).A
            print(type(obj))
            env.add(obj)

# while True:
#     q_rand = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])   
#     Move_J(robot,q_rand)

env.hold()