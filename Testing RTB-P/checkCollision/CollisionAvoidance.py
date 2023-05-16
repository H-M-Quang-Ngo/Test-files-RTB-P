import swift
import roboticstoolbox as rtb
import numpy as np
from numpy import pi
from spatialmath import SE3
import random
import spatialgeometry.geom as collisionObj

# return an list include <class 'spatialmath.pose3d.SE3'> object as transform of each link given joint state
def get_link_transform(robot,q):
    T = [] # Tranforms array of link
    for i in range(len(q)):
        T.append(robot.fkine(q,end = robot.links[i].name)) 
    return T

# check whether the robot touches the ground
def is_touch_ground(robot,q):
    T = get_link_transform(robot,q)
    height = [T[i].A[2,3] for i in range(2,6)]
    if min(height) > 0.1:
        return False
    else: 
        return True

# check collision with the obstacle
def is_collision(robot,q,obstacle):
    if robot.iscollided(q,obstacle,True):
        return True
    else:
        return False

# function to move robot back
count = 1
def move_back(robot, qpath):
    print('Unsafe movement!')
    for q in reversed(qpath):
            robot.q = q
            env.step(0.07)
    

def move_robot(robot, q_end):
    global count
    #path =rtb.jtraj(robot.q,q_end,t=50)
    path = rtb.mtraj(tfunc = rtb.trapezoidal,q0 = robot.q, qf = q_end,t=60)
    for i in range(len(path)):
        # check each link
        touch_ground= is_touch_ground(robot,path.q[i])
        touch_object = is_collision(robot,path.q[i],obstacle)
        if not touch_ground and not touch_object:
            #print('Move normal!')
            robot.q = path.q[i]
            env.step(0.05)
        else:
            if touch_ground: print(count,'.May touch ground, another path!')
            if touch_object: print(count,'.May touch obstacle, another path!')

            count = count + 1
            if i >= 2: move_back(robot,path.q[int(0.8*i):i])
            break

env = swift.Swift()
env.launch(realtime= "True")

# Make a robot and a collision object add to Swift
robot = rtb.models.UR3()
robot.q[1] = -np.pi/2 
obstacle = collisionObj.Sphere(radius = 0.08, pose = SE3(0.3,0.1,0.5), color = (0.8,0.5,0.5,1))

env.add(robot)
env.add(obstacle)

# run the robot randomly
while True:
    q_rand = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])

    if is_collision(robot,q_rand,obstacle) or is_touch_ground(robot,q_rand): continue
    
    move_robot(robot,q_rand)

# q_end = robot.qr
# path = rtb.jtraj(robot.q, q_end,50)

# for q in path.q:
#     robot.q = q
#     env.step(0.05)

# move_back(robot,path.q[35:49])
