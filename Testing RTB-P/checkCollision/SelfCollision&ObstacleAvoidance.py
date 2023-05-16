import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi

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

# check for self collision at a joint coordinate q, see self-collision text file
robo_clone = rtb.models.UR3() # make a clone for self collision check
def is_self_collision_UR3(q):
    global robo_clone
    robo_clone.q = q
    for i in reversed(range(8)[1:8]):
        stop_check = False
        if i == 2 :break
        for j in reversed(range(i - 1)[1:i-1]):
            is_collide = robo_clone.links[i].iscollided(shape = robo_clone.links[j].collision[0])
            if is_collide: 
                stop_check = True
                print("--COLLISION DETECT:",robo_clone.links[i].name," will collide with ",robo_clone.links[j].name)
                break    
        if stop_check: break
    return stop_check

# check collision with the obstacle
def is_collision_obstacle(robot,q,obstacle_list):
    is_collide = False
    for obstacle in obstacle_list:
        if robot.iscollided(q,obstacle,True):
            is_collide = True
            break
    return is_collide

#########################################
# move UR3/e robot with self-collsion detect incorporate
def move_robot(robot,qEnd):
    print("**TRY TO MOVE ALONG PATH:")
    global count
    global stop_loop
    # path = rtb.jtraj(q0 = robot.q, qf = qEnd,t=60)
    path = rtb.mtraj(tfunc=rtb.trapezoidal,q0 = robot.q,qf=qEnd,t =50)
    path_finished = True
    for i in range(len(path)):
        # check each link
        touch_ground= is_touch_ground(robot,path.q[i])
        touch_self = is_self_collision_UR3(path.q[i])
        touch_obstacle = is_collision_obstacle(robot,path.q[i],obstacle_list) 
        if not touch_ground and not touch_self and not touch_obstacle:
            #print('Move normal!')
            robot.q = path.q[i]
            env.step(0.05)
        else:
            if touch_ground: print(count,'.May touch ground at step', i ,', another path!')
            if touch_obstacle:  print(count,'.May touch obstacle at step', i ,', another path!')
            if touch_obstacle: print(count,'.Get self collision at step', i ,', another path!')
            
            count = count + 1
            path_finished = False 
            if i >= 2: move_back(robot,path.q[int(0.8*i):i])
            elif i==1: stop_loop = True
            break               
    print("Path finished:", path_finished)
    return path_finished

# function to move robot back
count = 1
def move_back(robot, path):
    print('Unsafe movement!')
    for q in reversed(path):
            robot.q = q
            env.step(0.07)

#############################################
# RUN #
env = swift.Swift()
env.launch(realtime= "True")

# Make a robot and collision objects and add to Swift
robot = rtb.models.UR3()
robot.q[1] = -np.pi/2 

obstacle_list = []
obstacle_list.append(collisionObj.Sphere(radius = 0.05, pose = SE3(0.2,0.2,0.3), color = (0.1,0.1,0.5,1)))
obstacle_list.append(collisionObj.Sphere(radius = 0.05, pose = SE3(-0.2,0.2,0.5), color = (0.1,0.4,0.5,1)))
obstacle_list.append(collisionObj.Cuboid(scale = [0.08,0.08,0.08], pose = SE3(0.2,-0.4,0.5), color = (0.3,0.2,0.5,1)))
obstacle_list.append(collisionObj.Cuboid(scale = [0.1,0.1,0.1], pose = SE3(0.2,-0.3,0.3), color = (0.5,0.2,0.2,1)))

env.add(robot)

for obstacle in obstacle_list:
    env.add(obstacle)

# run the robot randomly
stop_loop = False
while True:
    # if stop_loop:
    #     print("Can't recover. Stop moving!")
    #     break

    q_rand = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])
    self_co = is_self_collision_UR3(q_rand)
    ground_co = is_touch_ground(robot,q_rand)
    ob_co = is_collision_obstacle(robot,q_rand,obstacle_list)
    print("---Self collision: ",self_co,"; Ground collision:",ground_co,"; Obstacle collision:", ob_co)    
    
    if self_co or ground_co or ob_co:
        print("->Nonreachable goal! Regenerate final q!")
        continue
    else: 
        move_robot(robot,q_rand)
    print("----\n\n")

env.hold()