import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi
import time
import threading

#### ROBOT MOVE #####################################################
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
robo_clone = rtb.models.UR3() #make a clone for self collision check
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

# check collision with the obstacle list
def is_collision_obstacle(robot,q,obstacle_list):
    is_collide = False
    for obstacle in obstacle_list:
        if robot.iscollided(q,obstacle,True):
            is_collide = True
            break
    return is_collide

# move UR3/e robot with self-collsion detect incorporate
count = 1
def move_robot(robot,q_end,obstacle_list):
    print("**TRY TO MOVE ALONG PATH:")
    global stop_loop
    global count
    # path = rtb.jtraj(q0 = robot.q, qf = q_end,t=60)
    path = rtb.mtraj(tfunc=rtb.trapezoidal,q0 = robot.q,qf=q_end,t =50)
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
            if touch_self: print(count,'.Get self collision at step', i ,', another path!')
            
            count = count + 1
            path_finished = False 
            if i >= 2: move_back(robot,path.q[int(0.8*i):i])
            elif i==1: stop_loop = True
            if i == 1 : stop_loop = True
            break
                 
    print("Path finished:", path_finished)
    return path_finished

# function to move robot back
def move_back(robot, path):
    print('Unsafe movement!')
    for q in reversed(path):
            robot.q = q
            env.step(0.07)


# Randomly move the robot
stop_loop = False
def run_robot_random(obstacle_list):
    while True:
        if stop_loop:
            print("Can't recover. Stop moving!")
            break
       
        q_rand = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])
        
        selfCo = is_self_collision_UR3(q_rand)
        groundCo = is_touch_ground(robot,q_rand)
        # obCo = is_collision_obstacle(robot,q_rand,obstacle_list)
        
        print("---Self collision: ",selfCo,"; Ground collision:",groundCo) 
        if selfCo or groundCo:
            print("->Goal may touch ground or lead to self-collision! Regenerate final q!")
            continue
        else: 
            move_robot(robot,q_rand,obstacle_list)
        print("----\n\n")

#### OBJECT MOVE ####################################
# create a point lists for a circle
def circle(radius = 0.3,distance_z = 0.5):
    # Input the radius of the circle
    # Input the distance from the z-axis

    # Define the number of points to generate
    num_points = 300

    # Generate an array of angles from 0 to 2*pi with a step of 2*pi/num_points
    angles = np.linspace(0, 2*np.pi, num_points, endpoint=False)

    x = radius * np.cos(angles) 
    y = radius * np.sin(angles)
    z = distance_z * np.ones(num_points)

    # Create a list of [x, y, z] points
    points = np.stack([x, y, z], axis=1)

    # return the point list
    return points

# draw that circle
def circle_display(radius,distance_z):
    points = circle(radius,distance_z)  
    for p in points:
        dot = collisionObj.Sphere(radius=0.003,pose = SE3(p),color = (0.5,0.2,0.2,1))
        env.add(dot)

# move the input obstacle along the circular trajectory
def circle_move(obstacle,radius,distance_z):
    trajectory = circle(radius,distance_z)
    while True:
        for p in trajectory:
            obstacle.T = SE3(p)
            env.step(0.05)

#### RUN #######################################################################################
osbtacleList = []
obstacle = collisionObj.Cuboid(scale = [0.1,0.1,0.1], pose = SE3(0.2,0.3,0.6), color = (0.2,0.2,0.5,1))
osbtacleList.append(obstacle)

robot = rtb.models.UR3() 
robot.q[1] = -pi/2 

env = swift.Swift()
env.launch(realtime= "True")
env.add(obstacle)
env.add(robot)

circle_display(radius=0.3,distance_z=0.4)

t1 = threading.Thread(target = run_robot_random, args = (osbtacleList,))
t2 = threading.Thread(target = circle_move, args = (obstacle,0.3,0.4,))

t1.start()
t2.start()

t1.join()
t2.join()

print("All threads finished!")