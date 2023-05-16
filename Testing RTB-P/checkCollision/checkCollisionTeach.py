import swift
import roboticstoolbox as rtb
import numpy as np
import time
from spatialmath import SE3
import spatialgeometry.geom as collisionObj


env = swift.Swift()
env.launch()

# Make a robot and a collision object add to Swift
robot = rtb.models.UR3()
robot.q[1] = -np.pi/2 
obstacle = collisionObj.Sphere(radius = 0.08, pose = SE3(0.3,0.1,0.5), color = (0.8,0.5,0.5,1))

env.add(robot)
env.add(obstacle)

# This is our callback function from the sliders in Swift which set
# the joint angles of our robot to the value of the sliders
def set_joint(j, value):
    robot.q[j] = np.deg2rad(float(value))


# Loop through each link in the robot and if it is a variable joint,
# add a slider to Swift to control it
j = 0
for link in robot.links:
    if link.isjoint:

        # We use a lambda as the callback function from Swift
        # j=j is used to set the value of j rather than the variable j
        # We use the HTML unicode format for the degree sign in the unit arg
        env.add(
            swift.Slider(
                lambda x, j=j: set_joint(j, x),
                min=np.round(np.rad2deg(link.qlim[0]), 2),
                max=np.round(np.rad2deg(link.qlim[1]), 2),
                step=1,
                value=np.round(np.rad2deg(robot.q[j]), 2),
                desc="robot Joint " + str(j),
                unit="&#176;",
            )
        )

        j += 1


qref = [None]*robot.n
count = 1
def update_collision_status():  
    global qref
    global count
    if robot.iscollided(robot.q,obstacle,True):
        if not(np.array_equal(robot.q, qref)):
            print(count,".Collision detected!")
            qref = robot.q
            count = count + 1
        else: pass
    else:
        qref = [None]*robot.n

while True:
    # Process the event queue from Swift, this invokes the callback functions
    # from the sliders if the slider value was changed
    # env.process_events()

    # Update the environment with the new robot pose
    env.step(0)
    update_collision_status()
    time.sleep(0.01)