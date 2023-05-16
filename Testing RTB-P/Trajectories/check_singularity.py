import sys
sys.path.append('../Modules')
from ur3module import *

# create robot and environment
robot = rtb.models.UR3()
q_deg = [31.24, -50.72, -5.51, -140.39, -38.79, 200.26]
robot.q = [x*pi/180 for x in q_deg]

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

# manipulability threshold
thresh_hold = 0.011

# pose robot try to reach
final_pose_1 = robot.fkine(robot.q)*SE3.Tx(-0.25)

# RMRC implementation with singularity check
arrived = False
dt = 0.05
while not arrived:    
    v, arrived = rtb.p_servo(robot.fkine(robot.q), final_pose_1, 0.5)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.1,0.5,0.1,1)))
    env.step(dt)

    maniplty = robot.manipulability(robot.q,axes='trans')
    print(">",maniplty,'<\n')

    if maniplty <= thresh_hold: 
        print("Close to singularity!")
        break

# if the singularity happen, try to finish the path with other method
if not arrived:
    q_end = solve_for_valid_ik(robot,obj_pose=final_pose_1)
    move_robot_insurance(robot,q_end,env)

env.hold()


