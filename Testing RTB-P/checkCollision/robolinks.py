import roboticstoolbox as rtb
import swift

robot = rtb.models.UR3()
i = 0

for link in robot.links:
    print(i,". ",link)
    print("\n")
    i+=1

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

env.hold()
# def MoveCloneRobot(robot,qEnd):
#     path = rtb.jtraj(robot.q,qEnd,50)
#     for q in path.q:
#         robot.q = q
#         env.step(0.05)

# #robot clone
# robotClone = rtb.models.UR3()
# robotClone.q = robot.getj()

# #simulation env
# env = swift.Swift()
# env.launch(realtime=True)
# env.add(robotClone)

# #destination
# T = SE3(0.3, 0.2, 0.1) * SE3.Rx(-pi/2) * SE3.Ry(pi/4)
# qEnd = robotClone.ikine_LM(T,q0=robot.q).q
