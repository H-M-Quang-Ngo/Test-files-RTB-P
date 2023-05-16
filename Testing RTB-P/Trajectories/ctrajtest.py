import roboticstoolbox as rtb
import swift
from spatialmath import SE3
import numpy as np
from numpy import pi

# convert poses to joints path
def poses_to_joints(r,path):
    q_list = np.empty((len(path),6))
    q_list = np.empty((len(path),6))
    for i in range(len(path)):
        if i == 0:
            q_list[i] = r.ikine_LM(path[i],q0 = r.q).q
        else:
            q_list[i] = r.ikine_LM(path[i], q0=q_list[i-1]).q

        if (100*i/len(path))%10 == 0: 
            print(100*i/len(path),"%!")
        
    return q_list

# move robot function
def move_robot(r, q_list):    
    for q in q_list:
        r.q = q
        env.step(0.05)


# import robots and set position
r1 = rtb.models.UR3()
r1.q = [0, -pi/2, pi/3, 0, pi/2, 0]

# set up swift environment
env = swift.Swift()
env.launch(realtime=True)
env.add(r1)

# end pose
r1_start_T = r1.fkine(r1.q)
r1_final_1 = SE3.Tz(-0.2)*r1_start_T
r1_final_2 = SE3.Ty(-0.3)*r1_final_1
r1_final_3 = SE3.Tz(0.2)*r1_final_2
r1_final_4 = SE3.Ty(0.3)*r1_final_3


# trajectory ctraj
path_1 = rtb.ctraj(r1_start_T, r1_final_1, 50)
path_2 = rtb.ctraj(r1_final_1, r1_final_2, 50)
path_3 = rtb.ctraj(r1_final_2, r1_final_3, 50)
path_4 = rtb.ctraj(r1_final_3, r1_final_4, 50)


# joint trajectory
q_list1 = poses_to_joints(r1,path_1)
q_list2 = poses_to_joints(r1,path_2)
q_list3 = poses_to_joints(r1,path_3)
q_list4 = poses_to_joints(r1,path_4)

move_robot(r1,q_list1)
move_robot(r1,q_list2)
move_robot(r1,q_list3)
move_robot(r1,q_list4)

print('done')

# env.hold()
