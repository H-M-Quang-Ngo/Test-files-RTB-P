from ur3module import *

def gen_path(robot,q_goal,obstacle_list = None):

    tolerance = 0.1 # tolerance value (in rads) for maximum difference between a current joint state and the random joint state
    current_joint_state = robot.q
    
    iteration_joint = 0
    iteration_path = 0
    
    Kp = 0.1 # a proportional gain constant
    final_path = []

    print("--GENERATING PATH!")
    while iteration_path < 500:
        # path from current joint state to goal
        # path = rtb.mtraj(tfunc = rtb.trapezoidal, q0 = current_joint_state, qf = q_goal, t = 60)
        path = rtb.jtraj(q0=current_joint_state,qf=q_goal,t=60)
        path_valid, all_valid = get_valid_path(robot,path,obstacle_list)
        final_path += path_valid

        if not all_valid:
            if bool(path_valid):
                current_joint_state = path_valid[-1]

            while iteration_joint < 50:
                iteration_joint += 1
                
                # generate a random joint state
                error = q_goal - current_joint_state # error between the current joint state and the goal joint state
                random_joint_state = current_joint_state + Kp * error+ np.random.normal(0, tolerance, robot.n)

                # clip joint angles to valid range
                random_joint_state = np.clip(random_joint_state, -np.pi, np.pi)
            
                if not is_joint_valid(robot,random_joint_state,obstacle_list)[0]: continue 
                else: # create a path to connect the current joint state with the random joint state
                    print(">Check subpath:")
                    subpath = rtb.mtraj(tfunc = rtb.trapezoidal, q0 = current_joint_state, qf = random_joint_state, t = 15)
                    subpath_valid, all_subpath_valid = get_valid_path(robot,subpath,obstacle_list)
                    
                    if all_subpath_valid:
                        final_path += subpath_valid
                        current_joint_state = random_joint_state
                        iteration_joint = 0    
                        break
                    else: continue
        else: 
            break

        iteration_path += 1

        print("STEP:",iteration_path)
        if np.linalg.norm(final_path[-1]- q_goal) <= 0.05: break
        
    if np.linalg.norm(final_path[-1]- q_goal) <= 0.05: 
        print("--GENERATE SUCCESSFULL!")
        success = True
    else: 
        print("--GENERATE NOT SUCCESSFULL!")
        success = False
        
    return final_path,success


def gen_joints(robot,q_goal,obstacle_list = None):

    tolerance = 0.1 # tolerance value (in rads) for maximum difference between a current joint state and the random joint state
    current_joint_state = robot.q
    
    iteration_joint = 0
    iteration_path = 0
    iteration_joint_connect = 0
    
    Kp = 0.15 # a proportional gain constant
    # final_path = []
    sub_path = 0
    all_joints = []
    # print("--GENERATING PATH!")
    while iteration_path < 500:
        # path from current joint state to goal
        # path = rtb.mtraj(tfunc = rtb.trapezoidal, q0 = current_joint_state, qf = q_goal, t = 60)
        path = rtb.jtraj(q0=current_joint_state,qf=q_goal,t=60)
        path_valid, all_valid = get_valid_path(robot,path,obstacle_list)
        # final_path += path_valid
        sub_path+=1

        if bool(path_valid):
            all_joints.append(path_valid[0])
            all_joints.append(path_valid[-1])
            
        # print("--->Subs:",sub_path)

        if not all_valid:
            if bool(path_valid):
                current_joint_state = path_valid[-1]

            while iteration_joint < 50 and iteration_joint_connect < 500:
                iteration_joint += 1
                iteration_joint_connect += 1

                # generate a random joint state
                error = q_goal - current_joint_state # error between the current joint state and the goal joint state
                random_joint_state = current_joint_state + Kp * error+ np.random.normal(0, tolerance, robot.n)

                # clip joint angles to valid range
                random_joint_state = np.clip(random_joint_state, -np.pi, np.pi)
            
                if not is_joint_valid(robot,random_joint_state,obstacle_list)[0]: continue 
                else: # check whether the joint can be connected from the current joint state 
                    subpath = rtb.mtraj(tfunc = rtb.trapezoidal, q0 = current_joint_state, qf = random_joint_state, t = 15)
                    subpath_valid, all_subpath_valid = get_valid_path(robot,subpath,obstacle_list)
                    if all_subpath_valid:
                        current_joint_state = random_joint_state
                        iteration_joint = 0    
                        break
                    else: continue
        else: 
            break

        iteration_path += 1

        # print("STEP:",iteration_path)
        if bool(path_valid): 
            if np.linalg.norm(path_valid[-1]- q_goal) <= 0.05: break
        
    if bool(all_joints): 
        if np.linalg.norm(all_joints[-1]- q_goal) <= 0.05: 
            print("--GENERATE SUCCESSFULL!")
            success = True
        else: 
            print("--GENERATE NOT SUCCESSFULL!")
            success = False
    else: success = False    
    
    return all_joints,success

def gen_path_2(robot,q_goal,obstacle_list = None, hard = False):
    path_list = []
    list_count = 0
    gen_count = 0

    while list_count < 5:
        # if gen_count > 50: 
        #     break
        print("*GENERATE joint list ",gen_count,":")
        joints, success = gen_joints(robot,q_goal,obstacle_list)
        if success:
            path = rtb.mstraj(viapoints=np.array([joint for joint in joints]),dt=0.02,tacc=0.2,qdmax = np.pi)
            if get_valid_path(robot,path,obstacle_list)[1] :
                path_list.append(path.q)
                list_count += 1
                if hard: break #return when there is a valid path found
        gen_count +=1
        print("----------------------")
    
    shortest_path = []
    if bool(path_list):
        shortest_path = min(path_list, key= len)
    else: print("CANT FIND PATH")
    return shortest_path





def shortest_path(robot,q_goal,obstacle_list = None):
    path_list = []
    for i in range(2):
        print(">> GENERATE PATH ",i,":")
        path, success = gen_path(robot,q_goal,obstacle_list)
        if success: path_list.append(path)
        print("----------------------")
    shortest_path = min(path_list, key= len)
    return shortest_path



#TEST
# def gen_path_2(robot,q_goal,obstacle_list = None):

#     tolerance = 0.1 # tolerance value (in rads) for maximum difference between a current joint state and the random joint state
#     current_joint_state = robot.q
    
#     iteration_joint = 0
#     iteration_path = 0
    
#     Kp = 0.6 # a proportional gain constant
#     joint_list =[]

#     success = False

#     print("--GENERATING PATH:")
#     while iteration_joint < 1000:
            
#         iteration_joint += 1
#         print("Step:", iteration_joint)

#         # generate a random joint state
#         error = q_goal - current_joint_state # error between the current joint state and the goal joint state
#         random_joint_state = current_joint_state + Kp * error+ np.random.normal(0, tolerance, robot.n)

#         # clip joint angles to valid range
#         random_joint_state = np.clip(random_joint_state, -np.pi, np.pi)
        
#         if not is_joint_valid(robot,random_joint_state,obstacle_list)[0]: continue 
#         else: joint_list.append(random_joint_state)

#         if np.linalg.norm(random_joint_state- q_goal) <= 0.05: 
#             print("--GENERATE SUCCESSFULL!")
#             success = True
#             break
    
#     if not success: print("GENERATE NOT SUCCESSFULL!")   
#     return joint_list,success




# robot = rtb.models.UR3()
def create_point_cloud(robot):
    #array of joint limit for each robot joint

    qlim = robot.qlim
    # print(qlim)
    cloud = []
    print(qlim[0,0])
    #step through each joint 
    for i in np.arange(qlim[0,0], qlim[1,0], pi/4):
        for j in np.arange(qlim[0,1], qlim[1,1], pi/4):
            for k in np.arange(qlim[0,2], qlim[1,2], pi/4):
                for l in np.arange(qlim[0,3], qlim[1,3], pi/4):
                    for m in np.arange(qlim[0,4], qlim[1,4], pi/4):
                        ee_pose = robot.fkine([i,j,k,l,m,0]).A
                        point = list(ee_pose[0:3,3])
                        if point[2] <= 0 : continue
                        cloud.append(point)
    
    print(cloud)
    return cloud

def show_path(robot,path,env):
    for q in path:
        env.add(collisionObj.Sphere(radius=0.002, pose = robot.fkine(q),color = (0.5,0.1,0.1,1)))

def show_points(point_list, env):
    count = 0
    for point in point_list:
        # if count%1:
        #     env.add(collisionObj.Sphere(radius=0.003, pose = SE3(point),color = (0.0,0.5,0.1,1)))
        # count+=1
        env.add(collisionObj.Sphere(radius=0.01, pose = SE3(point),color = (0.0,0.5,0.1,1)))
   
    print("SHOW DONE!")
