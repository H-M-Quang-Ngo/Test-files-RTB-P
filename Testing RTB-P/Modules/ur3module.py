import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi

def get_link_transform(robot,q)->bool:
    """
    Get transform for each link of the robot

    :param robot: Manipulator 
    :type robot: URDF model
    :param q: joint state 
    :type q: array like
    :return: list of transform for each link at the given joint state
    :rtype: list of SE3 transforms

    Computes SE3 object for each link of the input robot 
    """
    transform_list = [] # Tranforms array of link
    for i in range(len(q)+1): # one is added because link 0 is the world frame
        transform_list.append(robot.fkine(q,end = robot.links[i].name)) 
    return transform_list

def is_touch_ground(robot,q)->bool:
    """
    Check whether any link of the input robot lower than the given certain height from the base,
    at the given joint state. Cannot be used without importing 'get_link_transform'

    :param robot: Manipulator 
    :type robot: URDF model
    :param q: joint state 
    :type q: array like
    :return: whether the ground touch happens for any link in robot at the given joint state
    :rtype: bool value
 
    """
    T = get_link_transform(robot,q)
    height = [T[i].A[2,3] for i in range(2,robot.n+1)]
    # print(height, T[1].A[2,3])
    if min(height) > T[1].A[2,3] + 0.1: # at position 1 is the robot base
        return False
    else: 
        return True

def is_collision_obstacle(robot,q,obstacle_list,is_swift=True)->bool:
    """
    Check whether any self collision happens in an UR3 robot. 

    :param robot: Manipulator 
    :type robot: URDF model
    :param q: joint state 
    :type q: array like
    :param obstacle_list: list of obstacles 
    :type obstacle_list: list of objects of Shape class
    :param is_swift: whether the Swift environment is used, default is True
    :type is_swift: bool
    :return: whether any self-collision happens for a UR3 robot at the given joint state 
    :rtype: bool value
 
    """
    is_collide = False
    for obstacle in obstacle_list:
        if robot.iscollided(q,obstacle,is_swift):
            is_collide = True
            break
    return is_collide


robo_clone = rtb.models.UR3() # make a clone for self collision check
def is_self_collision_UR3(q)->bool:
    """
    Check whether any self collision happens in an UR3 robot.
    Cannot be used when being imported separately without the robo_clone global variable

    :param q: joint state 
    :type q: array like
    :return: whether any self-collision happens for a UR3 robot at the given joint state 
    :rtype: bool value
 
    """
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

def is_elbow_up(robot, q):
    """
    Check whether the configuration q of the robot is elbow up

    :param robot: UR3 manipulator 
    :type robot: URDF model
    :param q: current joint state of the robot 
    :type q: array like
    :return: whether the configuration q of the robot is elbow up
    :rtype: boolean
 
    """
    # T = get_link_transform(robot,q)
    # if T[4].A[2,3] > T[-1].A[2,3]: # at position 4 is the elbow 
    #     # print(T[4].A[2,3],T[-1].A[2,3]) 
    #     return True
    T = robot.fkine(robot.q)
    R = T.R
    if R[1, 2] > 0: return True
    else: return False

def is_shoulder_left(robot, q):
    """
    Check whether the configuration q of the robot is shoulder left

    :param robot: UR3 manipulator 
    :type robot: URDF model
    :param q: current joint state of the robot 
    :type q: array like
    :return: whether the configuration q of the robot is shoulder left
    :rtype: boolean
 
    """
    # T = get_link_transform(robot,q)
    # if T[4].A[2,3] > T[-1].A[2,3]: # at position 4 is the elbow 
    #     # print(T[4].A[2,3],T[-1].A[2,3]) 
    #     return True
    T = robot.fkine(robot.q)
    R = T.R
    if R[0, 1] < 0: return True
    else: return False

def is_joint_valid(robot,q,obstacle_list = None):
    
    touch_ground= is_touch_ground(robot,q)
    touch_self = is_self_collision_UR3(q)
    if obstacle_list: touch_obstacle = is_collision_obstacle(robot,q,obstacle_list,is_swift = False)
    else: touch_obstacle = False 
    
    if not touch_ground and not touch_self and not touch_obstacle:
        return True,touch_ground,touch_self,touch_obstacle
    else:
        return False,touch_ground,touch_self,touch_obstacle

def solve_for_valid_ik(robot,obj_pose,relative_pose = SE3(0,0,0),q_guess = [], elbow_up_request = False, shoulder_left_request = False)-> np.array:
    """
    Solve for the valid inversed kinematics solution of the UR3 robot at the given goal pose.
    A valid solution will not lead to self-collision and the ground touch. 
    Cannot be use without importing 'is_self_collision_UR3' and 'is_touch_ground'

    :param robot: UR3 manipulator 
    :type robot: URDF model
    :param obj_pose: object pose that require the end-effector to reach 
    :type obj_pose: SE3 object
    :param relative_pose: relative pose from the end-effector to the object pose 
    :type relative_pose: SE3 object
    :param relative_pose: relative pose from the end-effector to the object pose 
    :type relative_pose: SE3 object
    :param q_guess: initial joint value to guess the solution, none by default
    :type q_guess: list of joints
    :param elbow_up_request: if the solution require an elbow up configuration, false by default 
    :type elbow_up_request: boolean
    :param shoulder_left_request: if the solution require an shoulder left configuration, false by default 
    :type shoulder_left_request: boolean
    :return: a valid solution IK solution using Levenberg-Marquadt optimization 
    :rtype: list of joint state
 
    """
    print("--SOLVING INVERSED KINEMATICS:")
    MAX_ITERATION =  100 
    TOLERANCE = 0.05 # tolerance, within 5cm from the goal
    
    it_solve_possible = 0 # iterator for solution-exist cases
    it_solve_not_possible = 0 # iterator for non-solution-exist cases
    valid_solution = False

    while not valid_solution:
        # random guess for ik_solver
        if (not bool(q_guess)): joint_guess_random = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])
        else: joint_guess_random = q_guess
        # Numerical inverse kinematics by default Levenberg-Marquadt optimization
        pick_joint_config = robot.ikine_LM(Tep = obj_pose*relative_pose,q0 = joint_guess_random, joint_limits= True)       
        if pick_joint_config.success:
            if it_solve_possible > MAX_ITERATION : break 
            it_solve_possible += 1 
            
            # if a solution exist, check whether it leads to self-collision or ground touching
            ground_touch = is_touch_ground(robot,pick_joint_config.q)
            self_collision = is_self_collision_UR3(pick_joint_config.q)
            if ground_touch or self_collision:
                print("     -Touch ground or self collision at step", it_solve_possible)
                if ground_touch: print("        -TOUCH GROUND!")
                continue
            else:
                # if the solution doesnt touch ground or self collision, check whether it is within goal tolerance:
                pose_diff =  robot.fkine(pick_joint_config.q) - obj_pose*relative_pose
                if np.linalg.norm(pose_diff[:3,3]) <= TOLERANCE:
                    if elbow_up_request: # when there is a request for elbow up config, check it
                        if is_elbow_up(robot,pick_joint_config.q):
                            print("     ->Valid elbow-up solution found!")
                            valid_solution = True
                            # break
                        else:
                            print("     -Solution found but elbow up condition doesnt match! Re-iterate...") 
                            continue
                    
                    if shoulder_left_request: # when there is a request for shoulder_left config, check it
                        if is_shoulder_left(robot,pick_joint_config.q):
                            print("     ->Valid shoulder-left solution found!")
                            valid_solution = True
                            break
                        else:
                            valid_solution = False
                            print("     -Solution found but shoulder left condition doesnt match! Re-iterate...") 
                            continue
                    print("     ->Valid solution found!")

                    valid_solution = True   
                else:
                    print("     -Get solution but error is larger than tolerance. Keep finding!") 
                    continue
        else:
            if it_solve_not_possible > MAX_ITERATION : break 
            it_solve_not_possible += 1
            print("     -Can't find solution at this step!")
    if valid_solution: 
        print("--SOLVING DONE!")
        return pick_joint_config.q
    else:
        print('     ->Cannot find valid solution!')
        print("--SOLVING DONE!")
        return robot.q

def move_robot_with_object(robot, path, env, obj = None, relative_pose = None, show_path = True):
    """
    Move the robot along the given path. Unsure about self-collision & obstacle (including ground) avoidance 

    :param robot: Manipulator 
    :type robot: URDF model
    :param path: path that the robot should follow along
    :type path: mxn list of ndarray with n-axis robot and m steps
    :param env: environment
    :type env: environment object
    :param obj: object that move along with the robot's end-effector
    :type obj: Shape class
    :param relative_pose: transform from robot end-effector to object's pose
    :type relative_pose: SE3 object
    :param show_path: show the path of the end-effector, true by default
    :type show_path: boolean
 
    """
    print('Unguarantee movement!')
    for q in path:
            robot.q = q
            if show_path: env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.5,0.1,0.1,1)))
            if obj and relative_pose: # if the object option is not None and the relative pose is given
                # update the object position
                pick = SE3(obj.T)
                pick = SE3(robot.fkine(q).A)* relative_pose.inv() 
                obj.T = pick.A
            env.step(0.05)


def move_robot_insurance(robot, q_end, env, obstacle_list = None, show_path = True):
    """
    Create a trapezoidal joint-state path from the current position to the input position and then move the UR3 robot. 
    Check self-collision & obstacle (including ground) and stop the robot when collision will happen. 
    Cannot be use without importing 'is_self_collision_UR3' and 'is_touch_ground'

    :param robot: UR3 manipulator 
    :type robot: URDF model
    :param q_end: final joint state of the robot
    :type q_end: 1x6 array like
    :param env: environment
    :type env: environment object
    :param obstacle_list: list of obstacles inside the environment
    :type obstacle_list: list of obstacles of Shape Class
    :param show_path: show the path of the end-effector, true by default
    :type show_path: boolean
    
    """
    print("**TRY TO MOVE ALONG PATH:")
    # path = rtb.jtraj(q0 = robot.q, qf = q_end,t=60)
    path = rtb.mtraj(tfunc=rtb.trapezoidal,q0 = robot.q,qf=q_end,t =50)
    path_finished = True
    for i in range(len(path)):
        # check each link
        touch_ground= is_touch_ground(robot,path.q[i])
        touch_self = is_self_collision_UR3(path.q[i])
        if obstacle_list: touch_obstacle = is_collision_obstacle(robot,path.q[i],obstacle_list)
        else: touch_obstacle = False 
        
        if not touch_ground and not touch_self and not touch_obstacle:
            if show_path: env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.5,0.1,0.1,1)))
            robot.q = path.q[i]
            env.step(0.05)
        else:
            if touch_ground: print('-May touch ground at step', i ,', another path!')
            if touch_obstacle:  print('-May touch obstacle at step', i ,', another path!')
            if touch_self: print('-Get self collision at step', i ,', another path!')
            path_finished = False
            break               
    print("Path finished:", path_finished)
    return path_finished

def get_valid_path(robot,path,obstacle_list = None):
    print("--CHECK VALIDITY OF PATH:")
    
    path_valid = []
    all_valid = False

    for i in range(len(path)):
        touch_ground= is_touch_ground(robot,path.q[i])
        touch_self = is_self_collision_UR3(path.q[i])
        if obstacle_list: touch_obstacle = is_collision_obstacle(robot,path.q[i],obstacle_list, is_swift = False)
        else: touch_obstacle = False 
        
        if not touch_ground and not touch_self and not touch_obstacle:
            path_valid.append(path.q[i])
        else:
            break

    if len(path_valid) == len(path): 
        print("---->Whole path is valid!") 
        all_valid = True
    else: 
        print("---->Only ",100*len(path_valid)/len(path),"% path is valid!")
    print("--PATH CHECKING DONE!")
    return path_valid, all_valid



