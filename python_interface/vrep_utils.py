import numpy as np
import vrep
import time

# this starts the communication with vrep (make sure that VRep is started and that the simulation is running)
def start_vrep_connection():
    vrep.simxFinish(-1) #just in case kills all previous communications

    client_id = vrep.simxStart('127.0.0.1',-1,True,True,5000,5) #starts communicating with VRep simulation
    if client_id <0:
        print 'ERROR in Starting Client!'
    else:
        print 'client started successfully'
        print 'client id = ' + str(client_id)
    return client_id

#returns a handle on each joint of the robot (list) and an handle for the endeffector
#also sets up streaming mode to talk to the robot
def initialize_communication(client_id):
    # get handles on each joint of the robot
    joint_handles = []
    for i in range(7):
        res, han = vrep.simxGetObjectHandle(client_id,"LBR_iiwa_7_R800_joint"+str(i+1),vrep.simx_opmode_blocking)
        if res == 0:
            print 'got handle of joint ' + str(i+1)
            print 'handle number ' + str(han)
            joint_handles.append(han)
        
            #setup streaming mode for joint positions (this will allow to speed up communication)
            res, j = vrep.simxGetJointPosition(client_id,joint_handles[i],vrep.simx_opmode_streaming)
            res, f = vrep.simxGetJointForce(client_id,joint_handles[i],vrep.simx_opmode_streaming)
            res, q1_rel = vrep.simxGetObjectQuaternion(client_id, joint_handles[i],-1,vrep.simx_opmode_streaming)
            res, p = vrep.simxGetObjectPosition(client_id, joint_handles[i],-1,vrep.simx_opmode_streaming)
        else:
            print 'error in getting handle for joint ' + str(i+1)
        
    # get a handle on the endeffector
    res, end_effector_handle = vrep.simxGetObjectHandle(client_id,"LBR_iiwa_7_R800_link8",vrep.simx_opmode_blocking)
    if res == 0:
        print 'got handle of end-effector'
        print 'handle number ' + str(end_effector_handle)
        res, j = vrep.simxGetJointPosition(client_id,end_effector_handle,vrep.simx_opmode_streaming)
        res, q1_rel = vrep.simxGetObjectQuaternion(client_id, end_effector_handle,-1,vrep.simx_opmode_streaming)
        res, p = vrep.simxGetObjectPosition(client_id, end_effector_handle,-1,vrep.simx_opmode_streaming)
    else:
        print 'error in getting handle for end-effector'
    
    time.sleep(1.)
    return joint_handles, end_effector_handle

# helper function to transform a quaternion into a rotation matrix
def getRotationFromQuaternion(q):
    R = np.zeros([3,3])
    s = 1. / (q[0]**2 + q[1]**2+q[2]**2 + q[3]**2)
    R[0,0] = 1 - 2*s*(q[1]**2 + q[2]**2)
    R[0,1] = 2*s*(q[0]*q[1] - q[2]*q[0])
    R[0,2] = 2*s*(q[0]*q[2] + q[1]*q[3])
    R[1,0] = 2*s*(q[0]*q[1] + q[2]*q[0])
    R[1,1] = 1 - 2*s*(q[0]**2 + q[2]**2)
    R[1,2] = 2*s*(q[1]*q[2] - q[0]*q[3])
    R[2,0] = 2*s*(q[0]*q[2] - q[1]*q[3])
    R[2,1] = 2*s*(q[1]*q[2] + q[0]*q[3])
    R[2,2] = 1 - 2*s*(q[0]**2 + q[1]**2)
    return R

#gets the objet position and orientation as a homogeneous transform
#the rotation axis a the joint is the z-axis (i.e. its coordinates are given by the 3rd column of the rotation)
def getPoseWorldFrame(client_id, handle):
    res, q = vrep.simxGetObjectQuaternion(client_id, handle,-1,vrep.simx_opmode_buffer)
    R = getRotationFromQuaternion(q)
    res, p = vrep.simxGetObjectPosition(client_id, handle,-1,vrep.simx_opmode_buffer)
    T = np.identity(4)
    T[0:3,0:3] = R
    T[0:3,3] = p
    return T

