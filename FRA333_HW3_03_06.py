# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
1. Janyawat Saingam_6506
2. Keerati Ubonmarl_6503
'''
import numpy as np
from HW3_utils import FKHW3

singularity = 0 # Singularity state
epsilon = 0.001 # Threshold

q = [0,0,0] # Initial q for joint
w = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # External force [3x1] and torque [3x1]

#=============================================<คำตอบข้อ 1>======================================================#
#code here
''' 
    Input 
        q - list of joint angles
    Output:  
        J_0 - Initial Jacobian (without end-effector transformation)
        J_e - End-effector Jacobian (with the effect of the end-effector's orientation)
'''
def endEffectorJacobianHW3(q:list[float])->list[float]:
    R,P,R_e,P_e = FKHW3(q)
    J_0 = np.zeros((6, len(q))) # Create jacobian at 0 matrix
    J_e = np.zeros((6, len(q))) # Create jacobian at end-effector matrix
    
    for i in range(len(q)):
        R_z_i = R[:,2,i] # J_0angular[i]
        P_i = P[:,i] # Position 
        
        J_0[ :3, i] = np.cross(R_z_i , (P_e - P_i)) # J_0 and linear
        J_0[ 3:6 , i] = R_z_i # J_0 add angular
        
        J_e[ :3, i] = np.cross(R_z_i , (P_e - P_i)) @ R_e # J_e and linear
        J_e[ 3:6 , i] = R_z_i @ R_e # J_e add angular
        
    return J_0 , J_e
     
    
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
'''
    Input: 
        q - list of joint angles
    Output: 
        singularity - boolean flag
'''
def checkSingularityHW3(q:list[float])->bool:
    J_e = (endEffectorJacobianHW3(q))[1]  # use only the J_e (end-effector Jacobian)
    J_e_linear = J_e[:3 , :] # use only linear to be metrix 3x3
   
    det_J = np.linalg.det(J_e_linear) # find det of J_e_linear
    if det_J < 0.001:
        singularity = 1 # that q is singularity
    else:
        singularity = 0 # that q isn't singularity
        
    return singularity # singularity state
    
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
'''
    Input: 
        q - list of joint angles
        w - list of external forces and torques [fx, fy, fz, τx, τy, τz]
    Output: 
        effort - list of required efforts for each joint
'''
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J_e = (endEffectorJacobianHW3(q))[1] # use only the J_e (end-effector Jacobian)
    J_Trans = np.transpose(J_e) # Transpose jacobian matrix
    
    tau = J_Trans @ w # J_Transpose cross to wrench
    return tau
#==============================================================================================================#

# print(endEffectorJacobianHW3(q)[1])
# print(checkSingularityHW3(q))
# print(computeEffortHW3(q,w))

# endEffectorJacobianHW3(q)
# checkSingularityHW3(q)
# computeEffortHW3(q,w)