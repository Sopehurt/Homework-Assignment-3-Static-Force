# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
1. Janyawat Saingam_6506
2. Keerati Ubolmart_6503
'''
import roboticstoolbox as rtb
import numpy as np
from FRA333_HW3_03_06 import endEffectorJacobianHW3, checkSingularityHW3, computeEffortHW3
from spatialmath import SE3
from math import pi, atan2
import random

#===========================================<ตรวจคำตอบข้อ 1>====================================================#
# code here
d_1 = 0.0892
a_2 = -0.425
a_3 = -0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

q = [0, 0, 0]
w = [1.0, 0.0, 1.0, 0.0, 1.0, 0.0]

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha=0.0, a=0.0, d=d_1, offset=pi),
        rtb.RevoluteMDH(alpha=pi/2, a=0.0, d=0.0, offset=0.0),
        rtb.RevoluteMDH(alpha=0.0, a=a_2, d=0.0, offset=0.0)
    ],
    tool=SE3([
        [0, 0, -1, (a_3 - d_6)],
        [0, 1, 0, -d_5],
        [1, 0, 0, d_4],
        [0, 0, 0, 1]
    ]),
    name="RRR_Robot"
)

jacobian_robot_toolbox_rounded = np.round(robot.jacobe(q), 6)
jacobian_my_equation_rounded = np.round(endEffectorJacobianHW3(q)[1], 6)

with np.printoptions(precision=6, suppress=True, floatmode='fixed'):
    print("1) Test = ")
    print("jacobian from robotics toolbox")
    print(jacobian_robot_toolbox_rounded)
    print("jacobian from my equation")
    print(jacobian_my_equation_rounded)
    print("-----------------------------------------------")
#==============================================================================================================#

#===========================================<ตรวจคำตอบข้อ 2>====================================================#
# code here
q1 = float(random.uniform(-pi, pi))
q2 = float(random.uniform(-pi, pi))
q3 = float(random.uniform(-pi, pi))
q = [q1, q2, q3]

det_J = np.linalg.det(robot.jacobe(q)[:3, :])  # find det of Jacobian end-effector from RTB at only 3x matrix
if det_J < 0.001:
    singularity = 1  # that q is singularity
else:
    singularity = 0  # that q isn't singularity

print("2) Test = ")
print("random q")
print(q)
print("check singularity state from RTB")
print(singularity)
print("return singularity state from my function")
print(checkSingularityHW3(q))
print("-----------------------------------------------")
#==============================================================================================================#

#===========================================<ตรวจคำตอบข้อ 3>====================================================#
# code here
w = [random.uniform(0, 10) for _ in range(6)]

J_e = robot.jacobe(q)
effort_robotics_toolbox = np.round(robot.pay(w, q, J_e), 6)
effort_my_equation = np.round(computeEffortHW3(q, w), 6)

with np.printoptions(precision=6, suppress=True, floatmode='fixed'):

    print("3) Test = ")
    print("random w")
    print(w)
    print("Tau form robotics_toolbox")
    print(effort_robotics_toolbox)  # force from robot
    print("Tau form my equation")
    print(effort_my_equation)  # force from external force
    print("effort_robotics_toolbox + effort_my_equation = ")
    print(effort_robotics_toolbox + effort_my_equation)
    print("-----------------------------------------------")
#==============================================================================================================#
