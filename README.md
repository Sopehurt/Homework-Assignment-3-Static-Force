# FRA333 Homework Assignment 3: Static Force

- Keerati Ubonmarl_6503
- Janyawat Saingam_6506

### Objective :
การบ้านนี้ถูกออกแบบึ้นมาเพื่อให้ผู้เรียนได้ประยุกต์ใช้องค์ความรู้การหาจลนศาสตร์เชิงอนุพันธ์ (Differential kinematics) ของหุ่นยนต์แขนกล 3 แกน (3-DOF Manipulator)

### Create RRR Robot :
นำความรู้ในเรื่องของการหาจลศาสตร์เชิงอนุพันธ์มาเพื่อคสบคุมหุ่นยนต์ RRR โดยเริ่มจากการสร้าง MDH Parameter ของหุ่นยนต์ RR ก่อนดังนี้

```python
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from math import pi,atan2
import random

robot = rtb.DHRobot(
[
    rtb.RevoluteMDH(alpha = 0.0     ,a = 0.0      ,d = d_1      ,offset = pi),
    rtb.RevoluteMDH(alpha = pi/2    ,a = 0.0      ,d = 0.0      ,offset = 0.0),
    rtb.RevoluteMDH(alpha = 0.0     ,a = a_2      ,d = 0.0      ,offset = 0.0)
],
tool = SE3([
    [0 , 0 , -1 , (a_3 - d_6)  ],
    [0 , 1 , 0 ,      -d_5     ],
    [1 , 0 , 0 ,       d_4     ],
    [0 , 0 , 0 ,        1      ]]),

name = "RRR_Robot"
)
```

จะได้ผลลัพธ์ดังนี้
```bash
┌────────┬───────┬────────────┬────────┐
│  aⱼ₋₁  │ ⍺ⱼ₋₁  │     θⱼ     │   dⱼ   │
├────────┼───────┼────────────┼────────┤
│    0.0 │  0.0° │  q1 + 180° │ 0.0892 │
│    0.0 │ 90.0° │         q2 │    0.0 │
│ -0.425 │  0.0° │         q3 │    0.0 │
└────────┴───────┴────────────┴────────┘

┌──────┬─────────────────────────────────────────────────┐
│ tool │ t = -0.47, -0.093, 0.11; rpy/xyz = 0°, -90°, 0° │
└──────┴─────────────────────────────────────────────────┘
```

และได้ pose ของหุ่นยนต์ดังนี้

![alt text](assets/image.png)