import Board as Board
from InverseKinematics import IK
import math
import random

ik = IK()
angles = ik.getRotationAngle((0, 18, 7), 0)
for i, item in enumerate(angles.values()):
    
    item = item - 90
    if i + 3 in [3, 5, 6]:
        item = -item
    print(i + 3, item)


# invert 3, 5, 6
Board.setMotorDegree(3, 19)
Board.setMotorDegree(4, 81)
Board.setMotorDegree(5, -10)