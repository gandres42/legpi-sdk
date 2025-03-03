import Board as Board
from InverseKinematics import IK
import math
import random

# ik = IK()
# angles = ik.getRotationAngle((0, 18, 7), 0)
# for i, item in enumerate(angles.values()):
    
#     item = item - 90
#     if i + 3 in [3, 5, 6]:
#         item = -item
#     print(i + 3, item)


# invert 3, 5, 6
Board.setMotorDegree(2, 0, blocking=False)
Board.setMotorDegree(3, 90, blocking=False)
Board.setMotorDegree(4, 90, blocking=False)
Board.setMotorDegree(5, -45, blocking=False)
Board.setMotorDegree(6, 0, blocking=True)