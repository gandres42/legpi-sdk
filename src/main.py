import Board as Board
from InverseKinematics import IK
import math
import random
import time

def resting_pose():
    times = []
    times.append(Board.setMotorAngle(2, 0, blocking=False))
    times.append(Board.setMotorAngle(3, 90, blocking=False))
    times.append(Board.setMotorAngle(4, 90, blocking=False))
    times.append(Board.setMotorAngle(5, -45, velocity=25, blocking=False))
    times.append(Board.setMotorAngle(6, 0, blocking=False))
    time.sleep(max(times) + .02)

def upright_pose():
    times = []
    times.append(Board.setMotorAngle(2, 0, blocking=False))
    times.append(Board.setMotorAngle(3, 0, blocking=False))
    times.append(Board.setMotorAngle(4, 0, blocking=False))
    times.append(Board.setMotorAngle(5, 0, velocity=25, blocking=False))
    times.append(Board.setMotorAngle(6, 0, blocking=False))
    time.sleep(max(times) + .02)

# ik = IK()
# angles = ik.getRotationAngle((0, 18, 7), 0)
# for i, item in enumerate(angles.values()):
#     item = item - 90
#     if i + 3 in [3, 5, 6]:
#         item = -item
#     print(i + 3, item)

# invert 3, 5, 6
upright_pose()
resting_pose()