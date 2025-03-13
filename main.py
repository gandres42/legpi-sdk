import time
import sys
from enum import Enum

sys.path.append('/home/gavin/legpi/armpy')

from armik.ArmMoveIK import *
from control.Board import *

# order: 6, 5, 4, 3
def row():
    times = []
    times.append(setServoAngle(6, 0, velocity=100))
    times.append(setServoAngle(5, 70, velocity=50))
    times.append(setServoAngle(4, 45, velocity=100))
    times.append(setServoAngle(3, 45, velocity=100))
    time.sleep(max(0, max(times)))

    times = []
    times.append(setServoAngle(6, 0))
    times.append(setServoAngle(5, 40, velocity=30))
    times.append(setServoAngle(4, 80))
    times.append(setServoAngle(3, 80))
    time.sleep(max(0, max(times) - .1))

    times = []
    times.append(setServoAngle(5, 70, velocity=100))
    # times.append(setServoAngle(5, 10, velocity=100))
    time.sleep(.3)

def backward_row():
    times = []
    times.append(setServoAngle(5, 00, velocity=150))
    times.append(setServoAngle(4, 90, velocity=100))
    times.append(setServoAngle(3, 90, velocity=100))
    # times.append(setServoAngle(5, 10, velocity=100))
    # times.append(setServoAngle(5, 10, velocity=100))
    time.sleep(1)

    times = []
    times.append(setServoAngle(6, 0))
    times.append(setServoAngle(5, 40, velocity=30))
    times.append(setServoAngle(4, 80))
    times.append(setServoAngle(3, 80))
    time.sleep(max(0, max(times) - .1))

    times = []
    times.append(setServoAngle(6, 0, velocity=100))
    times.append(setServoAngle(5, 70, velocity=50))
    times.append(setServoAngle(4, 45, velocity=100))
    times.append(setServoAngle(3, 45, velocity=100))
    time.sleep(max(0, max(times)))

    

while True:
    backward_row()