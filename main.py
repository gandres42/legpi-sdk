import time
import sys
from enum import Enum

sys.path.append('/home/gavin/legpi/armpy')

from armik.ArmMoveIK import *
from control.Board import *

setServoAngle(6, 0)