import time
import sys
from enum import Enum

sys.path.append('/home/gavin/legpi-sdk')
SCAN_COLORS = ['blue', 'red', 'green']
COLOR_POSITIONS = {
    'red': (-14, 11, 1),
    'green': (-14, 5, 1),
    'blue': (-14, 0, 1)
}

class state(Enum):
    SCANNING = 1
    MOVING = 2
    WAITING = 3
    GRIPPING = 4

from armpy.armik.ArmMoveIK import *
from armpy.armik.Transform import *
import armpy.control.Board as Board
from armpy.perception import Perception

def px_to_cm(coords):
    x, y = coords
    y = 480 - y
    return (x / 640) * 20 - 10, (y / 480) * (28 - 11) + 11

AK = ArmIK()
# result = AK.setPitchRangeMoving((10, 10, 10), -90, -90, 1)
# times = []
# for i, pulse in enumerate(result[0].values()):
#     servo_id = i + 3
#     angle = Board.__pulse_to_angle(pulse)
#     times.append(Board.setServoAngle(servo_id, angle, blocking=False, velocity=100))
# time.sleep(max(times))
# exit()

Board.setServoAngle(6, 0, velocity=100, blocking=True)
Board.setServoAngle(5, 45, velocity=100, blocking=True)
Board.setServoAngle(4, 90, velocity=100, blocking=True)
Board.setServoAngle(3, -90, velocity=100, blocking=True)
Board.setGripperPercent(20)
cap = cv2.VideoCapture(0)
move_time = time.monotonic()
move_goal = (0, 0)
move_color = None
robot_state = state.SCANNING

while True:
    ret, frame = cap.read()

    match robot_state:
        case state.SCANNING:
            for color in SCAN_COLORS:
                centers = Perception.find_centers_color(frame, color)
                if centers != []:
                    print(centers)
                    move_goal = px_to_cm(centers[0])
                    move_color = color
                    robot_state = state.MOVING
        case state.MOVING:
            result = AK.setPitchRangeMoving((move_goal[0], move_goal[1], 1), -90, -90, 0)
            times = []
            for i, pulse in enumerate(result[0].values()):
                servo_id = i + 3
                angle = Board.__pulse_to_angle(pulse)
                times.append(Board.setServoAngle(servo_id, angle, blocking=False, velocity=100))
            move_time = time.monotonic() + max(times)
            robot_state = state.WAITING
        case state.WAITING:
            if time.monotonic() >= move_time:
                robot_state = state.GRIPPING
        case state.GRIPPING:
            Board.setGripperPercent(70)
            result = AK.setPitchRangeMoving((0, 10, 10), -90, -90, 10)
            times = []
            for i, pulse in enumerate(result[0].values()):
                servo_id = i + 3
                angle = Board.__pulse_to_angle(pulse)
                times.append(Board.setServoAngle(servo_id, angle, blocking=False, velocity=100))
            time.sleep(max(times))
            result = AK.setPitchRangeMoving(COLOR_POSITIONS[move_color], -90, -90, 10)
            times = []
            for i, pulse in enumerate(result[0].values()):
                servo_id = i + 3
                angle = Board.__pulse_to_angle(pulse)
                times.append(Board.setServoAngle(servo_id, angle, blocking=False, velocity=100))
            time.sleep(max(times))
            result = AK.setPitchRangeMoving(COLOR_POSITIONS[move_color], -90, -90, 2)
            times = []
            for i, pulse in enumerate(result[0].values()):
                servo_id = i + 3
                angle = Board.__pulse_to_angle(pulse)
                times.append(Board.setServoAngle(servo_id, angle, blocking=False, velocity=100))
            time.sleep(max(times))
            Board.setGripperPercent(0)
            robot_state = state.SCANNING

cap.release()
