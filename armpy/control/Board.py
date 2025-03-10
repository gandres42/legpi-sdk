import time
import RPi.GPIO as GPIO
from armpy.control.BusServoCmd import *
from rpi_ws281x import PixelStrip
from rpi_ws281x import Color as PixelColor
import math
from threading import Thread, Lock


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

__RGB_COUNT = 2
__RGB_PIN = 12
__RGB_FREQ_HZ = 800000
__RGB_DMA = 10
__RGB_BRIGHTNESS = 120
__RGB_CHANNEL = 0
__RGB_INVERT = False

__MIN_GRIPPER_PULSE = 50
__MAX_GRIPPER_PULSE = 610

__PULSE_LIMITS = {
    6: (0, 1000),
    5: (130, 870),
    4: (0, 1000),
    3: (47, 1000),
    2: (0, 1000),
    1: (50, 610)
}
__SERVO_PULSE_LOCK = Lock()
__TIME_OUT = 50

RGB = PixelStrip(__RGB_COUNT, __RGB_PIN, __RGB_FREQ_HZ, __RGB_DMA, __RGB_INVERT, __RGB_BRIGHTNESS, __RGB_CHANNEL)
RGB.begin()
for i in range(RGB.numPixels()):
    RGB.setPixelColor(i, PixelColor(0,0,0))
    RGB.show()

def __angle_to_pulse(degree):
    # Apply the linear transformation
    scaled_value = ((degree + 90) / 180) * (870 - 130) + 130
    return int(scaled_value)

def __pulse_to_angle(pulse):
    # Apply the inverse transformation
    degree = ((pulse - 130) / (870 - 130)) * 180 - 90
    return int(degree)

def __setServoID(oldid, newid):
    """
    Configure the servo id number, the factory default is 1
    :param oldid: The original id, the factory default is 1
    :param newid: new id
    """
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_ID_WRITE, newid)

def __getServoID(id=None):
    """
    Read the serial port servo id
    :param id: default is empty
    :return: Return to the servo id
    """
    
    while True:
        if id is None:
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ)
        else:
            serial_servo_read_cmd(id, LOBOT_SERVO_ID_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ)
        if msg is not None:
            return msg

def __stopBusServo(id=None):
    '''
    Stop the servo operation
    :param id:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_STOP)

def __setServoDeviation(id, d=0):
    """
    Adjust deviation
    :param id: servo id
    :param d: deviation
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d)

def __saveBusServoDeviation(id):
    """
    Configuration deviation, power down protection
    :param id: Servo ID
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_WRITE)

def __getServoDeviation(id):
    '''
    Read the deviation value
    :param id: Servo number
    :return:
    '''
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_OFFSET_READ)
        count += 1
        if msg is not None:
            return msg
        if count > __TIME_OUT:
            return None

def __setServoAngleLimit(id, low, high):
    '''
    Set the rotation range of the servo
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high)

def __getServoAngleLimit(id):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0： 低位  1： 高位
    '''
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_LIMIT_READ)
        if msg is not None:
            count = 0
            return msg

def __setServoVinLimit(id, low, high):
    '''
    设置舵机电压范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high)

def __getServoVinLimit(id):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0： 低位  1： 高位
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_LIMIT_READ)
        if msg is not None:
            return msg

def __setServoMaxTemp(id, m_temp):
    '''
    设置舵机最高温度报警
    :param id:
    :param m_temp:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp)

def __getServoTempLimit(id):
    '''
    读取舵机温度报警范围
    :param id:
    :return:
    '''
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        if msg is not None:
            return msg

def __unloadBusServo(id):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

def getServoPulse(id):
    '''
    读取舵机当前位置
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ)
        if msg is not None:
            return msg

def setServoPulse(id, pulse, use_time):
    """
    Drive the serial port servo to the specified position
    :param id: The servo id to drive
    :pulse: Location
    :use_time: The time it takes to rotate
    """

    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, use_time)

def getServoAngle(id):
    return __pulse_to_angle(getServoPulse(id))

def setServoAngle(motor_id, angle, velocity=50, blocking=False, degrees=True):
    if not degrees: angle = math.degrees(angle)
    
    # unique motor joint restraints
    if motor_id == 1: return
    # if motor_id in [3, 5, 6]: angle = -angle
    
    # convert velocity to ms
    # start_degree = (getServoPulse(motor_id) - 130) * (180 / (870 - 130)) - 90
    start_degree = __pulse_to_angle(getServoPulse(motor_id))
    execution_ms = int(abs(angle - start_degree) / velocity * 1000)
    
    # map degrees into pulse
    target_pulse = __angle_to_pulse(angle)
    if target_pulse < __PULSE_LIMITS[motor_id][0]: target_pulse = __PULSE_LIMITS[motor_id][0]
    if target_pulse > __PULSE_LIMITS[motor_id][1]: target_pulse = __PULSE_LIMITS[motor_id][1]

    # adjust travel time based on actual remaining distance
    execution_s = execution_ms / 1000

    setServoPulse(motor_id, target_pulse, execution_ms)
    
    if blocking: time.sleep(execution_s)
        
    return execution_s

def getServoTemp(id):
    '''
    读取舵机温度
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ)
        if msg is not None:
            return msg

def getServoVin(id):
    '''
    读取舵机电压
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ)
        if msg is not None:
            return msg

def getServoLoadStatus(id):
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        if msg is not None:
            return msg

def setGripperPercent(perc, velocity=75, blocking=True):    
    # convert velocity to ms
    start_perc = (getServoPulse(1) - 50) * (100 / (610 - 50))
    ms = int(abs(perc - start_perc) / velocity * 1000)
    
    # map degrees into pulse
    target_pulse = int(perc * ((610 - 50) / 100) + 50)
    if target_pulse < __MIN_GRIPPER_PULSE: target_pulse = __MIN_GRIPPER_PULSE
    if target_pulse > __MAX_GRIPPER_PULSE: target_pulse = __MAX_GRIPPER_PULSE

    # adjust travel time based on actual remaining distance
    setServoPulse(1, target_pulse, ms)
    if blocking: time.sleep(ms / 1000)

def setBuzzer(new_state):
    GPIO.setup(31, GPIO.OUT)
    GPIO.output(31, new_state)