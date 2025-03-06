#!/usr/bin/python3
import sys
import cv2
import time
import threading

sys.path.append('/home/gavin/legpi-sdk')

from armpy.armik.ArmMoveIK import *
from armpy.armik.Transform import *
import armpy.control.Board as Board
from armpy.perception import Camera
from armpy.perception import Perception

AK = ArmIK()

range_rgb = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

color_range = {
    'red': [(0, 151, 100), (255, 255, 255)], 
    'green': [(0, 0, 0), (255, 115, 255)], 
    'blue': [(0, 0, 0), (255, 255, 110)], 
    'black': [(0, 0, 0), (56, 255, 255)], 
    'white': [(193, 0, 0), (255, 250, 255)], 
}

__target_color = ('red')

def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

#Find outlines with the largest area
#The parameter is a list of outlines to be compared
def getAreaMaxContour(contours) :
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours : #历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  #计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  #只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c

        return area_max_contour, contour_area_max  #返回最大的轮廓

# The angle of closing when clamping the clamp
servo1 = 500

# Initial position
def initMove():
    Board.setServoPulse(1, servo1 - 50, 300)
    Board.setServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

def setBuzzer(timer):
    # Board.setBuzzer(0)
    # Board.setBuzzer(1)
    # time.sleep(timer)
    # Board.setBuzzer(0)
    # time.sleep(timer)
    # Board.setBuzzer(1)
    # time.sleep(timer)
    # Board.setBuzzer(0)
    pass

# Set the RGB light color of the expansion board so that it is consistent with the color you want to track
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()

count = 0
_stop = False
color_list = []
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True
def reset():
    global _stop
    global count
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    global __target_color
    global start_count_t1

    count = 0
    _stop = False
    color_list = []
    get_roi = False
    __target_color = ()
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True

def init():
    print("ColorSorting Init")
    initMove()

def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorSorting Start")

def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorSorting Stop")

def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorSorting Exit")

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False 
world_X, world_Y = 0, 0
def move():
    global rect
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_color
    global start_pick_up
    global rotation_angle
    global world_X, world_Y
    
    #放置坐标
    coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
    }
    while True:
        if __isRunning:        
            if detect_color != 'None' and start_pick_up:  #如果检测到方块没有移动一段时间后，开始夹取
                #移到目标位置，高度6cm, 通过返回的结果判断是否能到达指定位置
                #如果不给出运行时间参数，则自动计算，并通过结果返回
                set_rgb(detect_color)
                setBuzzer(0.1)
                result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0)  
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                    time.sleep(result[2]/1000) #如果可以到达指定位置，则获取运行时间

                    if not __isRunning:
                        continue
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle) #计算夹持器需要旋转的角度
                    Board.setServoPulse(1, servo1 - 280, 500)  # 爪子张开
                    Board.setServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 1.5), -90, -90, 0, 1000)
                    time.sleep(1.5)

                    if not __isRunning:
                        continue
                    Board.setServoPulse(1, servo1, 500)  #夹持器闭合
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    Board.setServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  #机械臂抬起
                    time.sleep(1)

                    if not __isRunning:
                        continue
                    result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)   
                    time.sleep(result[2]/1000)
                    
                    if not __isRunning:
                        continue                   
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        continue                    
                    AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    Board.setServoPulse(1, servo1 - 200, 500)  # 爪子张开  ，放下物体
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)

                    initMove()  # 回到初始位置
                    time.sleep(1.5)

                    detect_color = 'None'
                    get_roi = False
                    start_pick_up = False
                    set_rgb(detect_color)
        else:
            if _stop:
                _stop = False
                Board.setServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)
          
# Run child threads
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()    

t1 = 0
roi = ()
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]

def run(img):
    global roi
    global rect
    global count
    global get_roi
    global center_list
    global unreachable
    global __isRunning
    global start_pick_up
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global start_count_t1, t1
    global detect_color, draw_color, color_list
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

    if not __isRunning:
        return img

    # frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    # frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    # # If an area is detected with an identified object, the area is detected until there is no
    # if get_roi and not start_pick_up:
    #     get_roi = False
    #     frame_gb = getMaskROI(frame_gb, roi, size)      
    # frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    # color_area_max = None
    # max_area = 0
    # areaMaxContour_max = 0
    
    if not start_pick_up:
        # for i in color_range:
        #     if i in __target_color:
        #         frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  # Perform bit operations on the original image and mask
        #         opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))  # Start the operation
        #         closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8)) # Closed operation
        #         contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find out the outline
        #         areaMaxContour, area_max = getAreaMaxContour(contours)  # Find the maximum profile
        #         if areaMaxContour is not None:
        #             if area_max > max_area: # Find the maximum area
        #                 max_area = area_max
        #                 color_area_max = i
        #                 areaMaxContour_max = areaMaxContour
        # if max_area > 2500:  # The maximum area has been found
        if Perception.find_centers(img) != []:
        #     rect = cv2.minAreaRect(areaMaxContour_max)
        #     box = np.intp(cv2.boxPoints(rect))
            
        #     roi = getROI(box) #获取roi区域
        #     get_roi = True
        #     img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # 获取木块中心坐标
             
        #     world_x, world_y = convertCoordinate(img_centerx, img_centery, size) #转换为现实世界坐标
            world_X = 0
            world_Y = 0
            color_area_max = None
            for color in ['red', 'green', 'blue']:
                centers = Perception.find_centers_color(img, color)
                if centers != []:
                    world_X = centers[0][0]
                    world_x = centers[0][0]
                    world_Y = centers[0][1]
                    world_y = centers[0][1]
                    color_area_max = color
                
            
            # cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)
            # cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
            #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[color_area_max], 1) #绘制中心点
            
            distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) #对比上次坐标来判断是否移动
            last_x, last_y = world_x, world_y
            if not start_pick_up:
                if color_area_max == 'red':  #红色最大
                    color = 1
                elif color_area_max == 'green':  #绿色最大
                    color = 2
                elif color_area_max == 'blue':  #蓝色最大
                    color = 3
                else:
                    color = 0
                color_list.append(color)
                # 累计判断
                if distance < 0.5:
                    count += 1
                    center_list.extend((world_x, world_y))
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 1:
                        # rotation_angle = rect[2] 
                        start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                        center_list = []
                        count = 0
                        start_pick_up = True
                else:
                    t1 = time.time()
                    start_count_t1 = True
                    center_list = []
                    count = 0

                if len(color_list) == 3:  #多次判断
                    # 取平均值
                    color = int(round(np.mean(np.array(color_list))))
                    color_list = []
                    if color == 1:
                        detect_color = 'red'
                        draw_color = range_rgb["red"]
                    elif color == 2:
                        detect_color = 'green'
                        draw_color = range_rgb["green"]
                    elif color == 3:
                        detect_color = 'blue'
                        draw_color = range_rgb["blue"]
                    else:
                        detect_color = 'None'
                        draw_color = range_rgb["black"]
        else:
            if not start_pick_up:
                draw_color = (0, 0, 0)
                detect_color = "None"

    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    return img

if __name__ == '__main__':
    init()
    start()
    __target_color = ('red', 'green', 'blue')
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
