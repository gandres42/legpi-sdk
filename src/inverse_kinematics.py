#!/usr/bin/env python3
# encoding: utf-8
#4 Inverse kinematics of the robotic arm of degree of freedom: Given the corresponding coordinates (X, Y, Z), and pitch angle, calculate the angle of rotation of each joint
# 2020/07/20 Aiden
import logging
from math import *

# CRITICAL, ERROR, WARNING, INFO, DEBUG
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class IK:
    # Count the servo from bottom to top
    # Common parameters, i.e. connecting rod parameters of 4-degree of freedom robot arm
    l1 = 6.10 #The distance between the center of the robotic arm chassis and the center axis of the second servo 6.10cm
    l2 = 10.16 #The distance from the second servo to the third servo is 10.16cm
    l3 = 9.64 #The distance from the third servo to the fourth servo is 9.64cm
    l4 = 0.00 #The specific assignment is not done here, and the assignment is reassigned according to the initialization selection

    # Special parameters of air pump
    l5 = 4.70 #The distance between the fourth servo and the nozzle is 4.70cm
    l6 = 4.46 #The distance from the nozzle to the nozzle is 4.46cm
    alpha = degrees(atan(l6 / l5)) # Calculate the angle between l5 and l4

    def __init__(self, arm_type='arm'):#Adapt parameters according to different types of clamps
        self.arm_type = arm_type
        if self.arm_type == 'pump': #If it is an air pump type mechanical arm
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))#The fourth servo to the suction nozzle as the fourth connecting rod
        elif self.arm_type == 'arm':
            self.l4 = 16.65  #The distance between the fourth servo and the end of the robot arm is 16.6cm. The end of the robot arm refers to when the claws are completely closed.

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4, L5=l5, L6=l6):
        # Change the connecting rod length of the robot arm to adapt to the robot arm of different lengths of the same structure
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4
        self.l5 = L5
        self.l6 = L6
        if self.arm_type == 'pump':
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))
            self.alpha = degrees(atan(self.l6 / self.l5))

    def getLinkLength(self):
        # Get the current set link length
        if self.arm_type == 'pump':
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4, "L5":self.l5, "L6":self.l6}
        else:
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, pitch_angle):
        # Given the specified coordinates and pitch angle, return the angle at which each joint should rotate, and if there is no solution, return False
        # coordinate_data is the coordinate end coordinate of the clamp, the coordinate unit is cm, passed in as a tuple, for example (0, 5, 10)
        # Alpha is the angle between the holder and the horizontal plane, unit

        # Suppose the end of the clamp is P(X, Y, Z), the coordinate origin is O, the origin is the projection of the center of the gimbal on the ground, and the projection of point P on the ground is P_
        # The intersection point between l1 and l2 is A, the intersection point between l2 and l3 is B, and the intersection point between l3 and l4 is C
        # CD is perpendicular to PD, CD is perpendicular to the z-axis, then the pitch angle Alpha is the angle between DC and PC, AE is perpendicular to DP_, and E is perpendicular to DP_, CF is perpendicular to AE, and F is perpendicular to AE
        # Angle representation: For example, the angle between AB and BC is represented as ABC
        X, Y, Z = coordinate_data
        if self.arm_type == 'pump':
            pitch_angle -= self.alpha
        #Find the rotation angle of the base
        theta6 = degrees(atan2(Y, X))
 
        P_O = sqrt(X*X + Y*Y) #P_Distance to origin O
        CD = self.l4 * cos(radians(pitch_angle))
        PD = self.l4 * sin(radians(pitch_angle)) #When the pitch angle is positive, PD is positive, and when the pitch angle is negative, PD is negative
        AF = P_O - CD
        CF = Z - self.l1 - PD
        AC = sqrt(pow(AF, 2) + pow(CF, 2))
        if round(CF, 4) < -self.l1:
            logger.debug('Height below 0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4):#The sum of the two sides is smaller than the third side
            logger.debug('Cannot form a connecting rod structure, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        #See theat4
        cos_ABC = round(-(pow(AC, 2)- pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*self.l3), 4) #余弦定理
        if abs(cos_ABC) > 1:
            logger.debug('Cannot form a connecting rod structure, abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC) #Inverse triangle to calculate the arc
        theta4 = 180.0 - degrees(ABC)

        #Quest theta5
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) #余弦定理
        if abs(cos_BAC) > 1:
            logger.debug('Cannot form a connecting rod structure, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))

        #Quest theta3
        theta3 = pitch_angle - theta5 + theta4
        if self.arm_type == 'pump':
            theta3 += self.alpha

        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6} # Return to the angle dictionary when there is a solution
            
if __name__ == '__main__':
    ik = IK('arm')
    ik.setLinkLength(L1=ik.l1 + 0.89, L4=ik.l4 - 0.3)
    print('Link length:', ik.getLinkLength())
    print(ik.getRotationAngle((0, 0, ik.l1 + ik.l2 + ik.l3 + ik.l4), 90))
