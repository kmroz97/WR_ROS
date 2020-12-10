#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math

from nav_msgs.msg import Odometry
import rospy


class MLine(object):
    def __init__(self):
        self.m_line = {'A': None, 'B': None, 'C': None}
        pass

    def calculate_m_line(self, target_x, target_y): #obliczenie równania prostej MLine
        global m_line
        odom = rospy.wait_for_message('/odom', Odometry)
        # print('pose')
        # print(odom)



        x1 = odom.pose.pose.position.x
        y1 = odom.pose.pose.position.y
        x2 = target_x
        y2 = target_y
        # Ax+By+C=0
        B = 1
        A = 1
        if (x2 == x1):
            B = 0
        elif (y2 == y1):
            A = 0
        else:
            A = -(y2 - y1) / (x2 - x1)
        C = - A * x1 - B * y1

        self.m_line['A'] = A
        self.m_line['B'] = B
        self.m_line['C'] = C

        self.target_x= target_x
        self.target_y= target_y
        self.start_x = x1
        self.start_y = y1

        # print(self.m_line)

    def calculate_distance_from_line(self, x, y): #odległość punktu od prostej
        A = self.m_line['A']
        B = self.m_line['B']
        C = self.m_line['C']
        return abs(A * x + B * y + C) / math.sqrt(math.pow(A, 2) + math.pow(B, 2))

    def calculate_distance_from_line_start(self, point_x, point_y): #odległość od punktu startowego
        diff_x =self.start_x- point_x
        diff_y =self.start_y- point_y
        return math.sqrt(math.pow(diff_x,2)+ math.pow(diff_y,2))
