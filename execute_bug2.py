#!/usr/bin/env python
# -*- coding: utf-8 -*-
from enum import Enum
import math

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan

import sys

sys.path.append('scripts')
sys.path.append('bug_alghoritms')
from definitions import State,Direction

from bug2 import Bug2

target_points = [{'x': 0, 'y': 7}] #ustawienie punktu celu w labiryncie
current_target_point = None

state = State.ROTATE_TO_TARGET
bug2 = None

# funkcja wywolywana przy przyjsciu danych ze skanera laserowego
def scan_callback(scan): 
    global state

    if (state == State.MOVE_TO_TARGET): #do tego stanu trafiamy automatycznie po obróceniu, w związku z własnościami klasy Movement, po której dziedziczy Obstacle
        obstacle_location = bug2.detect_obstacle(scan.ranges) 
        if (obstacle_location): #jednak jeśli zostanie wykryta przeszkoda, przechodzimy do stanu śledzenia tejże przeszkody
            print('wall detected')
            bug2.set_obstacle_location(Direction.RIGHT)
            state = state.FOLLOW_OBSTACLE
    elif (state == State.FOLLOW_OBSTACLE): #a jeśli iteracja wykaże, że znajdujemy się już w stanie śledzenia przeszkody, to wykonujemy metodę z klasy Obstacle
        bug2.go_around_obstacle(scan.ranges)


# funkcja wywolywana przy przyjsciu danych o lokalizacji robota
def odom_callback(odom):
    global state

    pose = Pose()
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    pose.theta = tf.transformations.euler_from_quaternion(
        [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
         odom.pose.pose.orientation.w])[2]
	#obliczanie odległosci miedzy punktem aktualnym a celu
    diff_x = current_target_point['x'] - pose.x 
    diff_y = current_target_point['y'] - pose.y

    if state == State.ROTATE_TO_TARGET:
        state = bug2.rotate_to_target(diff_x, diff_y, pose.theta)
    elif (state == State.MOVE_TO_TARGET):
        state = bug2.move_to_target(diff_x, diff_y)
    elif (state == State.FOLLOW_OBSTACLE): #Jeśli znajdujemy się w stanie śledzenia przeszkody, równolegle do go_around_obstacle jest wykonywana metoda specyficzna dla algorytmu Bug2
        state = bug2.go_from_obstacle(pose.x, pose.y)
    elif (state == State.GET_NEW_TARGET):
        print('fiiiiiiiiiiiiinnnnnnnnnnnnnniiiiiiiiiiiiiisssssssssssshhhhhhhhh')


if __name__ == "__main__":
    rospy.init_node('wr_zad', anonymous=True)

    print("ready")

    current_target_point = target_points.pop(0)
    bug2 = Bug2(current_target_point['x'], current_target_point['y'])

#Tworzymy dwóch Subskryberów sprawdzających równolegle położenie robota na planszy oraz skanujących w poszukiwaniu przeszkody
#Tworzy też Publishera zadającego prędkości kątowe i liniowe
    rospy.Subscriber('/odom', Odometry, odom_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        pub.publish(bug2.new_vel)  # wysłanie prędkosci zadanej
        rate.sleep()

    print("END")
