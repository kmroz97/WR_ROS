#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import turtlesim

import sys

sys.path.append('scripts')
from definitions import State, Direction

from movement import Movement

movement = Movement(angular_speed=0.2, linear_speed=0.5, linear_eps=0, angle_eps=0.05 , linear_distance_threshold=0.1, angle_distance_threshold =0.05) #wywołanie obiektu klasy movement
positions = [[10, 1], [4, 2], [3, 6], [2, 2], [7, 5]]
state = State.ROTATE_TO_TARGET

def set_new_target_point(): #funckja ustawiająca nowy punkt z kolejki, do którego ma dotrzeć żółw
    global target_point
    target_point = positions.pop(0)

def turtlesim_pose_callback(pose): #funkcja wywoływana przez subskrybera, który nasłuchuje pozycję żółwia
    global state

    diff_x = target_point[0] - pose.x
    diff_y = target_point[1] - pose.y

    if state == State.ROTATE_TO_TARGET: #stan, w którym żółw ma się obracać - gdy odbierze nowy punkt docelowy
        state = movement.rotate_to_target(diff_x, diff_y, pose.theta)
    elif state == State.MOVE_TO_TARGET: #stan, w którym żółw ma jechać do celu - po obróceniu się
        state = movement.move_to_target(diff_x, diff_y)
    elif state == State.GET_NEW_TARGET: #stan, w którym żółw się znajdzie, gdy dojedzie do celu i ustawia nowy punkt celu
        if positions:
            set_new_target_point()
            state = State.ROTATE_TO_TARGET
        else:
            state = State.STOP
    elif state == State.STOP: #stan, w którym żółw się znajdzie, gdy dojedzie do celu końcowego (kolejka już jest pusta)
        print('finito')
        sys.exit()

if __name__ == "__main__":
    rospy.init_node('wr_zad', anonymous=True)
    print("ready")
    rospy.Subscriber('/turtle1/pose', turtlesim.msg.Pose, turtlesim_pose_callback)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    set_new_target_point()

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        pub.publish(movement.new_vel)
        rate.sleep()

    print("END")
