#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import sys

sys.path.append('scripts')
from movement import Movement
from definitions import Direction, State


class Obstacle(Movement, object):
    def __init__(self, angular_speed, linear_speed):
        Movement.__init__(self, angular_speed, linear_speed)
        self.left_distance_to_obstacle_sensor = 90
        self.right_distance_to_obstacle_sensor = 270

        self.left_distance_sensor = 45
        self.forward_distance_sensor = 0
        self.right_distance_sensor = 315

        self.forward_distance_threshold = 0.4
        self.distance_threshold = 0.35

        self.wall_distance_threshold = 0.3

        self.obstacle_location = None

        self.previous_distance_to_obstacle = 0
        self.distance_to_obstacle = 0

        self.set_PID_params()

    def set_obstacle_location(self, obstacle_location):
        self.obstacle_location = obstacle_location

    def invert_obstacle_location(self):
        self.obstacle_location = Direction.LEFT if self.obstacle_location == Direction.RIGHT else Direction.RIGHT

    def invert_movement_direction(self, scan_ranges):
        self.turn_to_obstacle()
        self.set_distance_to_obstacle(scan_ranges)
        if self.distance_to_obstacle < self.wall_distance_threshold + 0.1 and self.distance_to_obstacle is not None:
            return State.FOLLOW_OBSTACLE
        return State.INVERT_MOVEMENT_DIRECTION

    def detect_obstacle(self, scan_ranges):
        if scan_ranges[self.left_distance_sensor] < self.distance_threshold:
            obstacle_location = Direction.LEFT
        elif scan_ranges[self.forward_distance_sensor] < self.distance_threshold:
            if scan_ranges[self.left_distance_sensor] >= scan_ranges[self.right_distance_sensor]:
                obstacle_location = Direction.RIGHT
            else:
                obstacle_location = Direction.LEFT
        elif scan_ranges[self.right_distance_sensor] < self.distance_threshold:
            obstacle_location = Direction.RIGHT
        else:
            obstacle_location = None
        return obstacle_location

    def turn_to_obstacle(self):
        if (self.obstacle_location == Direction.LEFT):
            self.set_rotate_right_speed()
        elif (self.obstacle_location == Direction.RIGHT):
            self.set_rotate_left_speed()

    def set_distance_to_obstacle(self, scan_ranges):
        if (self.obstacle_location == Direction.LEFT):
            self.distance_to_obstacle = scan_ranges[self.left_distance_to_obstacle_sensor]

        elif (self.obstacle_location == Direction.RIGHT):
            self.distance_to_obstacle = scan_ranges[self.right_distance_to_obstacle_sensor]

        self.distance_to_obstacle = 100 if math.isinf(self.distance_to_obstacle) else self.distance_to_obstacle

    def calculate_relative_distance_to_obstacle(self, distance_to_obstacle):
        self.relative_distacne_from_obsctacle = (
                                                        self.wall_distance_threshold - distance_to_obstacle) / self.wall_distance_threshold
        return self.relative_distacne_from_obsctacle

    def calculate_relative_absolute_dynamic_distance_to_obstacle(self, distance_from_wall):
        self.relative_absolute_previous_distacne_from_obsctacle = abs(
            distance_from_wall - self.previous_distance_to_obstacle) / self.wall_distance_threshold
        return self.relative_absolute_previous_distacne_from_obsctacle

    def set_PID_linear_velocity(self, relative_distance_difference, relative_absolute_dynamic_difference):
        self.new_vel.linear.x = max(self.min_linear_speed,
                                    self.linear_speed * (1 - min(abs(relative_distance_difference) * self.K_linear, 1)
                                                         - min(self.D_linear * relative_absolute_dynamic_difference,
                                                               1)))

    def set_PID_angular_velocity(self, angular_velocity_sign, relative_distance_difference,
                                 relative_absolute_dynamic_difference):
        new_angular_speed = angular_velocity_sign * self.angular_speed * (
                relative_distance_difference * self.K_angular - math.copysign(relative_absolute_dynamic_difference,
                                                                              relative_distance_difference) * self.D_angular)

        extra_factor = 0

        new_angular_speed = new_angular_speed + angular_velocity_sign * self.angular_speed * math.copysign(extra_factor,
                                                                                                           relative_distance_difference)
        self.new_vel.angular.z = math.copysign(min(self.max_angular_speed, abs(new_angular_speed)), new_angular_speed)

    def set_PID_params(self, K_linear=0.5, D_linear=0.02, min_linear_speed=0.1, K_angular=3, D_angular=2,
                       max_angular_speed=0.3, K_angular_extra_factor=0.05):
        self.K_linear = K_linear
        self.D_linear = D_linear
        self.min_linear_speed = min_linear_speed
        self.K_angular = K_angular
        self.D_angular = D_angular
        self.max_angular_speed = max_angular_speed
        self.K_angular_extra_factor = K_angular_extra_factor

    def go_around_obstacle(self, scan_ranges):
        new_obstacle = self.detect_obstacle(scan_ranges)
        if new_obstacle:
            self.turn_to_obstacle()
        else:
            self.set_distance_to_obstacle(scan_ranges)

            relative_distance_difference = self.calculate_relative_distance_to_obstacle(self.distance_to_obstacle)
            relative_absolute_dynamic_difference = self.calculate_relative_absolute_dynamic_distance_to_obstacle(
                self.distance_to_obstacle)

            self.set_PID_linear_velocity(relative_distance_difference, relative_absolute_dynamic_difference)

            angular_velocity_sign = 1 if self.obstacle_location == Direction.RIGHT else -1

            self.set_PID_angular_velocity(angular_velocity_sign, relative_distance_difference,
                                          relative_absolute_dynamic_difference)

            self.previous_distance_to_obstacle = self.distance_to_obstacle
