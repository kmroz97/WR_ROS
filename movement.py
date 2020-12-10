#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from geometry_msgs.msg import Twist
import sys

sys.path.append('scripts')
from definitions import Direction, State


class Movement(object): #klasa, której obiekt służy do sterowania żółwiem 
	def __init__(self, angular_speed, linear_speed, linear_eps=0.1, angle_eps=0.1, linear_distance_threshold=0.1, angle_distance_threshold=0.1): #konstruktor ustawiający parametry
		self.angular_speed = angular_speed
		self.linear_speed = linear_speed
		self.new_vel = Twist() #przekazywanie prędkości do publishera
		self.direction_to_target = None 
		self.angle_distance = 0
		self.previous_angle_distance = 100
		self.linear_distance = 0
		self.previous_linear_distance = 100
		self.linear_eps = linear_eps
		self.angle_eps = angle_eps
		self.linear_threshold = linear_distance_threshold 
		self.angle_threshold = angle_distance_threshold

	def set_rotate_right_speed(self):
		self.new_vel.linear.x = 0
		self.new_vel.angular.z = -self.angular_speed

	def set_rotate_left_speed(self):
		self.new_vel.linear.x = 0
		self.new_vel.angular.z = self.angular_speed

	def set_linear_velocity(self):
		self.new_vel.angular.z = 0
		self.new_vel.linear.x = self.linear_speed

	def set_stop_speed(self):
		self.new_vel = Twist()

	def set_previous_angle_distance(self, distance=None):
		self.previous_angle_distance = self.angle_distance if distance is None else distance

	def set_previous_linear_distance(self, distance=None):
		self.previous_linear_distance = self.linear_distance if distance is None else distance

	def calculate_linear_distance(self, point_a, point_b):
		return point_a - point_b

	def calculate_distance(self, diff_x, diff_y):
		return math.sqrt(math.pow(diff_x, 2) + math.pow(diff_y, 2))

	def calculate_angle_distance(self, diff_x, diff_y, current_angle): #metoda, która oblicza odległość kątową pomiędzy aktualnym ustawieniem, a ustawieniem w kierunku celu
		angle = math.atan2(diff_y, diff_x)

		if angle < 0:
			angle = 2 * math.pi + angle

		if current_angle < 0:
			current_angle = 2 * math.pi + current_angle

		self.angle_distance = angle - current_angle

		return self.angle_distance

	def calculate_rotate_to_target_direction(self): #metoda, która sprawdza, czy należy się obrócić w prawo czy w lewo
		if abs(self.angle_distance) < math.pi:
			if self.angle_distance > 0:
				self.direction_to_target = Direction.LEFT
			else:
				self.direction_to_target = Direction.RIGHT
		else:
			if self.angle_distance > 0:
				self.direction_to_target = Direction.RIGHT
			else:
				self.direction_to_target = Direction.LEFT
		return self.direction_to_target

	def rotate_to_target(self, diff_x, diff_y, current_angle): #metoda, która jest wywoływana w momencie, gdy znajdziemy się w stanie obracania
		self.calculate_angle_distance(diff_x, diff_y, current_angle) #wywołania sprawdzenia o ile trzeba się obrócić
		
		#poniżej sprawdzamy, czy osiągnęliśmy już odpowiednie ustawienie, poprzez sprawdzenie, czy przypadkiem nie zwiększa już się odległość kątowa
		if abs(self.angle_distance) > abs(self.previous_angle_distance) + self.angle_eps or abs(
				self.angle_distance) < self.angle_threshold:
			self.set_stop_speed() #jeśli tak się stanie, to zatrzymujemy żółwia i zmieniamy jego stan na ruch liniowy
			self.set_previous_angle_distance(100)
			return State.MOVE_TO_TARGET

		if self.calculate_rotate_to_target_direction() == Direction.LEFT: #wywołanie sprawdzenia, w którą stronę bardziej opłaca się obracać
			self.set_rotate_left_speed()
		else:
			self.set_rotate_right_speed()

		self.set_previous_angle_distance()

		return State.ROTATE_TO_TARGET

	def move_to_target(self, diff_x, diff_y): #metoda, która jest wywoływana w momencie, gdy znajdziemy się w stanie przemieszczania
		self.set_linear_velocity()
		self.linear_distance = self.calculate_distance(diff_x, diff_y)

		#poniżej sprawdzamy, czy osiągnęliśmy już dotarliśmy do celu, sprawdzając, czy różnica odległości od celu się nie zaczęła zwiększać
		if self.linear_distance > self.previous_linear_distance + self.linear_eps or abs(
				self.linear_distance) < self.linear_threshold:

			self.set_stop_speed() #jeśli tak to zatrzymujemy żółwia i ustawiamy go w stanie pobierania nowego celu z kolejki
			self.set_previous_linear_distance(100)

			return State.GET_NEW_TARGET
		else:
			self.set_previous_linear_distance()
		return State.MOVE_TO_TARGET
