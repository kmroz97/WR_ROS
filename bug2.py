#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import sys

sys.path.append('../scripts')
from MLine import MLine
from definitions import State
from obstacle import Obstacle

#obiekty typu Bug 2 dziedziczy po Obstacle, ale ponadto ma funkcje związane z algorytmem Bug2 
#posiada obiekty typu MLine i odpowiednio się do niego dostosowuje
class Bug2(Obstacle, object): 
	def __init__(self, target_x, target_y, angular_speed=0.3, linear_speed=0.2, distance_from_mline_threshold=0.1,
				 distance_from_nearest_point_threshold=0.1):
		Obstacle.__init__(self, angular_speed, linear_speed)

		self.mline = MLine() #stworzenie obiektu typu MLine
		self.mline.calculate_m_line(target_x, target_y) #pomiędzy początkiem, a końcem trasy
		self.distance_from_mline_threshold = distance_from_mline_threshold
		self.distance_from_nearest_point_threshold = distance_from_nearest_point_threshold

		self.distance_from_start_to_target = self.mline.calculate_distance_from_line_start(target_x, target_y)
		self.closest_point = {'x': 0, 'y': 0}
		self.hit_point_distance_from_start = 0 #zmienna zapisująca poprzednie miniecię z prostą MLine


	def go_from_obstacle(self, point_x, point_y): #funkcja wywoływana równoległa
		distance_from_m_line = self.mline.calculate_distance_from_line(point_x, point_y)
		if distance_from_m_line < self.distance_from_mline_threshold:
			distance_from_m_line_start = self.mline.calculate_distance_from_line_start(point_x, point_y)

			
			if self.distance_from_start_to_target > distance_from_m_line_start > self.hit_point_distance_from_start + 0.2:
				self.hit_point_distance_from_start = distance_from_m_line_start #tutaj ustawiamy zmienną 
				return State.ROTATE_TO_TARGET

			return State.FOLLOW_OBSTACLE
		else:
			return State.FOLLOW_OBSTACLE
