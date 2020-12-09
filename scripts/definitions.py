#!/usr/bin/env python
# -*- coding: utf-8 -*-

from enum import Enum

class State(Enum):
	FOLLOW_OBSTACLE = 'follow_obstacle'
	ROTATE_TO_OBSTACLE = 'rotate_to_obstacle'
	ROTATE_TO_TARGET = 'rotate'
	MOVE_TO_TARGET = 'move'
	GET_NEW_TARGET = 'get_new_target'
	INVERT_MOVEMENT_DIRECTION = 'invert_movement_direction'
	STOP ='stop'

class Direction(Enum):
	LEFT = 'left'
	RIGHT = 'right'
	FORWARD = 'forward'