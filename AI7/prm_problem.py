# Author: Ping-Jung Liu
# Date: November 15th 2017
# COSC 76 Assignment : Motion Planning
# Acknowledgement: Professor Devin Balkom for providing the general structure 

import sys
import math
import random
from datetime import datetime
from math import atan2
from cs1lib import *
from shapely.geometry import Polygon, Point, LineString
from numpy import pi, cos, sin, sinc, array, sqrt, arctan, \
    round
from prm_map import prm_map
from astar_search import astar_search

# prm_problem used for astar search
class prm_problem:

	def __init__(self, mapp, start, goal):
		# prm_map
		self.mapp = mapp
		# set start and goal
		self.start_state = start
		self.goal = goal
		# add start and goal to roadmap
		self.initialize = self.add_start_goal()

	# add start and goal to roadmap
	def add_start_goal(self):

		self.mapp.add_config(self.start_state, self.mapp.roadmapp)
		self.mapp.add_config(self.goal, self.mapp.roadmapp)
		return True

	# check for goal
	def goal_test(self, state):
		return tuple(state) == tuple(self.goal)

	# get the successors of a state
	def get_successors(self, state):

		neighbors = self.mapp.roadmapp[tuple(state)]
		successors = []
		for i in range(0, len(neighbors)):
			successors.append((neighbors[i], 1))
		return successors

	# this heuristics calculate the sum of angle differences between state and goal
	def heuristics(self, state):

		summ = 0
		for i in range(0, len(state)):
			diff = abs(state[i] - self.goal[i])
			if diff > pi:
				diff = 2 * pi - diff
			summ = summ + diff

		return summ

	def null_heuristic(state):
		return 0

# MAIN!!!!
if __name__ == "__main__":

	# initialize obstacles
	obstacles = [((-160, -160), (-160, -100), (-100, -100), (-100, -160)), ((100, -160), (100, -100), (160, -100), (160, -160)), ((100, 100), (100, 160), (160, 160), (160, 100)), ((-160, 100), (-160, 160), (-100, 160), (-100, 100))]
	
	before = datetime.now()
	# width, height, obstacles, arm number, arm lengths, resolution
	test_map = prm_map(800, 800, obstacles, 4, [100, 100, 100, 100], 10)
	test_prm = prm_problem(test_map, (1.5*pi, 0, 1.5*pi, 0), (0, 0, 0.5*pi, 0))
	after = datetime.now()
	# use astar_search to look for path between start and goal with the roadmap
	result = astar_search(test_prm, test_prm.heuristics)
	print("Time spent:")
	print(after - before)
	print(result)



# The following parts are for visualization! No need to read through!
# changing the above sections will directly affect the visuals
# so please dont modify the below part
########################################################################################

	poss = []
	for i in range(0, len(result.path)):
		poss.append(test_map.calc_pos((0, 0), [100, 100, 100, 100], result.path[i]))

	xx = 0
	def draw():

		global xx, poss
		width = 800
		height = 800
		clear()

		pos = poss[math.floor(xx/50)]
		if math.floor(xx/50) < len(poss) - 1:
			xx = xx + 1

		for i in range(0, len(pos) - 1):
			draw_line(width/2 + pos[i][0], height/2 - pos[i][1], width/2 + pos[i + 1][0], height/2 - pos[i + 1][1])

		set_fill_color(0, 0, 0)

		for i in range(0, len(obstacles)):
			
			x = width/2 + obstacles[i][1][0]
			y = height/2 - obstacles[i][1][1]
			w = obstacles[i][2][0] - obstacles[i][1][0]
			h = obstacles[i][1][1] - obstacles[i][0][1]
			draw_rectangle(x, y, w, h)

	start_graphics(draw ,width=800,height=800)

