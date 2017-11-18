# Author: Ping-Jung Liu
# Date: November 15th 2017
# COSC 76 Assignment : Motion Planning
# Acknowledgement: Professor Devin Balkom for providing the general structure 

from math import atan2
from cs1lib import *
from planarsim import *
from display_planar import *
import sys
import math
import random
from shapely.geometry import Polygon, Point, LineString
from numpy import pi, cos, sin, sinc, array, sqrt, arctan, \
	round
from datetime import datetime

class rrt_planner:

	def __init__(self, width, height, obstacles, start, goal, delta, resolution):

		# width and height of map
		self.width = width
		self.height = height
		# obstacles
		self.obstacles = obstacles
		# start and goal
		self.start = start
		self.goal = goal
		# time of each step
		self.delta = delta

		self.resolution = resolution
		# child to parent graph
		self.graph = {}
		# a list of current leaves
		self.leaves = []
		# a list of current parent
		self.parent = []
		# testing parameter, ignore
		self.flag = 0
		# approximate goal
		self.dist_to_goal = 30

	def solve(self):

		# grow the map until reaches the proximity of the goal
		goal_close = self.grow_map()
		print(goal_close)

		# backtrack to obtain the solution path
		path = []
		cur = goal_close
		while not cur == self.start:

			path.append(self.graph[cur])
			cur = self.graph[cur][0]

		return path

	# grow the whole map
	def grow_map(self):

		# potential goal?
		potential = self.expand(self.start)

		# loop until potential goal is close to actual goal
		i = 0
		while self.flag == 0:
			# observe the progess
			print(i)
			# random point
			random_x = random.uniform(0, self.width)
			random_y = random.uniform(0, self.height)
			# find the closest leaf
			neighbor = self.closest_vertex(random_x, random_y)
			# expand the leaf
			potential = self.expand(neighbor)
			i = i + 1

		return potential

	# find the closest leaf (vertex)
	def closest_vertex(self, x, y):

		neighbors = []
		for node in self.leaves:
			neighbors.append((self.dist((x, y), (node[0], node[1])), node))
		neighbors.sort()

		return neighbors[0][1]

	# expand the leaf
	def expand(self, vertex):

		# since we are expanding the leaf, put it in parent
		# then remove from leaves
		self.parent.append(vertex)
		if vertex in self.leaves:
			self.leaves.remove(vertex)
		# get the transform of the leaf
		start_t = transform_from_config(vertex)

		# try all six control moves 
		for i in range(0, 6):
			test_transform = single_action(start_t, controls_rs[i], self.delta)
			test_config = config_from_transform(test_transform)

			# test for collision and map bounds
			if self.good_control(vertex, controls_rs[i]) and not test_config in self.graph:
				self.graph[test_config] = (vertex, controls_rs[i])
				self.leaves.append(test_config)

				# if the resulting configuration is close to goal, return
				if self.close_to_goal(test_config):
					self.flag = 1
					return test_config
		return None

	# check if config is close to goal
	def close_to_goal(self, config):
		return self.dist((config[0], config[1]), (self.goal[0], self.goal[1])) < self.dist_to_goal

	# euclidean distance between two points
	def dist(self, p1, p2):
		return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2))

	# sample the trajectory of a control from config1 and check for collision
	def good_control(self, config1, control):

		t_list = sample_trajectory([control], [self.delta], self.delta, self.resolution, transform_from_config(config1))

		for i in range(0, len(t_list)):
			temp = config_from_transform(t_list[i])
			if self.point_collision((temp[0], temp[1])):
				return False

		return True

	# check if obstacles contains point
	def point_collision(self, point):

		polys = []
		for i in range(0, len(self.obstacles)):
			polys.append(Polygon(self.obstacles[i]))

		for i in range(0, len(polys)):
			if polys[i].contains(Point(point[0], point[1])):
				return True

		if self.out_of_bound(point):
			return True

		return False

	# out of map bounds
	def out_of_bound(self, point):

		x = point[0]
		y = point[1]
		return  x >= self.width or x <= 0 or y >= self.height or y <= 0

if __name__ == "__main__":

	# initialize all the parameters
	width = 400
	height = 400
	start = (300, 50, 0)
	goal = (200, 370, 0)
	delta = 5
	resolution = 5
	obstacles = [((330, 330), (330, 400), (370, 400), (370, 330)), ((60, 300), (60, 350), (150, 350), (150, 300)), ((100, 100), (100, 150), (400, 150), (400, 100)), ((30, 200), (30, 240), (350, 240), (350, 200))]
	# initialize rrt solver
	before = datetime.now()
	test_rrt = rrt_planner(width, height, obstacles, start, goal, delta, resolution)
	# find the path from start to goal with simple backtrack
	solution = test_rrt.solve()
	after = datetime.now()
	print("time spent:")
	print(after - before)



# The following parts are for visualization! No need to read through!
########################################################################################

	controls = []
	time = []
	for i in range(0, len(solution)):
		controls.append(solution[i][1])
		time.append(delta)
	controls.reverse()

	def draww():

		global test_rrt
		global solution
		global delta
		global resolution
		global controls
		global time
		global width
		global height
		g = test_rrt.graph
		
		for child in g:
			#if g[child] in solution:
			#	set_stroke_color(1, 0, 0)

			samples = sample_trajectory([g[child][1]], [delta], delta, 5, transform_from_config(g[child][0]))
			tview = TrajectoryView(samples, g[child][0][0], g[child][0][1], 1)
			tview.draw()
			set_stroke_color(0, 0, 0)

		set_stroke_color(1, 0, 0)
		samples = sample_trajectory(controls, time, delta*len(time), 1000, transform_from_config(start))
		tview = TrajectoryView(samples, start[0], start[1], 1)
		tview.draw()
		set_stroke_color(0, 0, 0)
		set_fill_color(1, 1, 1)

		for i in range(0, len(obstacles)):
			
			x = obstacles[i][1][0]
			w = obstacles[i][2][0] - obstacles[i][1][0]
			h = obstacles[i][1][1] - obstacles[i][0][1]
			y = obstacles[i][1][1] - h
			draw_rectangle(x, y, w, h)

	start_graphics(draww ,width=width,height=height)


