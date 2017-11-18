# Author: Ping-Jung Liu
# Date: November 15th 2017
# COSC 76 Assignment : Motion Planning
# Acknowledgement: Professor Devin Balkom for providing the general structure 

import sys
import math
import random
from math import atan2
from cs1lib import *
from shapely.geometry import Polygon, Point, LineString
from numpy import pi, cos, sin, sinc, array, sqrt, arctan, \
    round

# initialize the map and robot for prm
class prm_map:

	def __init__(self, width, height, obstacles, arm_n, arm_l, resolution):

		self.width = width
		self.height = height
		self.obstacles = obstacles
		# number of random nodes
		self.generate_size = 1000
		# number of neighbors
		self.knn = 15
		# number of arms
		self.arm_n = arm_n
		# lenght of arm
		self.arm_l = arm_l
		self.resolution = resolution
		# testing parameter, please ignore
		self.test = 0
		# the entire roadmap
		self.roadmapp = self.roadmap()



	def roadmap(self):
		#initialize roadmap
		graph = {}
		i = 0
		# grow node until generate_size reached
		while i < self.generate_size:
			# to observe the progress
			print(i)
			# find a random collision free configuration
			config = self.collisionFree_config(graph)
			config = tuple(config)
			# ignore if exist in graph
			if config in graph:
				continue
			# add the configuration to roadmap
			else:
				self.add_config(config, graph)
				i = i + 1

		return graph

	# find a random collision free configuration
	def collisionFree_config(self, graph):

		flag = False
		# loop until a legal configuration found
		while not flag:

			# find a random configuration
			test_config = []
			for i in range(0, self.arm_n):
				rand_theta = random.uniform(0, 2 * pi)
				test_config.append(roundpi(rand_theta))
			# calculate the positions of robot arms
			test_pos = self.calc_pos((0, 0), self.arm_l, test_config)
			# if legal, exit the loop and return the configuration
			if not self.collision(test_pos) and not tuple(test_config) in graph:
				flag = True

		return test_config

	# add a configuration to the roadmap
	def add_config(self, config, graph):

		# find the neighbors of config
		neighbors = self.nnr(config, graph)
		# create edges between config and its neighbors
		graph[config] = neighbors
		for neighbor in neighbors:
			if not config in graph[tuple(neighbor)]:
				graph[tuple(neighbor)].append(config)

	# n nearest neighbors
	def nnr(self, config, graph):

		if len(graph) == 1:
			return []

		# the simple method: loop through all visted node 
		# and find the K nearest neighbors
		simple = []
		for conf in graph:
			simple.append((self.dist_between_config(config, conf), conf))
		simple.sort()

		neighborss = []
		for i in range(0, len(simple)):
			neighborss.append(simple[i][1])

		# this step ensures the trajectory from a config to neighbor is collision free
		if len(neighborss) > self.knn + 100:
			neighborss = neighborss[0: self.knn + 100]
		# k nearest collision free neighbors
		new_neighborss = self.good_neighbors(config, neighborss)
		return new_neighborss

	# filter out the neighbors with trajectory on obstacle
	def good_neighbors(self, config, neighborss):

		neighbors = []
		for i in range(0, len(neighborss)):
			if self.good_neighbor(config, neighborss[i]):
				neighbors.append(neighborss[i])

		if len(neighbors) > self.knn:
			neighbors = neighbors[0: self.knn]
		return neighbors

	# check if there is obstacle between config and neighbor
	# no need to go into the details
	def good_neighbor(self, config, neighbor):

		res = self.resolution
		theta_l = []
		for i in range(0, len(config)):

			temp = []
			if config[i] > neighbor[i]:
				big = config[i]
				small = neighbor[i]
			else:
				big = neighbor[i]
				small = config[i]

			if big - small > pi:
				diff = 2*pi - (big - small)
				step = diff/res
				for j in range(0, res):
					big = big + step
					temp.append(big)
			else:
				diff = big - small
				step = diff/res
				for j in range(0, res):
					small = small + step
					temp.append(small)
			theta_l.append(temp)

		for i in range(0, res):

			test_config = []
			for j in range(0, len(config)):
				test_config.append(theta_l[j][i])

			test_pos = self.calc_pos((0, 0), self.arm_l, test_config)
			if self.collision(test_pos):
				return False
		return True

	# check if a robot arm is collision free
	def collision(self, pos):

		polys = []
		for i in range(0, len(self.obstacles)):
			polys.append(Polygon(self.obstacles[i]))

		robot = LineString(pos)

		for i in range(0, len(polys)):
			if polys[i].intersects(robot):
				return True

		for i in range(0, len(pos)):
			if self.out_of_bound(pos[i]):
				return True

		return False

	# check if a point is out of the map
	def out_of_bound(self, point):

		x = point[0]
		y = point[1]
		return  x > self.width/2 or x < -self.width/2 or y > self.height/2 or y < -self.height/2

	# calculate the position of robot arms
	def calc_pos(self, base, length, theta):

		x = [base[0]]
		y = [base[1]]

		for i in range(0, len(theta)):

			new_x = 0
			new_y = 0
			for j in range(0, i + 1):		

				coss = cos(sum(theta[0:j + 1]))
				sinn = sin(sum(theta[0:j + 1]))
				if abs(coss) < 0.0001:
					coss = round(coss)
				if abs(sinn) < 0.0001:
					sinn = round(sinn)

				new_x = new_x + length[j] * coss
				new_y = new_y + length[j] * sinn


			x.append(new_x)
			y.append(new_y)

		pos = []
		for i in range(0, len(x)):
			pos.append((x[i], y[i]))
		return pos

	# add up the angle different between two configuration
	def dist_between_config(self, config1, config2):

		summ = 0
		for i in range(0, len(config1)):
			diff = abs(config1[i] - config2[i])
			if diff > pi:
				diff = 2 * pi - diff
			summ = summ + diff

		return summ

def roundpi(num):
	return math.floor(num*100)/100

# some tests
if __name__ == "__main__":

	obstacles = [((-160, -160), (-160, -100), (-100, -100), (-100, -160)), ((100, -160), (100, -100), (160, -100), (160, -160)), ((100, 100), (100, 160), (160, 160), (160, 100)), ((-160, 100), (-160, 160), (-100, 160), (-100, 100))]
	test_map = prm_map(800, 800, obstacles, 4, [100, 100, 100, 100], 5)
	print(len(test_map.roadmapp))
