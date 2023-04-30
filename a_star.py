import random
import numpy as np
import matplotlib.pyplot as plt # Plotting our obstacle grid
import math
from math import sin, cos, pi
from matplotlib.path import Path # Plotting our search algorithm path
import maze_generation
from data_structure_library import *
from bot import Bot

class ABot(Bot):
	def __init__(self, nrow, ncol, color, x, y, theta) -> None:
		super().__init__(nrow, ncol, color, x, y, theta)
		self.goal = None
		self.destination = None
		self.maze = np.zeros((self.nrow * 2 + 1, self.ncol * 2 + 1))
		self.big_maze = 0
		self.destination = None
		
		self.loc_path = list()

	# a_star: Main path navigation function. Generates the shortest path through an occupancy grid maze using nodes 
	# Inputs:
	#	start: Starting location of the bot as an array /list with values (row, col)
	def a_star(self, start, end, maze):
		pqueue = PriorityQueue()
		nodes = generate_list_nodes(maze=maze, end=end)
		start_node = find_node(start[0], start[1], nodes)
		if not start_node:
			nodes.append(Node(start[0], start[1]))
			start_node = find_node(start[0], start[1], nodes)
		start_node.g = 0
		end_node = find_node(end[0], end[1], nodes)
		self.tree = Graph(start_node)
		path = []
		current_node = start_node
		pqueue.add(start_node)
		# A Star loop
		while len(pqueue.q) > 0:
			current_node = pqueue.get_min_dist_element()
			if current_node == end_node:
				break
			neighbors = get_neighbors(current_node, maze, nodes, self.tree)
			for n in neighbors:
				pqueue.add(n)
		
		while current_node != start_node:
			path.append(current_node)
			current_node = current_node.parent
		path.append(current_node)
		#path.reverse()

		#while len(pqueue.q) > 0:
		#	v = pqueue.get_min_dist_element()
		#	self.tree.remove_vertex(v)
		#	self.tree.remove_edge((v.parent, v))	
		return path

	def local_planner(self, path, maze):
		(hwalls, vwalls) = maze_generation.get_list_walls(maze)
		optimal_path = list()
		optimal_path.append(path[0])
		count = 0
		for i in range(len(path) - 1):
			path_good = self.check_path(optimal_path[count], path[i+1], hwalls, vwalls, buffer=0.1)
			if not path_good:
				optimal_path.append(path[i])
				count += 1
		optimal_path.append(path[-1])
		return optimal_path
	
	def check_path(self,pos1,pos2,hwalls,vwalls,buffer):
		dir=direction(pos1,pos2)
		dis=distance(pos1,pos2)
		(x,y) = (pos1.col,pos1.row)
		commands = []
		num_steps=dis//self._dt
		num_iter=int(num_steps)
		remainder=num_steps - num_iter
		for i in range(num_iter):
			x+=dis/num_iter*cos(dir)
			y+=dis/num_iter*sin(dir)
			if self.collision_detect(x,y,hwalls,vwalls,buffer=buffer):
				return False
		x+=remainder*cos(dir)
		y+=remainder*sin(dir)
		if self.collision_detect(x,y,hwalls,vwalls,buffer=buffer):
			return False
		return True
	
	def step(self):
		self.loc_path.append(Node(self._y, self._x))
		if self.destination:
			if round(distance(Node(self._y, self._x), Node(self.goal[0], self.goal[1])), 4) == 0:
				return True 
			else:
				self.motion_primitive()
		elif self.goal:
			start = (round(self._y, 4), round(self._x, 4))
			path = self.a_star(start, self.goal, self.big_maze)	
			self.path = self.local_planner(path, self.big_maze)
			self.destination = self.path.pop()

	def motion_primitive(self):
		max_ang = 0.2
		max_move = 1
		distance_to_dest = distance(Node(self._y, self._x), self.destination)
		angle_to_dest = (direction(Node(self._y, self._x), self.destination) - self._theta) % (2*pi)
		angle_to_dest = angle_to_dest 
		if round(distance_to_dest, 4) == 0:
			self.destination = self.path.pop()
		else:
			if abs(angle_to_dest) > 0.001:
				inc_val = min(angle_to_dest, max_ang)
				self._theta += (inc_val * ((angle_to_dest > 0) * 2 - 1)) % (2*pi)
			else:
				inc_val = min(distance_to_dest, max_move)
				self._x += inc_val * cos(self._theta)
				self._y += inc_val * sin(self._theta)

def main():
	print("A Star Main\n")
	# ---- Run Maze Generation code
	num_rows = 4 # Number of rows in the maze
	num_cols = 4 # Number of columns in the maze
	num_fires_smol = 5 # Number of 1x1 in the maze
	num_fires_med = 3 # Number of 2x2 in the maze
	num_fires_lrg = 1 # Number of 3x3 in the maze
	num_inside = 10 # Number of padding inside each cell
	num_ent = 1 # Number of entrances to the maze
	plot_maze = False
	[maze, big_maze, fires, entrances] = maze_generation.generate_maze(num_rows, num_cols, num_fires_smol, num_fires_med, num_fires_lrg, num_inside, num_ent, plot_maze)
	
	
	# ---- Run A*
	bot = ABot(len(big_maze)-1, len(big_maze[0]), color="cyan", x=0, y=0, theta=0)
	start = entrances[0]
	end = [1, 1]
	path = bot.a_star(start, end, big_maze)
	maze_generation.plot(big_maze, path, bot, fires)

if __name__ == "__main__":
	main()
