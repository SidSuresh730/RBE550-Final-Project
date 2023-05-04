import random
import numpy as np
import matplotlib.pyplot as plt # Plotting our obstacle grid
import math
from math import sin, cos, pi
from matplotlib.path import Path # Plotting our search algorithm path
import maze_generation
from data_structure_library import *
from bot import Bot
from time import process_time

# A* bot and relevant functions 
class ABot(Bot):
	def __init__(self, nrow, ncol, color, x, y, theta) -> None:
		super().__init__(nrow, ncol, color, x, y, theta)

		self.maze = None
		self.loc_path = list()

	# a_star: Main path navigation function. Generates the shortest path through an occupancy grid maze using nodes 
	# Inputs:
	#	start: Starting location of the bot as an array /list with values (row, col)
	#	end: Desired end location of the bot as an array /list with values (row, col)
	#	maze: Starting location of the bot as an array /list with values (row, col)
	# Output:
	#	path: List of nodes in the maze starting at the end and finishing at the start. Also populates A*s tree object for plotting
	# Note: This function does take in an end loc and a maze, however those can be removed for this specific application
	def a_star(self, start, end, maze):
		pqueue = PriorityQueue()
		nodes = self.generate_list_nodes(maze=maze, end=end)
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

		while len(pqueue.q) > 0:
			v = pqueue.get_min_dist_element()
			self.tree.remove_vertex(v)
			self.tree.remove_edge((v.parent, v))	
		return path

	# Function that takes a path from A* and runs collisison detection between the nodes as you advance along the path in order to get the most optimal movement between nodes
	# Inputs:
	#	path: Starting location of the bot as an array /list with values (row, col)
	#	maze: 2D occupancy grid representing the maze
	# Output:
	#	optimal_path: List of minimal nodes in the maze for A* to get to the goal using motion primitives and avoiding collision
	# Note: This function does take in a maze, however this can be removed for this specific application
	def local_planner(self, path, maze):
		# Get the list of walls to collision detect
		(hwalls, vwalls) = maze_generation.get_list_walls(maze)
		optimal_path = list()
		optimal_path.append(path[0])
		count = 0
		# Iterate along the path two nodes at a time. If we can move from one to the other without collision keep going, otherwise add it to the list of required nodes
		for i in range(len(path) - 1):
			path_good = self.check_path(optimal_path[count], path[i+1], hwalls, vwalls, buffer=0.1)
			if not path_good:
				optimal_path.append(path[i])
				count += 1
		# Add the final node to the path
		optimal_path.append(path[-1])
		return optimal_path
	
	# Collision detect bot movement between two nodes based on the _dt value of the robot. Done by checking collision bot location against walls in small increments
	# Inputs:
	#	pos1: First node on the segment
	#	pos2: Second node on the segment (Generic names to be potentially used with non-nodes in the future)
	#	hwalls: List of horizontal walls in the maze
	#	vwalls: List of vertical walls in the maze
	#	buffer: Distance buffer in collision detection so you don't get too close
	# Output:
	#	Boolean, True if the path is fine False if it is not
	def check_path(self,pos1,pos2,hwalls,vwalls,buffer):
		# Get the distance and direction between the two nodes
		dir=direction(pos1,pos2)
		dis=distance(pos1,pos2)
		(x,y) = (pos1.col,pos1.row)
		# Get a number of values to increment over in the movement
		num_steps=dis//self._dt
		num_iter=int(num_steps)
		remainder=num_steps - num_iter
		# Slowly increment along the line segment path drawn between the two nodes ...
		for i in range(num_iter):
			x+=dis/num_iter*cos(dir)
			y+=dis/num_iter*sin(dir)
			# ... and collisison detect at every possible segment against the walls. If collision at any point return false
			if self.collision_detect(x,y,hwalls,vwalls,buffer=buffer):
				return False
		# Increment the last small value along the line segment as it would not be broken up perfectly and we don't want to over/under shoot
		x+=remainder*cos(dir)
		y+=remainder*sin(dir)
		# One final collisison check, if nothing then we are good to go
		if self.collision_detect(x,y,hwalls,vwalls,buffer=buffer):
			return False
		return True
	
	# Increment the actions of the A* bot. Also seen in the flow chart made in the presentation which represents the communication between bot step functions in simulation.
	# Step details are explained bellow in the code. A goal is the final location along the path of nodes, the destination is the next node in the path
	# Inputs:
	#	None, everything is done as part of the class variables
	# Output:
	#	True if finished moving to the goal location provided, otherwise nothing (designed to run continually)
	def step(self):
		t_start = process_time()
		# Everytime we step append a value to the bot's loc_path, used in viusalization of the step run of matplotlib
		self.loc_path.append(Node(self._y, self._x))
		# If the robot has a destination to go to,
		if self.destination:
			# First check if  we are at that goal (rounding to avoid floating point path). Need to create nodes to use the node distance function
			if round(distance(Node(self._y, self._x), Node(self.goal[0], self.goal[1])), 4) == 0:
				# If we are at the goal check if that goal was a fire, and if so put it out. Otherwise returning true resets the goal and destination in simulation and thus stops the robot
				if self.fire:
					t_stop = process_time()
					self.fire.active=False
					self.fire=None
					return (True,t_stop-t_start)
				else:
					return (True,0)
			# If we are not at the goal, progress along the path by running the motion primitive.
			else:
				self.motion_primitive()
		# If the robot foes not have destination to go to, check if it has a path. If it has a path run A* to that path, then the local planner to get a smoothed path and set the robot's
		# destination equal to the last value in the path (the order of nodes to go to gets added backwards in A*), if no goal no need to do anything
		elif self.goal:
			start = (round(self._y, 4), round(self._x, 4))
			path = self.a_star(start, self.goal, self.maze)	
			self.path = self.local_planner(path, self.maze)
			self.destination = self.path.pop()
		t_stop = process_time()
		return (False, t_stop-t_start)

	# function that returns a list of nodes inside a maze, only returns free spots (no wall nodes)
	# Input: 
	#	maze, a 2D occupancy grid representing the maze
	#	end, a node that is the end goal of A*. Used in populating node.h values
	# Output:
	#	nodes, a list of nodes
	def generate_list_nodes(self, maze, end):
		num_rows = len(maze)
		num_cols = len(maze[0])
		nodes = []
		for i in range(num_rows):
			for j in range(num_cols):
				if maze[i][j]:
					node = Node(i, j)
					if(end):
						node.h = math.dist([i, j], end)
					else:
						node.h = 0
					nodes.append(node)
		return nodes

# Function for getting the eligable neighbors of the given node from a list of nodes in a grid.
# As this function is only needed for A*, it also handles all cost function needs for A* such as checking the nodes g value and only appending nodes that are not occupied in the graph
# Inputs:
#	node: The node whose neighbors we are looking to examine
#	field: 2D occupancy grid representing the maze the nodes and its neighbors are contained in
#	nodes: List of nodes to search through
#	graph: The bots graph object that we can append to, in A*'s case this is used for plotting mainly
# Output:
#	node
def get_neighbors(node, field, nodes, graph):
	neighbors = []
	row_mod = [-1, -1, -1, 0, 0, 1, 1, 1]
	col_mod = [-1, 0, 1, -1, 1, -1, 0, 1]
	num_neighbors = min(len(row_mod), len(col_mod))
	for i in range(num_neighbors):
		row = node.row + row_mod[i]
		col = node.col + col_mod[i]
		if row >= 0 and col >= 0 and row < len(field) and col < len(field[0]) and field[row][col]:
			Neighbor = find_node(row, col, nodes)
			alt = node.g + distance(node, Neighbor)
			if alt < Neighbor.g:
				graph.remove_edge((Neighbor.parent, Neighbor))
				Neighbor.g = alt
				Neighbor.f = Neighbor.g + Neighbor.h
				Neighbor.parent = node
				graph.E.append((Neighbor.parent, Neighbor))
				neighbors.append(Neighbor)
	return neighbors


# A* main generates a maze based on the parameters given, then runs A* from the entrance of the maze to the first fire in the list. It also then demonstrates the A* stepping from
# the entrance to the first fire, this notably also runs the local planner to smooth the path and advances A* via motion primitives. It plots both of these, the step plot after 
# the previous has been closed
def main():
	print("A Star Main\n")
	# ---- Run Maze Generation code
	num_rows = 4 # Number of rows in the maze
	num_cols = 4 # Number of columns in the maze
	num_fires_smol = 5 # Number of 1x1 in the maze
	num_fires_med = 3 # Number of 2x2 in the maze
	num_fires_lrg = 1 # Number of 3x3 in the maze
	num_inside = 4 # Number of padding inside each cell
	num_ent = 1 # Number of entrances to the maze
	plot_maze = False
	[maze, fires, entrances] = maze_generation.generate_maze(num_rows=num_rows, num_cols=num_cols, num_fires_smol=num_fires_smol, num_fires_med=num_fires_med, num_fires_lrg=num_fires_lrg, num_inside=num_inside, num_ent=num_ent, plot_maze=plot_maze)
	
	# ---- Run A* bot once
	start = entrances[0]
	end = [fires[0].row, fires[0].col]
	bot = ABot(len(maze), len(maze[0]), color="cyan", x=entrances[0][1], y=entrances[0][0], theta=entrances[0][2])
	path = bot.a_star(start, end, maze)
	maze_generation.plot(maze, path, bot, fires)
	
	# ---- Step A*
	terminator = ABot(len(maze), len(maze[0]), color='cyan', x=entrances[0][1], y=entrances[0][0], theta=entrances[0][2])
	terminator.maze = maze
	terminator.goal = (fires[0].row, fires[0].col)
	while True:
		(done,t) = terminator.step()
		if done:
			break
	maze_generation.plot(field=terminator.maze, path=terminator.loc_path, bot=terminator, fires=fires)

if __name__ == "__main__":
	main()
