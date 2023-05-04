from data_structure_library import Node, Graph, distance, direction, PriorityQueue
import math
import random
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import maze_generation
from bot import Bot
from time import process_time

# Genereate a bot class with all variables needed to be passed between functions
class RRTBot(Bot):
	def __init__(self, epsilon, start, nrow, ncol, color,x ,y ,theta) -> None:
		super().__init__(nrow, ncol, color,x,y,theta)
		self.epsilon = epsilon
		self.current_pos = start
		self.tree = Graph(start)
		self.success = 0
		self.frontier_min=5*self.epsilon
		self.frontier_max=8*self.epsilon
		self.visited=[]
		self.fire = None
		self.frontiers=PriorityQueue()
		self.root=self.current_pos
		self.reverse_path=[]
		self.astar_loc=self.current_pos
		self.current_pos.parent=self.current_pos

	# one iteration of the RRT algorithm
	# Input: 
	#	hwalls, a list of hwall objects that represent all horizontal walls in the maze. Used for collision detection
	#	vwalls, a list of vwall objects that represent all vertical walls in the maze. Used for collision detection
	#	fires, a list of fire objects that represent all the fires in the maze. Used for fire detection
	#	buffer, a float representing the desired buffer between the bot and the fires used for detection
	# Output: None, but the grapg is updated inside the bot local variables
	def rrt_search(self, hwalls, vwalls, fires, buffer):
		new_frontiers=0
		max_frontiers=5
		pqueue = PriorityQueue()
		# Local variable used for counting success
		num_success = 0
		while num_success<100:
			#random sample a node
			q_rand = Node(random.uniform(0,self.nrow-1), random.uniform(0,self.ncol-1))
			#find closest node in tree to random node
			q_curr = self.find_closest_node(q_rand)
			dir = direction(q_curr, q_rand)
			dis = distance(q_curr, q_rand)
			#attempt to grow tree in direction of q_rand a maximum of epsilon
			q_new = Node(row=round(q_curr.row + min(dis,self.epsilon)*math.sin(dir),2),col=round(q_curr.col + min(dis,self.epsilon)*math.cos(dir),2))
			if self.cell(q_new.col,q_new.row) not in self.visited:
				possible_edge = (q_curr, q_new)
				(x,y,collision,outside_bounds,frontier,fire)=self.local_planner(q_curr,q_new,hwalls,vwalls,fires,buffer=buffer)
				# Make sure there are no collisions between the potential node and its parent, and make sure the distance is not outside of the sampling range of the robot
				if not collision and not outside_bounds:
					# Add the node and edge to the tree, if it's a frontier increment frontier and if that's greater than our max number of frontiers break
					q_new.row=y
					q_new.col=x
					q_new.parent = q_curr
					count = 0
					q_new.f = -1*distance(q_new,q_curr)
					self.success+=1
					num_success+=1
					self.tree.V.append(q_new)
					self.visited.append(self.cell(q_new.col,q_new.row))
					self.tree.E.append(possible_edge)
					pqueue.add(q_new)
					if frontier:
						self.frontiers.add(q_new)
						new_frontiers+=1
					if new_frontiers>max_frontiers:
						break
	# Function to generate a path between a provided node and the root
	# Input: q_new, the node we are building a path for to the root
	# Output: Path, a list of nodes from the input node to the root of the tree
	def build_path(self,q_new):
		path=[]
		curr = q_new
		while curr != self.root:
			path.append(curr)
			curr=curr.parent
		return path

	# Increment the actions of the RRT bot. Also seen in the flow chart made in the presentation which represents the communication between bot step functions in simulation.
	# Step details are explained bellow in the code. A goal is the final location along the path of nodes, the destination is the next node in the path
	# Inputs:
	#	hwalls, a list of hwall objects that represent all horizontal walls in the maze. Used for collision detection
	#	vwalls, a list of vwall objects that represent all vertical walls in the maze. Used for collision detection
	#	fires, a list of fire objects that represent all the fires in the maze. Used for fire detection
	#	buffer, a float representing the desired buffer between the bot and the fires used for detection
	# Output:
	#	Time elapsed
	def step(self,hwalls,vwalls,fires,buffer):
		t_start = process_time()
		# If the robot has a path to follow, update it's current position and procede 
		if len(self.path)>0:
			self.current_pos=self.path.pop()
			self._x,self._y=self.current_pos.col,self.current_pos.row
			(fire,detect)=self.fire_detect(self._x,self._y,fires,buffer)
			# If we see a new fire and we dont already have one, we want to mark that we have a fire, then path plan back along the tree to where A* is, and then simulation class will deliver
			# the fire and its location to A* once we get there
			if detect and not self.fire:
				self.fire=fire
				self.fire.found=True
				loc=self.astar_loc
				loc = loc.parent
				self.path = self.build_path(loc)
				path_to_root = self.build_path(self.current_pos)
				path_to_root.reverse()
				# Set up a temp path so we can remove all common nodes between the current location to the root and the goal to the root, that way once we reverse it and remove extra
				# we are left with current to goal
				temp = self.path+path_to_root
				for n in self.path:
					if n in path_to_root:
						temp.remove(n)
				for n in path_to_root:
					if n in self.path:
						temp.remove(n)
				self.path=temp
				astar_loc = Node(self.current_pos.row, self.current_pos.col)
				astar_loc.parent=self.current_pos.parent
				self.astar_loc=astar_loc
		else:
			# If we do not have a path to go, start running RRT
			self.rrt_search(hwalls=hwalls,vwalls=vwalls,fires=fires,buffer=buffer)
			# Once we have frontiers to go to, set the best frontier as the next goal to go to and path plan there
			if len(self.frontiers.q)>0:
				front = self.frontiers.get_min_dist_element()
				self.path=self.build_path(front)					
				if self.current_pos != self.root:
					path_to_root = self.build_path(self.current_pos)
					path_to_root.reverse()
					# Set up a temp path so we can remove all common nodes between the current location to the root and the goal to the root, that way once we reverse it and remove extra
					# we are left with current to goal
					temp = self.path+path_to_root
					for n in self.path:
						if n in path_to_root:
							temp.remove(n)
					for n in path_to_root:
						if n in self.path:
							temp.remove(n)
					self.path=temp
		t_stop = process_time()
		# Return time elapsed in step
		return t_stop-t_start

	# local planner to test possible path
	# Collision detect bot movement between two nodes based on the _dt value of the robot. Done by checking collision bot location against walls in small increments
	# Inputs:
	#	pos1: First node on the segment
	#	pos2: Second node on the segment (Generic names to be potentially used with non-nodes in the future)
	#	hwalls: List of horizontal walls in the maze
	#	vwalls: List of vertical walls in the maze
	#	fires, a list of fire objects that represent all the fires in the maze. Used for fire detection
	#	buffer: Distance buffer in collision detection so you don't get too close
	# Output:
	#	List of variables 
	# If collision detected: collision=True
	# If not: collision=false, x, y, are the end location of the new node, outside_boundsis True if outside the bot sensor range False if not, frontier is True if the new node is in the frontier
	# range, fire_detected is handled elsewhere
	def local_planner(self,pos1,pos2,hwalls,vwalls,fires,buffer):
		# Get the distance and direction between the two nodes
		dir=direction(pos1,pos2)
		dis=distance(pos1,pos2)
		(x,y) = (pos1.col,pos1.row)
		collision=False
		outside_bounds=False
		frontier=False
		fire_detected=None
		# Get a number of values to increment over in the movement
		num_steps=dis//self._dt
		num_iter=int(num_steps)
		remainder=num_steps - num_iter
		# Slowly increment along the line segment path drawn between the two nodes ...
		for i in range(num_iter):
			x+=dis/num_iter*math.cos(dir)
			y+=dis/num_iter*math.sin(dir)
			# ... and collisison detect at every possible segment against the walls. If collision at any point return 
			if self.collision_detect(x,y,hwalls,vwalls,buffer=buffer):
				collision=True
				return (x,y,collision,outside_bounds,frontier,fire_detected)
		# Increment the last small value along the line segment as it would not be broken up perfectly and we don't want to over/under shoot
		x+=remainder*math.cos(dir)
		y+=remainder*math.sin(dir)
		# One final collisison check, if nothing then we are good to check parameters for the new node
		if self.collision_detect(x,y,hwalls,vwalls,buffer=buffer):
			collision=True
			return (x,y,collision,outside_bounds,frontier,fire_detected)
		if distance(Node(y,x),self.current_pos)>self.frontier_max:
			outside_bounds=True
		elif distance(Node(y,x),self.current_pos)>=self.frontier_min:
			frontier=True
		return (x,y,collision,outside_bounds,frontier,fire_detected)

	# method for finding closest node in tree to randomly generated node
	# Input: Randomly generated node
	# Output: Closest node in tree to the random node
	def find_closest_node(self, q_rand):
		min_dist = float('Inf')
		min_node = None
		for node in self.tree.V:
			new_dist = distance(node, q_rand)
			if new_dist < min_dist:
				min_dist = new_dist
				min_node = node
		return min_node
	
	# Collision detection but for the fires in the maze instead
	# Input:
	#	x: current horizontal location of the bot
	#	y: current vertical location of the bot
	#	fires: a list of fire objects that represent all the fires in the maze. Used for fire detection
	#	buffer: a float representing the desired buffer between the bot and the fires used for detection
	# Outputs:
	#	fire, fire object detected or None if none found
	#	Boolean, true if in range of a fire false if not
	def fire_detect(self,x,y,fires,buffer):
		buffer=buffer*10
		# For every fire in the list, check if it has not been found and is not extinguished. If so then check if the distance between the fire in the x and y direction are less than 
		# The maximum allowable distance based on the center of the objects and the buffer added. If so there has been a fire detected
		for fire in fires:
			if not fire.found and fire.active:
				not_y,not_x=False,False
				xmin=fire.col-buffer
				xmax=fire.col+fire.size+buffer
				ymin=fire.row-fire.size-buffer
				ymax=fire.row+buffer
				cx = 0.5*(xmin+xmax)
				cy = 0.5*(ymin+ymax)
				rx=abs(0.5*(xmax-xmin))
				ry=abs(0.5*(ymax-ymin))
				if abs(y-cy)>ry+self.radius:
					not_y=True
				if abs(x-cx)>rx+self.radius:
					not_x=True
				if not(not_x or not_y):
					return (fire, True)
		return (None, False)		
	
def main():
	# ---- Run Maze Generation code
	num_rows = 4 # Number of rows in the maze
	num_cols = 4 # Number of columns in the maze
	num_fires_smol = 5 # Number of 1x1 in the maze
	num_fires_med = 3 # Number of 2x2 in the maze
	num_fires_lrg = 1 # Number of 3x3 in the maze
	num_inside = 8 # Number of padding inside each cell
	num_ent = 1 # Number of entrances to the maze
	plot_maze = False
	[maze, fires, entrances] = maze_generation.generate_maze(num_rows=num_rows, num_cols=num_cols, num_fires_smol=num_fires_smol, num_fires_med=num_fires_med, num_fires_lrg=num_fires_lrg, num_inside=num_inside, num_ent=num_ent, plot_maze=plot_maze)
	start = Node(entrances[0][0], entrances[0][1])
	# Get the list of walls to pass to the robot
	(hwalls, vwalls) = maze_generation.get_list_walls(maze)
	print("RRT main start")
	# Generate the robot
	bot = RRTBot(epsilon=0.5, start=start, nrow=len(maze), ncol=len(maze[0]), color='cyan', x=start.col, y=num_rows-start.row-1, theta=0)
	# Max number of success for demonstration
	rrt_limit = 300
	buffer = bot.radius * 1.1
	while bot.success<rrt_limit:
		bot.step(hwalls, vwalls, fires, buffer=buffer)
	# No path to fires here, just shown is the bot exploring the maze
	maze_generation.plot(field=maze,path=None, bot=bot, fires=fires)

if __name__ == "__main__":
	main()
