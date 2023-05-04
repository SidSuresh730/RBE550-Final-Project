from data_structure_library import *
import numpy as np
import matplotlib.pyplot as plt
from math import sin,cos,radians,degrees,pi,inf

class Bot():
	def __init__(self, nrow, ncol, color, x, y, theta) -> None:

		self.nrow = nrow
		self.ncol = ncol
		self.color = color
		self._x = x
		self._y = y
		self._theta=theta
		self.fire = None
		self.radius=0.25		
		self.path=[]
		self.tree = Graph(Node(y, x))
		self._dt=0.01
		self.goal = None
		self.destination = None

	# Run collision detection between the current location of the bot and all horizontal and veritcal walls in the maze
	# Input:
	#	x: Float, Horizontal location of the robot in simulation space, effectively the col (0 at top left in visualization)
	#	y: Float, Vertical location of the robot in simulation space, effectively the row (0 at top left in visualization)
	#	hwalls: List of Hwall objects, representing the horizontal walls in the maze to collision detect against 
	#	vwalls: List of Vwall objects, representing the vertical walls in the maze to collision detect against 
	#	buffer: Float, Buffer against robot -> wall collision detection, so we dont end up getting to close to an object
	# Output:
	#	Boolean, true if there is a collision false if not
	def collision_detect(self,x,y,hwalls,vwalls,buffer):
		for wall in hwalls:
			not_y,not_x=False,False
			xmin=wall.llim
			xmax=wall.ulim
			ymin=wall.row-buffer
			ymax=wall.row+buffer
			cx = 0.5*(xmin+xmax)
			cy = 0.5*(ymin+ymax)
			rx=abs(0.5*(xmax-xmin))
			ry=abs(0.5*(ymax-ymin))
			if abs(y-cy)>ry+self.radius:
				not_y=True
			if abs(x-cx)>rx+self.radius:
				not_x=True
			if not(not_x or not_y):
				return True
		for wall in vwalls:
			not_y,not_x=False,False
			ymin=wall.llim
			ymax=wall.ulim
			xmin=wall.col-buffer
			xmax=wall.col+buffer
			cx = 0.5*(xmin+xmax)
			cy = 0.5*(ymin+ymax)
			rx=abs(0.5*(xmax-xmin))
			ry=abs(0.5*(ymax-ymin))
			if abs(y-cy)>ry+self.radius:
				not_y=True
			if abs(x-cx)>rx+self.radius:
				not_x=True
			if not(not_x or not_y):
				return True
		return False		
	
	# Increments a bots x and y value based on its current destination and current position. If not angled towards the destination rotate the bot, otherwise drive forward
	# Input: None, everything done is using internal bot variables
	# Output: Nothing, the bot's _x and _y values are edited in the function however 
	def motion_primitive(self):
		# Max angle and max distance to move based on calculations performed on a diwheel bot
		max_ang = 0.3
		max_move = 1
		# Turn loc into node in order to use the distance and direction function between it and the destination, then get distance and angle to move
		# For angle to move subtract the robot's current theta
		distance_to_dest = distance(Node(self._y, self._x), self.destination)
		angle_to_dest = (direction(Node(self._y, self._x), self.destination) - self._theta)
		angle_to_dest = angle_to_dest 
		# If we are at the destination, pop the bot's path for the next destination to go to. This works because the check that there is a pth value 
		# that can be popped is performed in the bot specific step functions
		if distance_to_dest == 0:
			self.destination = self.path.pop()
		else:
			# If not check the angle. If the angle we need to get to is greater than a small value (floating point nonsese) then rotate towards that desired angle.
			if abs(angle_to_dest) > 0.001:
				inc_val = min(abs(angle_to_dest), max_ang)
				# * 2 -1 is my favorite way to map a boolean to -1 or 1
				self._theta += (inc_val * ((angle_to_dest > 0) * 2 - 1)) 
			# If the angle is good, either move the robot the maximum distance it needs to go or until it is right on top of the goal, whichever is smaller. 
			# This is also run with the angle to move to, and is a way to get the robot lined up precisely.
			else:
				inc_val = min(distance_to_dest, max_move)
				self._x += inc_val * cos(self._theta)
				self._y += inc_val * sin(self._theta)
	
	# Converts a row value of the bot from array reference frame to plotting reference frame, only used when plotting
	# Input: Row, the row value to be converted
	# Output: The y location of that row from a standard reference frame, Y pointing up and X pointing right
	def conv(self, row):
		return self.nrow - row - 1
	
	# bot.plot (it's fun to say I promise), plots the bot.tree atribute including veritces and edges
	# Input: None but uses self.tree
	# Output: None but adds to the plot being generated in matplotlib
	def plot(self):
		# plot vertices
		for node in self.tree.V:
			plt.plot(node.col, self.conv(node.row), 'r.')
		for edge in self.tree.E:
			x_arr = [edge[0].col, edge[1].col]
			y_arr = [self.conv(edge[0].row), self.conv(edge[1].row)]
			plt.plot(x_arr, y_arr, self.color, linestyle="dotted", linewidth=2)
	
	# method for defining a cell that is useful for the algorithm
	
	def cell(self, x, y):
		# cell has length and width of n
		n=self.radius
		return (x//n, y//n)
