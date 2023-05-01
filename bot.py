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
		self._dt=0.05
		self.goal = None
		self.destination = None

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
		
	def motion_primitive(self):
		max_ang = 0.3
		max_move = 1
		distance_to_dest = distance(Node(self._y, self._x), self.destination)
		angle_to_dest = (direction(Node(self._y, self._x), self.destination) - self._theta)
		angle_to_dest = angle_to_dest 
		# if round(distance_to_dest, 4) == 0:
		if distance_to_dest == 0:
			self.destination = self.path.pop()
		else:
			if abs(angle_to_dest) > 0.001:
				# print("Rofl comma copter", angle_to_dest)
				inc_val = min(abs(angle_to_dest), max_ang)
				self._theta += (inc_val * ((angle_to_dest > 0) * 2 - 1)) 
			else:
				inc_val = min(distance_to_dest, max_move)
				self._x += inc_val * cos(self._theta)
				self._y += inc_val * sin(self._theta)
	
	def conv(self, row):
		return self.nrow - row - 1
	
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
		
		

	def line_box_collision(self,pos1,pos2,hwalls,vwalls):
		list_points = list()
		wall_width = 0.2
		for wall in hwalls:
			xmin=wall.llim
			xmax=wall.ulim
			ymin=wall.row-wall_width/2
			ymax=wall.row+wall_width/2
			list_points.append([xmin, ymin])
			list_points.append([xmin, ymax])
			list_points.append([xmax, ymin])
			list_points.append([xmax, ymax])
		for wall in vwalls:
			xmin=wall.col-wall_width/2
			xmax=wall.col+wall_width/2
			ymin=wall.llim
			ymax=wall.ulim
			list_points.append([xmin, ymin])
			list_points.append([xmin, ymax])
			list_points.append([xmax, ymin])
			list_points.append([xmax, ymax])
		for point in list_points:
			dist = self.point_line_dist(point, pos1, pos2)
			if dist < self.radius:
				return True
		return False
	
	def point_line_dist(self, p0, p1_node, p2_node):
		p1 = [p1_node.row, p1_node.col]
		p2 = [p2_node.row, p2_node.col]
		print("PLD", p0, p1, p2)
		dist = abs((p2[0] - p1[0]) * (p2[1] - p1[1]) - (p1[0] - p0[0]) * (p1[1] - p0[1])) / math.sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2))
		return dist

