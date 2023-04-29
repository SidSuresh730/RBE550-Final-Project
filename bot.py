from data_structure_library import *
import numpy as np
import matplotlib.pyplot as plt
from math import sin,cos,radians,degrees,pi,inf

class Bot():
	def __init__(self, nrow, ncol, color,x,y,theta) -> None:

		self.nrow = nrow
		self.ncol = ncol
		self.tree = None
		self.color = color
		self.path = []
		self.radius=0.25
		self._x=x
		self._y=y
		self._theta=theta
		self._dt=0.1

	def collision_detect(self,x,y,hwalls,vwalls,buffer):
		# assume corners given in clockwise direction starting in top left
		wall_corners_list = list()
		robot_corners = list()
		wall_width = 0.1
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
		
	def kin_move(self, u_omega, u_psi):	
		# calculate deltas based on nonholonomic constraints of diwheel robot
		xD = self._r*u_omega*cos(self._theta)
		yD = self._r*u_omega*sin(self._theta)
		thetaD = self._r/self._L*round(u_psi,4)

		# integrate deltas for a small timestep, dt
		# round the results
		self._x += round(xD*self._dt, 4)
		self._y += round(yD*self._dt, 4)
		self._theta += round(thetaD*self._dt, 2)
		# limit theta to domain [0, 2*pi)
		self._theta = round(self._theta%(2*pi),2)
		# self._corners = self.getPoints(self._x, self._y, self._theta)

	# method used in search to explore possible paths of the robot
	# without changing the actual state of the object
	def motion_primitive(self, x, y, theta, velocity, degrees, field, occupied): # occupied no longer needed but kept 
		#initialize starting cell to current cell (based on given values)
		starting_cell = self.cell(x, y, theta)
		collision = False
		count = 0
		# goal = False
		# check for 0, 0, condition, which would lead to an infinite loop
		# (necessary since 0 velocity is part of the velocity list)
		# (to allow for in place rotation)
		if abs(velocity)<0.1 and abs(degrees)<1:
			print("Won't move")
			# reuse collision so this path will be ignored
			# think about it as collision with previous state of the robot
			collision = True
		else:
			# simple way to guarantee robot moves to different cell
			while self.cell(x, y, theta) == starting_cell:
				# count variable for use in plotting path
				count +=1
				# Same as kin_move
				xD = self._r*velocity*cos(theta)
				yD = self._r*velocity*sin(theta)
				thetaD = self._r/self._L*round(radians(degrees), 4)
				x += round(xD*self._dt, 4)
				y += round(yD*self._dt, 4)
				theta += round((thetaD*self._dt), 2)
				theta = round(theta%(2*pi), 2)
				corners = self.getPoints(x, y, theta)
				# Check for collision after each timestep
				if self.willCollide(corners, field):
					collision = True
		return [x, y, theta, collision, count]
	
	def conv(self, row):
		return self.nrow - row
	
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
		# cell has length and width of 1
		return (int(x), int(y))
		
		

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
