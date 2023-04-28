from data_structure_library import *
import numpy as np
import matplotlib.pyplot as plt
from math import sin,cos,radians,degrees,pi

class Bot():
	def __init__(self, nrow, ncol, color,x,y,theta) -> None:

		self.nrow = nrow
		self.ncol = ncol
		self.tree = None
		self.color = color

		self.radius=0.5
		self._x=x
		self._y=y
		self._theta=theta
		self._dt=0.1

	# def bot_detect(self,x,y,box):
	# 	# assume corners given in clockwise direction starting in top left
	# 	[c1,c2,c3,c4] = box
	# 	cx = 0.5*(c1[0]+c3[0])
	# 	cy = 0.5*(c1[1]+c3[1])
	# 	rx=abs(0.5*(c2[0]-c1[0]))
	# 	ry=abs(0.5*(c4[1]-c1[1]))
	# 	if abs(y-cy)>ry+self.radius:
	# 		return False
	# 	if abs(x-cx)>rx+self.radius:
	# 		return False
	# 	return True		

	def collision_detect(self,x,y,maze):
		points=np.array([(x-self.radius,y-self.radius),(x-self.radius,y+self.radius),(x+self.radius,y-self.radius),(x+self.radius,y+self.radius)])
		for point in points:
			(x,y)=self.cell(x=point[0],y=point[1])
			if maze[y,x]==OB_NORM:
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
	
	# local planner to get mainshaft movement commands
	def local_planner(self,pos1,pos2,maze):
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
			if self.collision_detect(x,y,maze):
				return False
		x+=remainder*cos(dir)
		y+=remainder*sin(dir)
		if self.collision_detect(x,y,maze):
			return False
		return True

	def conv(self, row):
		return self.nrow - row
	
	def plot(self):
		# plot vertices
		for node in self.tree.V:
			plt.plot(node.col, self.nrow-node.row, 'r.')
		for edge in self.tree.E:
			x_arr = [edge[0].col, edge[1].col]
			y_arr = [self.conv(edge[0].row), self.conv(edge[1].row)]
			plt.plot(x_arr, y_arr, self.color, linestyle="dotted", linewidth=2)

	
	# method for defining a cell that is useful for the algorithm
	def cell(self, x, y):
		# cell has length and width of 1
		return (int(x), int(y))
