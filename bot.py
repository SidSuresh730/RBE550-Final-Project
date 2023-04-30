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

		self.path = []
		self.radius=0.25		
		self.path=None
		self.tree = None
		self._dt=0.1

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
		# cell has length and width of 1
		return (int(x), int(y))
