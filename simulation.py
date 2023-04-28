from math import sin, cos, tan, radians, degrees
# import mypylib.myfunctions as mf
from data_structure_library import *
from maze_generation import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random
import pygame
from rrt_robot import *
from mapper_robot import *
from fire_fighting_robot import *
from time import process_time
# States of grid/obstacles
EMPTY = 0
OB_NORM = 1
OB_BURN = 2
OB_EXT = 3
OB_BURNED = 4
OBSTACLE_LIFE = 500
#Colors
GREEN = (0,255,0)
RED = (255,0,0)
DARK_GREEN = (0,153,0)
BLACK = (0,0,0)
TAN = (210,180,140)
WHITE = (255,255,255)
FIRE_ENGINE_RED = (206,32,41) 
# Pygame constants and inits
WIDTH, HEIGHT = 1500,1500
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
FPS = 10
# WUMPUS_IMAGE = pygame.image.load('wumpus.png')
# WUMPUS = pygame.transform.scale(WUMPUS_IMAGE,(25,25))
pygame.display.set_caption("First Line Robust Automatic Aviators")

class Simulation:
	def __init__(self,nrows,ncols,smol,med,lrg,num_inside,num_ent,pixel_factor):
		self.pixel_factor = pixel_factor
		# maze values
		self.num_rows = nrows # Number of rows in the maze
		self.num_cols = ncols # Number of columns in the maze
		self.num_fires_smol = smol # Number of 1x1 in the maze
		self.num_fires_med = med # Number of 2x2 in the maze
		self.num_fires_lrg = lrg # Number of 3x3 in the maze
		self.num_inside = num_inside # Number of padding inside each cell
		self.num_ent = num_ent # Number of entrances to the maze
		self.plot_maze = False
		[self.maze, self.fires, self.entrances] = generate_maze(self.num_rows, self.num_cols, self.num_fires_smol, self.num_fires_med, self.num_fires_lrg, self.num_inside, self.num_ent, self.plot_maze)
		nrow=len(self.maze)
		ncol=len(self.maze[0])
		self.offset = ((WIDTH-ncol*self.pixel_factor)//2,(HEIGHT-nrow*self.pixel_factor)//2)
		self.wall_thickness=1
		self.robots=[]
	
	# def set_wall_thickness(self,thickness):
	# 	self.wall_thickness=thickness
	def add_bot(self,bot):
		self.robots.append(bot)

	def draw_field(self):
		left = self.offset[0]
		top = self.offset[1]
		nrow=len(self.maze)
		ncol=len(self.maze[0])
		width = ncol*self.pixel_factor
		height = nrow*self.pixel_factor
		borders = pygame.Rect(left,top,width,height)
		pygame.draw.rect(surface=WIN,color=TAN,rect=borders, width=0)

	def draw_obstacles(self):
		(hwalls,vwalls) = get_list_walls(self.maze)
		for wall in hwalls:
			xmin=wall.llim
			xmax=wall.ulim+1
			ymin=wall.row
			ymax=wall.row+self.wall_thickness
			corners=np.array([(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax)])
			pygame.draw.polygon(surface=WIN,color=BLACK,points=self.pixel_factor*corners+self.offset)
		for wall in vwalls:
			ymin=wall.llim
			ymax=wall.ulim+1
			xmin=wall.col
			xmax=wall.col+self.wall_thickness
			corners=np.array([(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax)])
			pygame.draw.polygon(surface=WIN,color=BLACK,points=self.pixel_factor*corners+self.offset)

	def draw_robot(self, bot):
		(x,y,theta,r)=(bot._x,bot._y,bot._theta,bot.radius)
		point_center=np.array((x+0.75*r*cos(theta),y+0.75*r*sin(theta)))
		pygame.draw.circle(surface=WIN,center=self.pixel_factor*np.array((x,y))+self.offset,radius=self.pixel_factor*r,color=bot.color)
		pygame.draw.circle(surface=WIN,center=self.pixel_factor*point_center+self.offset,radius=self.pixel_factor*0.125*r,color=BLACK)

	def draw_window(self):
		# pygame.init()
		WIN.fill(WHITE) # first 
		# print field
		self.draw_field()
		# print obstacles
		self.draw_obstacles()
		# print robots
		for b in self.robots:
			self.draw_robot(b)
		# # print Firetruck
		# self.draw_firetruck()
		pygame.display.update()
def main():
	nrows=4
	ncols=4
	smol=5
	med=3
	lrg=1
	num_inside=10
	num_ent=1
	pixel_factor=20
	sim = Simulation(nrows=nrows,ncols=ncols,smol=smol,med=med,lrg=lrg,num_inside=num_inside,num_ent=num_ent,pixel_factor=pixel_factor)
	start=Node(row=2, col=2)
	theta=0
	x=start.col
	y=start.row
	bot = RRTBot(epsilon= 1, start=Node(row=1, col=1), nrow=nrows, ncol=ncols, color=RED,x=x,y=y,theta=theta)
	sim.add_bot(bot)
	while True:	
		sim.draw_window()
if __name__== "__main__":
	main()