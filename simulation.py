from math import sin, cos, tan, radians, degrees
# import mypylib.myfunctions as mf
from data_structure_library import *
from maze_generation import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random
import pygame
from rrt_robot import RRTBot
from a_star import ABot
from mapper_robot import *
from fire_fighting_robot import *
from time import process_time

# Pygame constants and inits
WIDTH, HEIGHT = 1000,1000
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
FPS = 30
# WUMPUS_IMAGE = pygame.image.load('wumpus.png')
# WUMPUS = pygame.transform.scale(WUMPUS_IMAGE,(25,25))
pygame.display.set_caption("First Line Robust Automatic Aviators: Two Blind Mice")

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
		[self.maze, self.fires, self.entrances] = generate_maze(num_rows=self.num_rows, num_cols=self.num_cols, num_fires_smol=self.num_fires_smol, num_fires_med=self.num_fires_med, num_fires_lrg=self.num_fires_lrg, num_inside=self.num_inside, num_ent=self.num_ent, plot_maze=self.plot_maze)
		self.hwalls,self.vwalls= get_list_walls(self.maze)
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
		(x,y,theta,r)=(bot._x+0.5,bot._y+0.5,bot._theta,bot.radius)
		point_center=np.array((x+0.75*r*cos(theta),y+0.75*r*sin(theta)))
		pygame.draw.circle(surface=WIN,center=self.pixel_factor*np.array((x,y))+self.offset,radius=self.pixel_factor*r,color=bot.color)
		pygame.draw.circle(surface=WIN,center=self.pixel_factor*point_center+self.offset,radius=self.pixel_factor*0.125*r,color=BLACK)
		if type(bot) == RRTBot:
			pygame.draw.circle(surface=WIN,center=self.pixel_factor*np.array((x,y))+self.offset,radius=bot.frontier_min*self.pixel_factor, width=1,color=BLACK)
			for edge in bot.tree.E:
				x_arr = self.pixel_factor*np.array([edge[0].col, (edge[0].row)])+self.offset+(0.5,0.5)
				y_arr = self.pixel_factor*np.array([edge[1].col, (edge[1].row)])+self.offset+(0.5,0.5)
				points=[x_arr,y_arr]
				pygame.draw.lines(surface=WIN,color=BLACK,points=points,closed=False)
		if type(bot) == ABot:
			nice = 69

	def draw_fires(self):
		for fire in self.fires:
			top=self.pixel_factor*(fire.row-fire.size+1)+self.offset[0]
			left=self.pixel_factor*(fire.col)+self.offset[1]
			w=self.pixel_factor*(fire.size)
			r=pygame.Rect(left,top,w,w)
			if fire.found:
				pygame.draw.rect(surface=WIN,rect=r,color=GREEN)
			elif fire.active:
				pygame.draw.rect(surface=WIN,rect=r,color=RED)
			# else:
			# 	pygame.draw.rect(surface=WIN,rect=r,color=TAN)

	def draw_window(self):
		# pygame.init()
		WIN.fill(WHITE) # first 
		# print field
		self.draw_field()
		# print obstacles
		self.draw_obstacles()
		# draw fires
		self.draw_fires()
		# print robots
		for b in self.robots:
			self.draw_robot(b)
		pygame.display.update()

	def run(self):#, simtime):
		pygame.init()
		clock = pygame.time.Clock()
		run=True
		done = False
		while run:
			clock.tick(FPS)
			for event in pygame.event.get():
				if event.type==pygame.QUIT:
					run=False
			self.draw_window()
			print(self.robots[0].goal)
			print(self.robots[0].destination)
			#done2 = self.robots[0].step(hwalls=self.hwalls,vwalls=self.vwalls,fires=self.fires,buffer=0.1)
			done = self.robots[1].step()
			if done:
				self.robots[1].goal = None
				self.robots[1].destination = None
				print("Done!")
			#if done2:
			#	self.robots[0].goal = None
			#	self.robots[0].destination = None
			if self.robots[0].fire:
				# Got to A* pos
				print("Hey!", distance(self.robots[0].current_pos, Node(self.robots[1]._y, self.robots[1]._x)))
				print(self.robots[0].current_pos, Node(self.robots[1]._y, self.robots[1]._x))
				if distance(self.robots[0].current_pos, Node(self.robots[1]._y, self.robots[1]._x)) < 2 and self.robots[1].goal = None:
					self.robots[1].goal = (self.robots[0].fire.row, self.robots[0].fire.col)
					self.robots[0].fire=None
		pygame.quit()

def main():
	nrows=4
	ncols=4
	smol=5
	med=3
	lrg=1
	num_inside=8
	num_ent=1
	pixel_factor=20
	sim = Simulation(nrows=nrows,ncols=ncols,smol=smol,med=med,lrg=lrg,num_inside=num_inside,num_ent=num_ent,pixel_factor=pixel_factor)
	start = Node(sim.entrances[0][0], sim.entrances[0][1])
	x=start.col
	y=start.row
	theta=sim.entrances[0][2]
	bert = RRTBot(epsilon=0.5, start=start, nrow=len(sim.maze), ncol=len(sim.maze[0]), color=GREEN,x=x-1,y=y,theta=theta)
	terminator = ABot(len(sim.maze), len(sim.maze[0]), color=BLUE, x=x, y=y, theta=sim.entrances[0][2]) 
	terminator.loc = start
	terminator.big_maze = sim.maze
	
	sim.add_bot(bert)
	sim.add_bot(terminator)
	sim.run()
	# while True:
	# 	sim.robots[0].step(sim.hwalls,sim.vwalls,sim.fires,0.1)	
		# sim.draw_window()
if __name__== "__main__":
	main()
