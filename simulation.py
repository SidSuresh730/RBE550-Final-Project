import random
import pygame
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, tan, radians, degrees
from matplotlib.animation import FuncAnimation
from data_structure_library import *
from maze_generation import *
from rrt_robot import RRTBot
from a_star import ABot
from time import process_time,sleep
import sys
# Pygame constants and inits
WIDTH, HEIGHT = 1000,1000
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
FPS = 30
pygame.display.set_caption("First Line Robust Automatic Aviators: Two Blind Mice")

# Simulation class that controls the bots and maze. We initialize it with a maze when generaterd then create the bots separately and place them inside the sim's .robot variable
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
		# Generate maze parameters and store them when the Simulation object is created
		[self.maze, self.fires, self.entrances] = generate_maze(num_rows=self.num_rows, num_cols=self.num_cols, num_fires_smol=self.num_fires_smol, num_fires_med=self.num_fires_med, num_fires_lrg=self.num_fires_lrg, num_inside=self.num_inside, num_ent=self.num_ent, plot_maze=self.plot_maze)
		# Store the walls for collision detection
		self.hwalls,self.vwalls= get_list_walls(self.maze)
		nrow=len(self.maze)
		ncol=len(self.maze[0])
		self.offset = ((WIDTH-ncol*self.pixel_factor)//2,(HEIGHT-nrow*self.pixel_factor)//2)
		self.wall_thickness=0.1
		self.robots=[]
	
	# Add a bot to the class. Used mainly for clarity / orginization
	# Input: Bot, the respective bot class to add to the Sim
	# Output: None, but the class variable is updated
	def add_bot(self,bot):
		self.robots.append(bot)

	# Generate the visual of the field in pygame
	def draw_field(self):
		left = self.offset[0]
		top = self.offset[1]
		nrow=len(self.maze)-1
		ncol=len(self.maze[0])-1
		width = ncol*self.pixel_factor
		height = nrow*self.pixel_factor
		borders = pygame.Rect(left,top,width,height)
		pygame.draw.rect(surface=WIN,color=TAN,rect=borders, width=0)

	# Draw the walls in pygame as polygons
	def draw_obstacles(self):
		(hwalls,vwalls) = get_list_walls(self.maze)
		for wall in hwalls:
			xmin=wall.llim
			xmax=wall.ulim
			ymin=wall.row-self.wall_thickness
			ymax=wall.row+self.wall_thickness
			corners=np.array([(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax)])
			pygame.draw.polygon(surface=WIN,color=BLACK,points=self.pixel_factor*corners+self.offset)
		for wall in vwalls:
			ymin=wall.llim
			ymax=wall.ulim
			xmin=wall.col-self.wall_thickness
			xmax=wall.col+self.wall_thickness
			corners=np.array([(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax)])
			pygame.draw.polygon(surface=WIN,color=BLACK,points=self.pixel_factor*corners+self.offset)

	# Draw the fires in pygane
	def draw_fires(self):
		for fire in self.fires:
			top=self.pixel_factor*(fire.row-fire.size)+self.offset[0]
			left=self.pixel_factor*(fire.col)+self.offset[1]
			w=self.pixel_factor*(fire.size)
			r=pygame.Rect(left,top,w,w)
			# Once we find the robot we can redraw it as the same color as the field, once it's found by the RRT bot we can switch the color to green
			if not fire.active:
				pygame.draw.rect(surface=WIN,rect=r,color=TAN)
			elif fire.found:
				pygame.draw.rect(surface=WIN,rect=r,color=DARK_GREEN)
			else:
				pygame.draw.rect(surface=WIN,rect=r,color=RED)

	# Draw the robots in pygame
	# The RRT bot is a cyan circle with a black circle around it showing the frontier range
	# The A* bot is a blue circle
	def draw_robot(self, bot):
		(x,y,theta,r)=(bot._x,bot._y,bot._theta,bot.radius)
		point_center=np.array((x+5*r*cos(theta),y+2*r*sin(theta)))
		point=np.array((x,y))
		pygame.draw.circle(surface=WIN,center=self.pixel_factor*np.array((x,y))+self.offset,radius=self.pixel_factor*r,color=bot.color)
		# Need extra set up for the RRT bot "sensor" range and RRT 
		if type(bot) == RRTBot:
			(x,y,theta,r)=(bot.current_pos.col,bot.current_pos.row,bot._theta,bot.radius)
			pygame.draw.circle(surface=WIN,center=self.pixel_factor*np.array((x,y))+self.offset,radius=bot.frontier_min*self.pixel_factor, width=1,color=BLACK)
			for edge in bot.tree.E:
				x_arr = self.pixel_factor*np.array([edge[0].col, (edge[0].row)])+self.offset
				y_arr = self.pixel_factor*np.array([edge[1].col, (edge[1].row)])+self.offset
				points=[x_arr,y_arr]
				pygame.draw.lines(surface=WIN,color=BLACK,points=points,closed=False)
		# Placeholder to hold other bot things to draw
		if type(bot) == ABot:
			pass

	# Function to draw all field parameters 
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

	# Main run function. Handles robot communication and stepping the robots when appropriate
	def run(self,simtime):#, simtime):
		pygame.init()
		clock = pygame.time.Clock()
		time = 0
		rrt_time = 0
		astar_time = 0
		min_bot_ineract_dist = 3
		done = False
		all_out = False
		
		while time<simtime:
			time+=1
			clock.tick(FPS)
			for event in pygame.event.get():
				if event.type==pygame.QUIT:
					time=simtime
			# Draw everything needed every cycle
			self.draw_window()
			# Step the RRT bot by giving it the needed params
			rrt_time+=self.robots[0].step(hwalls=self.hwalls,vwalls=self.vwalls,fires=self.fires,buffer=0.1)
			# Step the A* bot, all params are contained within the bot object
			(done,t) = self.robots[1].step()
			astar_time+=t
			# If the A* bot has reached it's goal, clear it's goal and destination
			if done:
			 	self.robots[1].goal = None
			 	self.robots[1].destination = None
			# Inter robot comms here. If the RRT bot has found a fire, it's knowledge of A*'s position
			if self.robots[0].fire:
				# If the robots are close enough pass the fire from RRT to A* and clear RRT's fire variable. This code can be examined in the future
				if distance(self.robots[0].current_pos, Node(self.robots[1]._y, self.robots[1]._x)) < min_bot_ineract_dist and self.robots[1].goal == None:
					self.robots[1].goal = (self.robots[0].fire.row, self.robots[0].fire.col)
					self.robots[1].fire = self.robots[0].fire
					self.robots[0].fire=None
		self.get_metrics(rrt_time,astar_time)
		pygame.quit()
	
	# write metrics to file for data analysis
	def get_metrics(self,rrt_time,astar_time):
		num_active = 0
		num_fires = len(self.fires)
		for fire in self.fires:
			if fire.active:
				num_active += 1
			print("Active Fires: %d, Total Fires: %d" % (num_active,num_fires))
		f = open("test_data.csv","a")
		f.write("RRT Time: %.2f, AStar Time: %.2f\n" % (rrt_time,astar_time))
		f.write("Active Fires: %d, Total Fires: %d\n" % (num_active,num_fires))
		f.close()

def main(argv):
	if len(argv)<8:
		# Default Maze params and generation
		nrows=3
		ncols=3
		smol=5
		med=3
		lrg=1
		num_inside=4
		simtime=1200
	else:
		nrows=int(argv[1])
		ncols=int(argv[2])
		smol=int(argv[3])
		med=int(argv[4])
		lrg=int(argv[5])
		num_inside=int(argv[6])
		simtime=int(argv[7])
	num_ent=1
	pixel_factor=int(WIDTH/(nrows*(num_inside+5)))
	f = open("test_data.csv","a")
	f.write("Simtime: %d, nrows: %d, ncols: %d, smol: %d, med: %d, lrg: %d, numinside: %d\n" % (simtime,nrows,ncols,smol,med,lrg,num_inside))
	f.close()
	sim = Simulation(nrows=nrows,ncols=ncols,smol=smol,med=med,lrg=lrg,num_inside=num_inside,num_ent=num_ent,pixel_factor=pixel_factor)
	# Robot params based on the maze generated
	start = Node(sim.entrances[0][0], sim.entrances[0][1])
	x=start.col
	y=start.row
	theta=sim.entrances[0][2]
	# Generate the RRT bot (RRT became ert became Bert)
	bert = RRTBot(epsilon=0.3, start=start, nrow=len(sim.maze), ncol=len(sim.maze[0]), color=CYAN,x=x-1,y=y,theta=theta)
	# Generate the A* bot and provide it with it's initial positions to use within the step function
	terminator = ABot(len(sim.maze), len(sim.maze[0]), color=BLUE, x=x, y=y, theta=sim.entrances[0][2]) 
	terminator.loc = start
	terminator.maze = sim.maze
	# Add the bots to the sim and start er up
	sim.add_bot(bert)
	sim.add_bot(terminator)

	sim.run(simtime)

if __name__== "__main__":
	main(argv=sys.argv)
