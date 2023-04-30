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

class RRTBot(Bot):
	def __init__(self, epsilon, start, nrow, ncol, color,x ,y ,theta) -> None:
		super().__init__(nrow, ncol, color,x,y,theta)
		self.epsilon = epsilon
		self.current_pos = start
		self.tree = Graph(start)
		# self.collisions = 0
		self.success = 0
		self.frontier_min=3*self.epsilon
		self.frontier_max=8*self.epsilon
		self.visited=[]
		self.fail_counter=0
		self.fire = None
	
	# one iteration of the RRT algorithm
	# Input: None
	# Output: None
	def rrt_search(self, hwalls, vwalls, fires, buffer):
		#random sample a node
		frontiers = []
		pqueue = PriorityQueue()
		num_fail=0
		f=0
		while self.success<100:
			# print(self.success, self.ncol, self.nrow)
			q_rand = Node(random.uniform(0,self.nrow-1), random.uniform(0,self.ncol-1))
			while self.cell(q_rand.col,q_rand.row) in self.visited:
				q_rand = Node(random.uniform(0,self.nrow-1), random.uniform(0,self.ncol-1))
			#find closest node in tree to random node
			q_curr = self.find_closest_node(q_rand)
			dir = direction(q_curr, q_rand)
			dis = distance(q_curr, q_rand)
			# print(dis,dir)
			#attempt to grow tree in direction of q_rand a maximum of epsilon
			# q_new = Node(row=round(q_curr.row + (dis%self.epsilon)*math.sin(dir),2),col=round(q_curr.col +(dis%self.epsilon)*math.cos(dir),2))
			q_new = Node(row=round(q_curr.row + min(dis,self.epsilon)*math.sin(dir),2),col=round(q_curr.col + min(dis,self.epsilon)*math.cos(dir),2))
			# print(min(dis,self.epsilon)*math.sin(dir), min(dis,self.epsilon)*math.cos(dir), min(dis,self.epsilon))
			# print(q_rand,q_new,q_curr)
			possible_edge = (q_curr, q_new)
			(x,y,collision,outside_bounds,frontier,fire)=self.local_planner(q_curr,q_new,hwalls,vwalls,fires,buffer=buffer)
			if not collision:
				q_new.row=y
				q_new.col=x
				q_new.parent = q_curr
				q_new.g = -1*distance(q_new,self.current_pos)
				# pqueue.add(q_new)
				if not outside_bounds:
					self.tree.V.append(q_new)
					self.visited.append(self.cell(q_new.col,q_new.row))
					self.tree.E.append(possible_edge)
					pqueue.add(q_new)
					if frontier:
						f+=1
						print("Frontier",f)
						frontiers.append(q_new)
						if len(frontiers)>5:
							# self.build_path(q_new)
							break
					self.success+=1
				if fire:
					print("Fire!")
					fire.active=False
					self.build_path(q_new)
					break
				# Willy wonky case
				if self.success==99 and len(frontiers)==0:
					print("Wonky")
					q = pqueue.get_min_dist_element()
					frontiers.append(q)
			else:
				self.fail_counter+=1
		self.success=0
		# print(frontiers)
		return frontiers

	def build_path(self,q_new):
		print("Building path")
		self.path=[]
		curr = q_new
		while curr != self.current_pos:
			# print("Building!")
			self.path.append(curr)
			curr=curr.parent
			# print(curr)
	
	def step(self,hwalls,vwalls,fires,buffer):
		t_start = process_time()
		if len(self.path)>0:
			print("on my way")
			self.current_pos = self.path.pop()
			self._x, self._y = (self.current_pos.col, self.current_pos.row)
		else:
			if self.fire:
				self.fire.active=False
				self.fire=None
			print("Running RRT")
			self.tree=Graph(self.current_pos)
			frontiers = self.rrt_search(hwalls=hwalls,vwalls=vwalls,fires=fires,buffer=buffer)
			# if len(self.path)==0:
			# 	self.fail_counter+=1
			if frontiers:
				front = frontiers.pop()
				self.build_path(front)
			# print(self.path)
		t_stop = process_time()
		return t_stop-t_start

	# local planner to test possible path
	def local_planner(self,pos1,pos2,hwalls,vwalls,fires,buffer):
		dir=direction(pos1,pos2)
		dis=distance(pos1,pos2)
		(x,y) = (pos1.col,pos1.row)
		# commands = []
		collision=False
		outside_bounds=False
		frontier=False
		fire_detected=None
		num_steps=dis//self._dt
		num_iter=int(num_steps)
		remainder=num_steps - num_iter
		for i in range(num_iter):
			x+=dis/num_iter*math.cos(dir)
			y+=dis/num_iter*math.sin(dir)
			# y=self.conv(y)
			if self.collision_detect(x,y,hwalls,vwalls,buffer=buffer):
				collision=True
				break
			(fire, detected) = self.fire_detect(x,y,fires,buffer)
			if detected:
				fire_detected=fire
				break
		if not fire_detected and not collision:
			x+=remainder*math.cos(dir)
			y+=remainder*math.sin(dir)
		# y=self.conv(y)
		if self.collision_detect(x,y,hwalls,vwalls,buffer=buffer):
			collision=True
		(fire, detected) = self.fire_detect(x,y,fires,buffer)
		if detected:
			fire_detected=fire
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
	
	def fire_detect(self,x,y,fires,buffer):
		for fire in fires:
			if fire.active:
				not_y,not_x=False,False
				xmin=fire.col-buffer
				xmax=fire.col+fire.size+buffer
				ymin=fire.row-buffer
				ymax=fire.row+fire.size+buffer
				cx = 0.5*(xmin+xmax)
				cy = 0.5*(ymin+ymax)
				rx=abs(0.5*(xmax-xmin))
				ry=abs(0.5*(ymax-ymin))
				if abs(y-cy)>ry+self.radius:
					not_y=True
				if abs(x-cx)>rx+self.radius:
					not_x=True
				if not(not_x or not_y):
					print("Fire detected!")
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
	plot_maze = True
	[maze, fires, entrances] = maze_generation.generate_maze(num_rows=num_rows, num_cols=num_cols, num_fires_smol=num_fires_smol, num_fires_med=num_fires_med, num_fires_lrg=num_fires_lrg, num_inside=num_inside, num_ent=num_ent, plot_maze=plot_maze)
	start = Node(entrances[0][0], entrances[0][1])
	(hwalls, vwalls) = maze_generation.get_list_walls(maze)
	print("rrt main start", start)
	
	bot = RRTBot(epsilon=0.5, start=start, nrow=len(maze)-1, ncol=len(maze[0]), color='cyan', x=start.col, y=num_rows-start.row-1, theta=0)
	rrt_limit = 500
	buffer = bot.radius * 1.1
	# while bot.success<rrt_limit:
		# print(bot.success)
	while True:
		bot.step(hwalls, vwalls, fires, buffer=buffer)
		if len(bot.path)<1:
			maze_generation.plot(field=maze,path=None, bot=bot, fires=fires)
	# plt.show()
if __name__ == "__main__":
	main()
