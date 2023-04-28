from data_structure_library import Node, Graph, distance, direction
import math
import random
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import maze_generation
from bot import Bot

class RRTBot(Bot):
	def __init__(self, epsilon, start, nrow, ncol, color,x ,y ,theta) -> None:
		super().__init__(nrow, ncol, color,x,y,theta)
		self.epsilon = 1 #epsilon
		self.current_pos = start
		self.tree = Graph(start)
		# self.collisions = 0
		self.success = 0
	
	# one iteration of the RRT algorithm
	# Input: None
	# Output: None
	def rrt_search(self, hwalls,vwalls, buffer):
		#random sample a node
		frontiers = []
		num_fail=0
		while self.success<100:
			print(self.success)
			q_rand = Node(random.uniform(1.0,self.ncol), random.uniform(1.0,self.nrow))
			while q_rand in self.tree.V:
				q_rand = Node(random.uniform(1.0,self.ncol), random.uniform(1.0,self.nrow))
			#find closest node in tree to random node
			q_curr = self.find_closest_node(q_rand)
			dir = direction(q_curr, q_rand)
			dis = distance(q_curr, q_rand)
			#attempt to grow tree in direction of q_rand a maximum of epsilon
			q_new = Node(row=round(q_curr.row + (dis%self.epsilon)*math.sin(dir),2),col=round(q_curr.col +(dis%self.epsilon)*math.cos(dir),2))
			possible_edge = (q_curr, q_new)
			# if not self.will_collide(possible_edge, hwalls, vwalls) and not self.lies_on_edge(q_new) and not self.too_close(q_new, hwalls, vwalls, buffer):
			if self.local_planner(q_curr,q_new,hwalls,vwalls,buffer=buffer):
				q_new.parent = q_curr
				self.tree.V.append(q_new)
				self.current_pos=q_new
				self.tree.E.append(possible_edge)
				self.success+=1
			# else:
			# 	num_fail+=1
			# 	if num_fail>10000:
			# 		break
		# print("Failures: %d" % num_fail)
		# if num_fail>99:
		# 	self.plot()
		self.success=0
		return frontiers

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
	
	def too_close(self, node, hwalls, vwalls, buffer):
		for wall in hwalls:
			if node.col >= wall.llim-buffer and node.col <= wall.ulim+buffer:
				if node.row >= wall.row-buffer and node.row<=wall.row+buffer:
					#print("Too close!")
					return True
		for wall in vwalls:
			if node.row >= wall.llim-buffer and node.row <= wall.ulim+buffer:
				if node.col>= wall.col-buffer and node.col<=wall.col+buffer:
					#print("Too close!")
					return True
		return False

	# method to determine if a given edge will collide with an obstacle
	# Input: Edge, horizontal wall's list, vertical walls list
	# Output: Boolean (True: Will collide, False: Will NOT collide)
	def will_collide(self,edge, hwalls, vwalls) -> bool:
		# check if vertical edge
		if(edge[1].col == edge[0].col):
			for wall in hwalls:
				l = min(edge[0].row, edge[1].row)
				u = max(edge[0].row, edge[1].row)
				if edge[0].col>=wall.llim and edge[0].col<=wall.ulim:
					if l <= wall.row and u >= wall.row:
						return True
			return False
		#find slope of edge
		m = (edge[1].row-edge[0].row)/(edge[1].col-edge[0].col)
		#check collision with horizontal walls
		for wall in hwalls:
			l = min(edge[0].row, edge[1].row)
			u = max(edge[0].row, edge[1].row)
			if wall.row >= l and wall.row <= u:
				#print("Possible collision!")
				y = wall.row
				y1 = edge[0].row
				x1 = edge[0].col
				point = round((y-y1)/m+x1,2)
				if point >= wall.llim and point <= wall.ulim:
					#print("H Collision!")
					# self.collisions += 1
					return True
		#check collision with vertical walls
		for wall in vwalls:
			l = min(edge[0].col, edge[1].col)
			u = max(edge[0].col, edge[1].col)
			if wall.col >= l and wall.col <= u:
				x = wall.col
				y1 = edge[0].row
				x1 = edge[0].col
				point = round(m*(x-x1)+y1,2)
				if point >= wall.llim and point <= wall.ulim:
					#print("V Collision!")
					return True
		return False

	# method to determine if a node lies on a current edge of the tree
	# Input: node to add to tree
	# Output: Boolean (True: there is an edge that the node would coincide with, False: No edge)
	def lies_on_edge(self, node):
		for edge in self.tree.E:
			# check if vertical edge
			if(edge[1].col == edge[0].col):
				if(node.col==edge[0].col):
					if node.row in np.arange(min(edge[0].row, edge[1].row), max(edge[0].row, edge[1].row)+1, 0.01):
							#print("On edge vertical!")
							return True
			elif node.col in np.arange(min(edge[0].col, edge[1].col), max(edge[0].col, edge[1].col)+1, 0.01):
				# find slope of edge
				m = (edge[1].row-edge[0].row)/(edge[1].col-edge[0].col)
				if node.row - edge[0].row == m*(node.col-edge[0].col):
					#print("On edge!")
					return True
		return False
		
def main():
	# ---- Run Maze Generation code
	num_rows = 3 # Number of rows in the maze
	num_cols = 3 # Number of columns in the maze
	num_fires_smol = 5 # Number of 1x1 in the maze
	num_fires_med = 3 # Number of 2x2 in the maze
	num_fires_lrg = 1 # Number of 3x3 in the maze
	num_inside = 5 # Number of padding inside each cell
	num_ent = 1 # Number of entrances to the maze
	plot_maze = True
	[maze, fires, entrances] = maze_generation.generate_maze(num_rows, num_cols, num_fires_smol, num_fires_med, num_fires_lrg, num_inside, num_ent, plot_maze)
	start = Node(entrances[0][0], entrances[0][1])
	(hwalls, vwalls) = maze_generation.get_list_walls(maze)
	print("rrt main start", start)
	
	#for fire in fires:
	#	print("Fire", fire)
	
	#for h in hwalls:
	#	print(h)
		
	#print(b)
	
	bot = RRTBot(epsilon= 1, start=start, nrow=len(maze)-1, ncol=len(maze[0]), color='cyan', x=0, y=0, theta=0)
	rrt_limit = 500
	buffer = .5
	# while bot.success<rrt_limit:
		# print(bot.success)
	bot.rrt_search(hwalls, vwalls, buffer=buffer)
	
	maze_generation.plot(field=maze,path=None, bot=bot, fires=fires)
	# plt.show()
if __name__ == "__main__":
	main()
