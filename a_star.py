import random
import numpy as np
import matplotlib.pyplot as plt # Plotting our obstacle grid
import math
from matplotlib.path import Path # Plotting our search algorithm path
import maze_generation
from data_structure_library import *
from bot import Bot
# g: Start node to node n
# h: node n to end node
# g + h = f

class ABot(Bot):
	def __init__(self, nrow, ncol, color, x, y, theta) -> None:
		super().__init__(nrow, ncol, color, x, y, theta)


	def a_star(self, start, end, maze):
		pqueue = PriorityQueue()
		nodes = generate_list_nodes(maze=maze, end=end)
		start_node = find_node(start[0], start[1], nodes)
		start_node.g = 0
		self.tree = Graph(start_node)
		end_node = find_node(end[0], end[1], nodes)
		path = []
		current_node = start_node
		pqueue.add(start_node)
		# A Star loop
		while len(pqueue.q) > 0:
			current_node = pqueue.get_min_dist_element()
			if current_node == end_node:
				break
			neighbors = get_neighbors(current_node, maze, nodes, self.tree)
			for n in neighbors:
				pqueue.add(n)
		
		while current_node != start_node:
			path.append(current_node)
			current_node = current_node.parent
		path.append(current_node)
		path.reverse()
		
		for p in path:
			print("P", p)
		
		cool_path = self.local_plan(path, maze)
		
		#while len(pqueue.q) > 0:
		#	v = pqueue.get_min_dist_element()
		#	self.tree.remove_vertex(v)
		#	self.tree.remove_edge((v.parent, v))	
		return path

	def local_plan(self, path, maze):
		print("Cool!")
		

def main():
	print("A Star Main\n")
	# ---- Run Maze Generation code
	num_rows = 4 # Number of rows in the maze
	num_cols = 4 # Number of columns in the maze
	num_fires_smol = 5 # Number of 1x1 in the maze
	num_fires_med = 3 # Number of 2x2 in the maze
	num_fires_lrg = 1 # Number of 3x3 in the maze
	num_inside = 10 # Number of padding inside each cell
	num_ent = 1 # Number of entrances to the maze
	plot_maze = False
	[maze, big_maze, fires, entrances] = maze_generation.generate_maze(num_rows, num_cols, num_fires_smol, num_fires_med, num_fires_lrg, num_inside, num_ent, plot_maze)
	
	
	# ---- Run A*
	bot = ABot(len(big_maze)-1, len(big_maze[0]), color="cyan", x=0, y=0, theta=0)
	start = entrances[0]
	end = [1, 1]
	path = bot.a_star(start, end, big_maze)

	maze_generation.plot(big_maze, path, bot, fires)

if __name__ == "__main__":
	main()
