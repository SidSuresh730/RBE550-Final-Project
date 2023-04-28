from data_structure_library import Node, PriorityQueue, distance, generate_list_nodes, Graph, get_neighbors, find_node
import math
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import maze_generation
from bot import Bot


class MapperBot(Bot):
	def __init__(self, nrow, ncol, color) -> None:
		super().__init__(nrow, ncol, color)
		self.nodes = None

	def djikstra_map(self, s, maze):
		# dist = {}
		# prev = {}
		self.tree = Graph(s)
		queue = PriorityQueue()
		self.nodes = generate_list_nodes(maze=maze, end=None)
		s.g = 0
		queue.add(s)
		# While queue not empty
		while len(queue.q)>0:
			# self.dji_iterations+=1 
			u = queue.get_min_dist_element() #get minimum dist element and remove from queue
			# self.tree.V.append(u)
			neighbors = get_neighbors(u, maze, self.nodes, self.tree)
			for v in neighbors: # update neighbor distances as necessary
				# print("There are neighbors")
				# if v in queue.q:
				#	 print('v in q')
				#	 alt = u.g + distance(v, u)
				#	 print(alt)
				#	 if alt<v.g:
				#		 print("Shorter path found")
				# self.tree.remove_edge((v.parent, v))
				#		 v.g = alt
				#		 v.parent = u
				# self.tree.E.append((u,v))
				queue.add(v)

	def path_to_node(self, start, goal):
		path_node = find_node(goal.row, goal.col, self.nodes)
		path = list()
		while path_node != start:
			path.append(path_node)
			path_node = path_node.parent
		path.append(path_node)
		return path
			


		# while v != s:
		#	 path.append(current_node)
		#	 current_node = current_node.parent
		# path.append(current_node)
		# return path

	# def conv(self, nrow, row):
	#	 return nrow - row

	# def plot(self, nrow):
	#	 print('Plotting Graph: Dji')
	#	 # plot vertices
	#	 for node in self.tree.V:
	#		 plt.plot(node.col, nrow-node.row, 'rx')
	#	 for edge in self.tree.E:
	#		 x_arr = [edge[0].col, edge[1].col]
	#		 y_arr = [self.conv(nrow, edge[0].row), self.conv(nrow, edge[1].row)]
	#		 plt.plot(x_arr, y_arr, color='red', linestyle="--")
	#	 # plt.show()

# def plot(field, bot):
#	 print("Plotting")
#	 num_rows = len(field)
#	 num_cols = len(field[0])
#	 bot.plot()
# 	# Plot all occupancy grid locations
#	 for j in range(num_rows): 
#		 for i in range(num_cols):
#			 if field[j, i] == 0: # Wall
#				 plt.plot(i, num_rows - j - 1, 'kx')
#			 elif field[j, i] == 1: # Empty
#				 plt.plot(i, num_rows - j - 1, 'bx')
#			 elif field[j, i] == 2: # Fire
#				 plt.plot(i, num_rows - j - 1, 'rx')
#			 elif field[j, i] == 3: # Entrance
#				 plt.plot(i, num_rows - j - 1, 'gx')
#			 else:
#				 plt.plot(i, num_rows - j - 1, 'b.')
# 	# Plot horizontal walls
#	 for j in range(num_rows): 
#		 for i in range(num_cols - 1):
#			 if not field[j, i] and not field[j, i+1]:
#				 line = np.array([[j, i], [j, i+1]])
#				 plt.plot(line[:, 1], num_rows - 1 - line[:, 0], 'k-')				
# 	# Plot vertical walls
#	 for j in range(num_rows - 1): 
#		 for i in range(num_cols):
#			 if not field[j, i] and not field[j+1, i]:
#				 line = np.array([[j, i], [j+1, i]])
#				 plt.plot(line[:, 1], num_rows - 1 - line[:, 0], 'k-')
	
#	 plt.axis([-1, num_cols, -1, num_rows])
#	 plt.title("Maze")
#	 plt.show()

def main():
	print("Dij Main\n")
	# ---- Run Maze Generation code
	size_maze = 4
	num_rows = size_maze # Number of rows in the maze
	num_cols = size_maze # Number of columns in the maze
	num_fires_smol = 1 # Number of 1x1 in the maze
	num_fires_med = 1 # Number of 2x2 in the maze
	num_fires_lrg = 0 # Number of 3x3 in the maze
	num_inside = 2 # Number of padding inside each cell
	num_ent = 1 # Number of entrances to the maze
	plot_maze = False
	
	
	[maze, big_maze, fires, entrances] = maze_generation.generate_maze(num_rows, num_cols, num_fires_smol, num_fires_med, num_fires_lrg, num_inside, num_ent, plot_maze)
	# ---- Run Dij
	# start = [1, 1]
	#start = Node(entrances[0][0], entrances[0][1])
	start = Node(0, 0)
	final_node = Node(3, 3)
	bot = MapperBot(len(maze)-1, len(maze[0]), 'blue')
	bot.djikstra_map(start, maze)
	
	path = bot.path_to_node(start, final_node)
	
	maze_generation.plot(field=maze, path=path, bot=bot)
	
	#bot.djikstra_map(Node(5, 5), maze)
	#path = bot.path_to_node(Node(5, 5), Node(1, 3))
	#maze_generation.plot(field=maze, path=path, bot=bot)

if __name__ == "__main__":
	main()
