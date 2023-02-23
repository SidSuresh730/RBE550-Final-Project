import random
import numpy as np
import matplotlib.pyplot as plt # Plotting our obstacle grid
import math
from matplotlib.path import Path # Plotting our search algorithm path
from maze_generation import *
from data_structure_library import *

# g: Start node to node n
# h: node n to end node
# g + h = f

def a_star(start, end, maze):
	Pqueue = PriorityQueue()
	visited = []
	nodes = generate_list_nodes(end, maze)
	path = []
	Start = Node(start[0], start[1])
	Start.g = 0
	Start.h = 0
	Start.f = 0
	current_node = Start
	prev_node = Start
	End = Node(end[0], end[1])
	#Pqueue.add(End)
	Pqueue.add(Start)
	while len(Pqueue.q) > 0:
		prev_node = current_node
		current_node = Pqueue.get_min_dist_element()
		if current_node.row == End.row and current_node.col == End.col:
			break
		neighbors = get_neighbors(current_node, maze, nodes)
		for n in neighbors:
			Pqueue.add(n)
	
	while current_node != Start:
		path.append(current_node)
		current_node = current_node.parent
	path.append(current_node)
	return path


# def generate_list_nodes(end, maze):
# 	num_rows = len(maze)
# 	num_cols = len(maze[0])
# 	nodes = []
# 	for i in range(num_rows):
# 		for j in range(num_cols):
# 			if maze[i][j]:
# 				node = Node(i, j)
# 				node.h = math.dist([i, j], end)
# 				node.f = node.g + node.h
# 				nodes.append(node)
# 	return nodes


# def get_neighbors(node, field, nodes):
# 	neighbors = []
# 	row_mod = [-1, -1, -1, 0, 0, 1, 1, 1]
# 	col_mod = [-1, 0, 1, -1, 1, -1, 0, 1]
# 	for i in range(8):
# 		row = node.row + row_mod[i]
# 		col = node.col + col_mod[i]
# 		if field[row][col]:
# 			Neighbor = find_node(row, col, nodes)
# 			if node.g + 1 < Neighbor.g:
# 				Neighbor.g = node.g + 1
# 				Neighbor.f = Neighbor.g + Neighbor.h
# 				Neighbor.parent = node
# 				neighbors.append(Neighbor)
# 	return neighbors
	
	
# def find_node(row, col, nodes):
# 	for node in nodes:
# 		if node.row == row and node.col == col:	
# 			return node
# 	return 0


def main():
	print("A Star Main\n")
	# ---- Run Maze Generation code
	num_rows = 10 # Number of rows in the maze
	num_cols = 10 # Number of columns in the maze
	num_fires_smol = 0 # Number of 1x1 in the maze
	num_fires_med = 0 # Number of 2x2 in the maze
	num_fires_lrg = 0 # Number of 3x3 in the maze
	num_inside = 5 # Number of padding inside each cell
	num_ent = 0 # Number of entrances to the maze
	plot_maze = False
	maze = generate_maze(num_rows, num_cols, num_fires_smol, num_fires_med, num_fires_lrg, num_inside, num_ent, plot_maze)
	# ---- Run A*
	start = [1, 1]
	end = [len(maze) - 2, len(maze[0]) - 2]
	path = a_star(start, end, maze)
	plot(maze, path)
	print(start)
	print(end)
	# ---- Test Code
	#field = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]])
	#Test_Node = Node(1, 1)
	#neighbors = get_neighbors(Test_Node, field)
	#for n in neighbors:
	#	print(n)

if __name__ == "__main__":
	main()
