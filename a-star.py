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
	pqueue = PriorityQueue()
	nodes = generate_list_nodes(end, maze)
	start_node = find_node(start[0], start[1], nodes)
	start_node.g = 0
	end_node = find_node(end[0], end[1], nodes)
	path = []
	current_node = start_node
	pqueue.add(start_node)
	# A Star loop
	while len(pqueue.q) > 0:
		current_node = pqueue.get_min_dist_element()
		if current_node == end_node:
			break
		neighbors = get_neighbors(current_node, maze, nodes)
		for n in neighbors:
			pqueue.add(n)
	
	while current_node != start_node:
		path.append(current_node)
		current_node = current_node.parent
	path.append(current_node)
	return path


def generate_list_nodes(end, maze):
	num_rows = len(maze)
	num_cols = len(maze[0])
	nodes = []
	for i in range(num_rows):
		for j in range(num_cols):
			if maze[i][j]:
				node = Node(i, j)
				node.h = math.dist([i, j], end)
				nodes.append(node)
	return nodes


def get_neighbors(node, field, nodes):
	neighbors = []
	row_mod = [-1, -1, -1, 0, 0, 1, 1, 1]
	col_mod = [-1, 0, 1, -1, 1, -1, 0, 1]
	for i in range(8):
		row = node.row + row_mod[i]
		col = node.col + col_mod[i]
		if field[row][col]:
			neighbor = find_node(row, col, nodes)
			if node.g + 1 < neighbor.g or neighbor.g == math.inf:
				neighbor.g = node.g + 1
				neighbor.f = neighbor.g + neighbor.h
				neighbor.parent = node
				neighbors.append(neighbor)
	return neighbors
	
	
def find_node(row, col, nodes):
	for node in nodes:
		if node.row == row and node.col == col:	
			return node
	return 0


def main():
	print("A Star Main\n")
	# ---- Run Maze Generation code
	num_rows = 10 # Number of rows in the maze
	num_cols = 10 # Number of columns in the maze
	num_fires_smol = 5 # Number of 1x1 in the maze
	num_fires_med = 3 # Number of 2x2 in the maze
	num_fires_lrg = 1 # Number of 3x3 in the maze
	num_inside = 5 # Number of padding inside each cell
	num_ent = 0 # Number of entrances to the maze
	plot_maze = False
	maze = generate_maze(num_rows, num_cols, num_fires_smol, num_fires_med, num_fires_lrg, num_inside, num_ent, plot_maze)
	# ---- Run A*
	start = [1, 1]
	end = [len(maze) - 2, len(maze[0]) - 2]
	path = a_star(start, end, maze)
	plot(maze, path)

if __name__ == "__main__":
	main()
