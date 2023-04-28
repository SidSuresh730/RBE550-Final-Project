import random
import sys
import numpy as np
import matplotlib.pyplot as plt # Plotting our obstacle grid
import matplotlib.patches as patches
from matplotlib.path import Path # Plotting our search algorithm path
from math import inf # Used for infinity

from data_structure_library import Node, Fire, VWall, HWall

empty_list = np.array([])


#class Node:
#	def __init__(self, row, col, connected):
#		self.row = row
#		self.col = col
#		self.connected = connected
#		
#	def Connect(self, node):
#		self.connected = np.append(self.connected, node)
#	def __str__(self):
#		print(self.row, self.col)
#		for node in self.connected:
#			print(self.row, self.col, "-", node.row, node.col)
#		return "Node printed"

# Given an occupancy grid turn it into a non looping maze
def random_kruskal_maze(field):
	num_rows = len(field)
	num_cols = len(field[0])
	nodes = []
	walls = []
	vwalls = []
	hwalls = []
	#final_walls = []
	#Create a list of the nodes
	for i in range(num_rows):
		for j in range(num_cols):
			if field[i][j]:
				node = Node(i, j)
				nodes.append(node)		
	# Create a list of the walls between the Nodes
	for i in range(1, num_rows - 1):
		for j in range(1, num_cols - 1):	
			if i % 2 != j % 2:
				walls.append(np.array([i, j]))
	# Perform Kruskal's, randomly iterate, check if connected already, if not remove wall
	random.shuffle(walls)
	while walls:
		check_wall = walls.pop()
		[Node1, Node2] = check_nodes = get_separated(check_wall, nodes, field)
		nodes_connected = Node1 in Node2.connected
		if not nodes_connected:
			field[check_wall[0]][check_wall[1]] = 1
			connect_nodes(Node1, Node2)
	return field
	

# Given location of a node and a list of Nodes, return the full node from the list
def find_node(row, col, nodes):
	for i in range(len(nodes)):
		if nodes[i].row == row and nodes[i].col == col:
			return nodes[i]
	return 0 
		

# Return the nodes separated by a wall	
# Since the wall is effectively just a point in the gridm we need to check if the nodes are above and below or left and right			
def get_separated(wall, nodes, field):
	row = wall[0]
	col = wall[1]
	if field[row][col - 1] and field[row][col + 1]: # If left and right values are unoccupied, those are the nodes and return them
		return [find_node(row, col - 1, nodes), find_node(row, col + 1, nodes)]
	elif field[row - 1][col] and field[row + 1][col]: # If above and below values are unoccupied, those are the nodes and return them
		return [find_node(row- 1, col, nodes), find_node(row + 1, col, nodes)]
	else:
		return [0] # Panic


def connect_nodes(node1, node2):
	for node1_con in node1.connected:
		for node2_con in node2.connected:
			if node1_con not in node2_con.connected and node2_con not in node1_con.connected:
				node1_con.Connect(node2_con)
				node2_con.Connect(node1_con)


def get_list_walls(maze):
	hwalls = []
	vwalls = []
	start_point = []
	end_point = []
	start_true = False
	for i in range(len(maze)): 
		for j in range(len(maze[0])):
			if not maze[i, j]:
				if not start_true:
					start_point = [i, j]
					start_true = True
				end_point = [i, j]
			else:
				if start_point != end_point:
					Hwall = HWall(start_point[0], start_point[1], end_point[1])
					hwalls.append(Hwall)
				start_true = False
				start_point = []
				end_point = []
		if start_point != end_point:
			Hwall = HWall(start_point[0], start_point[1], end_point[1])
			hwalls.append(Hwall)
		start_true = False
		start_point = []
		end_point = []
	for j in range(len(maze[0])): 
		for i in range(len(maze)):
			if not maze[i, j]:
				if not start_true:
					start_point = [i, j]
					start_true = True
				end_point = [i, j]
			else:
				if start_point != end_point:
					Vwall = VWall(start_point[1], start_point[0], end_point[0])
					vwalls.append(Vwall)
				start_true = False
				start_point = []
				end_point = []
		if start_point != end_point:
			Vwall = VWall(start_point[1], start_point[0], end_point[0])
			vwalls.append(Vwall)
		start_true = False
		start_point = []
		end_point = []
	return (hwalls, vwalls)


def generate_fires(maze, num_fires_smol, num_fires_med, num_fires_lrg):
	num_rows = len(maze)
	num_cols = len(maze[0])
	med_size = 2
	lrg_size = 3
	fires = []
	# Generate 1x1 fires
	fires_made = 0
	while fires_made < num_fires_smol:
		fire_row = random.randint(1, num_rows-1)
		fire_col = random.randint(1, num_rows-1)
		if maze[fire_row][fire_col] == 1:
			maze[fire_row][fire_col] = 2
			fires_made += 1
			fires.append(Fire(fire_row, fire_col, 1))
	# Generate 2x2 fires
	fires_made = 0
	while fires_made < num_fires_med:
		count = 0
		fire_row = random.randint(0, num_rows-2)
		fire_col = random.randint(0, num_rows-2)
		for i in range(med_size): 
			for j in range(med_size):
				if maze[fire_row - i][fire_col + j] == 1:
					count +=1
		if count >= med_size*med_size:
			for i in range(med_size): 
				for j in range(med_size):
					maze[fire_row - i][fire_col + j] = 2
			fires_made += 1
			fires.append(Fire(fire_row, fire_col, med_size))
	# Generate 3x3 fires
	fires_made = 0
	while fires_made < num_fires_lrg:
		count = 0
		fire_row = random.randint(0, num_rows-2)
		fire_col = random.randint(0, num_rows-2)
		for i in range(lrg_size): 
			for j in range(lrg_size):
				if maze[fire_row + i - 1][fire_col + j - 1] == 1:
					count +=1
		if count >= lrg_size*lrg_size:
			for i in range(lrg_size): 
				for j in range(lrg_size):
					maze[fire_row + i - 1][fire_col + j - 1] = 2
			fires_made += 1
			fires.append(Fire(fire_row, fire_col, lrg_size))
	return fires
			

def maze_expansion(maze, num_inside):
	num_rows = len(maze)
	num_cols = len(maze[0])
	num_ex_rows = int(((num_rows-1)/2)*(num_inside+1)+1)
	num_ex_cols = int(((num_cols-1)/2)*(num_inside+1)+1)
	expanded_maze = np.zeros((num_ex_rows, num_ex_cols))
	for i in range(num_rows): 
		for j in range(num_cols):
			if maze[i, j] == 1:
				for k in range(num_inside):
					for m in range(num_inside):
						expanded_maze[int(((i-1)/2)*(num_inside+1)+1) + k, int(((j-1)/2)*(num_inside+1)+1) + m] = 1
	return expanded_maze
	

def generate_entrances(maze, num_ent):
	num_rows = len(maze)
	num_cols = len(maze[0])
	entrances = []
	for i in range(num_ent):
		ent_loc = random.randint(1, 4)
		n = random.randint(1, num_rows-2)
		if ent_loc == 1: # Top side
			maze[0][n] = 3
			entrances.append([0, n])
		if ent_loc == 2: # Bott side
			maze[num_rows-1][n] = 3
			entrances.append([num_rows-1, n])
		if ent_loc == 3: # Left side
			maze[n][0] = 3
			entrances.append([n, 0])
		if ent_loc == 4: # Right side
			maze[n][num_cols-1] = 3
			entrances.append([n, num_cols-1])
	return entrances
		
def plot(field, path, bot, fires):
	print("Plotting")
	num_rows = len(field)
	num_cols = len(field[0])
	ax = plt.axes()
	ax.set_facecolor("gray")
	# Plot all occupancy grid locations
	for j in range(num_rows): 
		for i in range(num_cols):
			if field[j, i] == 0: # Wall
				plt.plot(i, num_rows - j - 1, 'ks')
			elif field[j, i] == 1: # Empty
				plt.plot(i, num_rows - j - 1, 'b.')
			elif field[j, i] == 2: # Fire
				plt.plot(i, num_rows - j - 1, 'rx')
			elif field[j, i] == 3: # Entrance
				plt.plot(i, num_rows - j - 1, 'go')
			else:
				plt.plot(i, num_rows - j - 1, 'yx')
	# Plot horizontal walls
	for j in range(num_rows): 
		for i in range(num_cols - 1):
			if not field[j, i] and not field[j, i+1]:
				line = np.array([[j, i], [j, i+1]])
				plt.plot(line[:, 1], num_rows - 1 - line[:, 0], 'k-', linewidth=10)				
	# Plot vertical walls
	for j in range(num_rows - 1): 
		for i in range(num_cols):
			if not field[j, i] and not field[j+1, i]:
				line = np.array([[j, i], [j+1, i]])
				plt.plot(line[:, 1], num_rows - 1 - line[:, 0], 'k-', linewidth=10)
	if bot:
		bot.plot()
	if path:
		data = [] 
		for n in path:
			data.append([n.row, n.col])
		data2 = np.array(data)
		if len(data) > 0:
			plt.plot(data2[:, 1], num_rows - 1 - data2[:, 0], 'y--')
	if fires:
		for fire in fires:
			if fire.size == 1:
				#print("Fire def", fire)
				rect = patches.Rectangle((fire.col-.25, num_rows-fire.row-1.25), width=.5, height=.5, linewidth=1, edgecolor='r', facecolor='none')
				ax.add_patch(rect)
			if fire.size == 2:
				#print("Fire def", fire)
				rect = patches.Rectangle((fire.col, num_rows-fire.row-1), width=1, height=1, linewidth=1, edgecolor='r', facecolor='none')
				ax.add_patch(rect)
			if fire.size == 3:
				#print("Fire def", fire)
				rect = patches.Rectangle((fire.col-1, num_rows-fire.row-2), width=2, height=2, linewidth=1, edgecolor='r', facecolor='none')
				ax.add_patch(rect)		
	plt.axis([-1, num_cols, -1, num_rows])
	plt.title("Maze")
	plt.show()

def generate_maze(num_rows, num_cols, num_fires_smol, num_fires_med, num_fires_lrg, num_inside, num_ent, plot_maze):
	# ---- Make sure maze is large enough for fires desired
	print("Generating Maze")
	if num_inside == 1 and (num_fires_med or num_fires_lrg):
		print("Maze insides too small")
		sys.exit()
	if num_inside == 2 and num_fires_lrg:
		print("Maze insides too small")
		sys.exit()
	# ---- Generate maze with all walls active
	maze = np.zeros((num_rows * 2 + 1, num_cols * 2 + 1))
	for i in range(num_rows):
		for j in range(num_cols):
			maze[i*2+1][j*2+1] = 1
	print(maze)
	# ---- Turn field into maze ----
	maze = random_kruskal_maze(maze)
	print("")
	print(maze)
	# ---- Increase maze size ----
	big_maze = maze_expansion(maze, num_inside)
	# ---- Generate the starting fires
	fires = generate_fires(big_maze, num_fires_smol, num_fires_med, num_fires_lrg)
	#print(walls)
	# ---- Generate the entrance to the maze
	entrances = generate_entrances(big_maze, num_ent)
	# ---- Get the list of all walls in the maze
	(hwalls, vwalls) = get_list_walls(big_maze)
	# ---- Plot maze ----
	if plot_maze:
		plot(field=big_maze, path=None, bot=None, fires=fires)
	return [big_maze, fires, entrances]
	
def main():
	print("Run Maze Generation Main\n")
	# ---- Run Maze Generation code
	num_rows = 4 # Number of rows in the maze
	num_cols = 4 # Number of columns in the maze
	num_fires_smol = 5 # Number of 1x1 in the maze
	num_fires_med = 3 # Number of 2x2 in the maze
	num_fires_lrg = 1 # Number of 3x3 in the maze
	num_inside = 3 # Number of padding inside each cell
	num_ent = 1 # Number of entrances to the maze
	plot_maze = True
	[maze, big_maze, fires, entrances] = generate_maze(num_rows, num_cols, num_fires_smol, num_fires_med, num_fires_lrg, num_inside, num_ent, plot_maze)

	# ---- Test code ----
		
if __name__ == "__main__":
	main()
