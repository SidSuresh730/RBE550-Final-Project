import random
import sys
import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
from matplotlib.path import Path 
from math import inf, pi 
from data_structure_library import Node, Fire, VWall, HWall

# Given an occupancy grid turn it into a perfect maze
# Inputs:
#	field, a 2D array of 0s for walls and 1 for open space. 
# Output:
#	field, the same 2D aray now as a completed maze
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
# Helper function for random_kruskal_maze
# Inputs:
#	row: Row of the desired node location
#	col: Col of the desired node location
#	nodes: List of nodes in the grid of the desired node location
# Outputs:
# 	nodes[i], the node object in the list of nodes with the given row and col, or 0 if no node exists
def find_node(row, col, nodes):
	# Iterate through the list searching for the correct row / col
	for i in range(len(nodes)):
		if nodes[i].row == row and nodes[i].col == col:
			return nodes[i]
	return 0 

# Return the nodes separated by a wall	
# Since the wall is effectively just a point in the grid we need to check if the nodes are above and below or left and right
# Helper function for random_kruskal_maze
# Inputs:
#	wall: An array cotaining the row and col of the wall in the field we are checking
#	nodes: List of nodes in the field
#	field: 2D array of 0s for walls and 1 for open space.  
# Outputs:	
#	An array of the two nodes in from the list of nodes separated by the wall
def get_separated(wall, nodes, field):
	row = wall[0]
	col = wall[1]
	if field[row][col - 1] and field[row][col + 1]: # If left and right values are unoccupied, those are the nodes and return them
		return [find_node(row, col - 1, nodes), find_node(row, col + 1, nodes)]
	elif field[row - 1][col] and field[row + 1][col]: # If above and below values are unoccupied, those are the nodes and return them
		return [find_node(row- 1, col, nodes), find_node(row + 1, col, nodes)]
	else:
		return [0] # Panic

# Add all nodes in node1's total connected list to node2.connected, and vice versa. Does not add in repeat nodes
# Helper function for random_kruskal_maze
# Input:
#	node1: First node to be connected
#	node2: Second node to be connected
# Output:
# 	No output, node objects class variables are updated
def connect_nodes(node1, node2):
	for node1_con in node1.connected:
		for node2_con in node2.connected:
			if node1_con not in node2_con.connected and node2_con not in node1_con.connected:
				node1_con.Connect(node2_con)
				node2_con.Connect(node1_con)


# Function that takes a maze, and returns an expanded version of the maze, where each cell int he maze is now a number wide equal to num_inside
# Note: The walls still take up a single width inside the maze
# Input:
#	maze: 2D occupancy grid representing the maze
#	num_inside: Desired number of nodes per cell of the expanded maze
# Output:
# 	expanded_maze: A new 2D occupancy grid representing the entire expanded maze
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

# Function that generates the number of each type of fire given, and returns those as a list of Fire object while updating the occupancy grid maze
# Input:
#	maze: 2D occupancy grid that represents a maze
#	num_fires_smol: Number of 1x1 fires to place in the maze 
#	num_fires_med: Number of 2x2 fires to place in the maze
#	num_fires_lrg: Number of 3x3 fires to place in the maze
# Output:
# 	maze: 2D occupancy grid that the maze, now with values of 2 to represent a fire
#	fires: List of the Fire objects made
def generate_fires(maze, num_fires_smol, num_fires_med, num_fires_lrg):
	num_rows = len(maze)
	num_cols = len(maze[0])
	med_size = 2
	lrg_size = 3
	fires = []
	# Generate 1x1 fires
	# Pick a random number, if that location in the maze is unnocupied change it to contain a fire, increment the counter and repeat until the coutner hits the num smol
	fires_made = 0
	while fires_made < num_fires_smol:
		fire_row = random.randint(1, num_rows-1)
		fire_col = random.randint(1, num_rows-1)
		if maze[fire_row][fire_col] == 1:
			maze[fire_row][fire_col] = 2
			fires_made += 1
			fires.append(Fire(fire_row, fire_col, 1))
	# Generate 2x2 fires
	# Same idea for 1x1 fires, but now we need to start in the bottom left coord of the fire, and check top left, top right, and bottom right as well to make sure those locations are unoccupied
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
	# Same idea for 2x2 fires, but checks all 9 total coords
	fires_made = 0
	while fires_made < num_fires_lrg:
		count = 0
		fire_row = random.randint(0, num_rows-3)
		fire_col = random.randint(0, num_rows-3)
		for i in range(lrg_size): 
			for j in range(lrg_size):
				if maze[fire_row - i][fire_col + j] == 1:
					count +=1
		if count >= lrg_size*lrg_size:
			for i in range(lrg_size): 
				for j in range(lrg_size):
					maze[fire_row - i][fire_col + j] = 2
			fires_made += 1
			fires.append(Fire(fire_row, fire_col, lrg_size))
	return maze, fires
	
# Function that places entrances in the maze, entrances are simply holes in the outer most maze walls
# Input:
#	maze: 2D occupancy grid representing the maze
#	num_ent: Number of entrances to add to the maze
# Output:
#	maze: 2D occupancy grid that the maze, now with values of 3 to represent entrances (These values have no difference than unoccupied tiles, except for plotting)
#	entrances: Array of [row, col] values representing the locations of entrances to the maze
def generate_entrances(maze, num_ent):
	num_rows = len(maze)
	num_cols = len(maze[0])
	entrances = []
	count = 0
	# We set up a count and an if statement check to make sure the entrance is not directly next to a wall when we generate it
	while count < num_ent:
		ent_loc = random.randint(1, 4)
		n = random.randint(1, num_rows-2)
		ent_loc = 4
		n = num_rows-3
		if ent_loc == 1: # Top side
			if maze[1][n]: 
				maze[0][n] = 3
				entrances.append([0, n, 3*pi/2])
				count +=1
		if ent_loc == 2: # Bott side
			if maze[num_rows-2][n]: 
				maze[num_rows-1][n] = 3
				entrances.append([num_rows-1, n, pi/2])
				count +=1
		if ent_loc == 3: # Left side
			if maze[n][1]: 
				maze[n][0] = 3
				entrances.append([n, 0, 0])
				count +=1
		if ent_loc == 4: # Right side
			if maze[n][num_cols-2]: 
				maze[n][num_cols-1] = 3
				entrances.append([n, num_cols-1, pi])
				count +=1
	return maze, entrances

# Function to generate a list of hwall and vwall objects containing the row or col of the wall (for horizontal or vertical) and it's upper and lower limits
# This function is also used by the bots as walls are used in collision detection
# Input:
#	maze: 2D occupancy grid that represents a perfect maze
# Output:
# 	hwalls: list of hwall objects that represent the horizontal walls in the maze
#	vwalls: list of vwall objects that represent the vertical walls in the maze
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
	return hwalls, vwalls

# Function to plot the maze using matplotlib. Can plot the maze, path of the robot, nodes in the robot's graph, and collision boxes of the fires
# Input:
#	field: 2D occupancy grid representing the maze
#	path: List of nodes representing a path to plot
#	bot: Bot object, used to plot the bot's self.tree values
#	fires: List of fire objects in the maze
# Output:
# 	Produces a matplotlib figure, no values are returned though
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
				plt.plot(i, num_rows - j - 1, 'gs')
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
				rect = patches.Rectangle((fire.col, num_rows-fire.row-1 ), width=2, height=2, linewidth=1, edgecolor='r', facecolor='none')
				ax.add_patch(rect)		
	plt.axis([-1, num_cols, -1, num_rows])
	plt.title("Maze")
	plt.show()

# Function to fully generate and return a maze and all supporting objects for initial generation
# Input:
#	num_rows: Int, number of rows in the maze
#	num_cols: Int, number of columns in the maze
#	num_inside: Int, desired number of nodes per cell of the expanded maze
#	num_fires_smol: Int, number of 1x1 fires to place in the maze
#	num_fires_med: Int, number of 2x2 fires to place in the maze
#	num_fires_lrg: Int, number of 3x3 fires to place in the maze
#	num_ent: Int, number of entrances to add to the maze
#	plot_maze: Boolean, True if we want to plot the maze False if not
# Output:
#	maze: 2D occupancy grid array representing the minimal size of the maze, no entrances or fires
#	big_maze: 2D occupancy grid array representing the full expanded maze
#	fires: List of the Fire objects made
#	entrances: Array of [row, col] values representing the locations of entrances to the maze 
def generate_maze(num_rows, num_cols, num_inside, num_fires_smol, num_fires_med, num_fires_lrg, num_ent, plot_maze):
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
	# ---- Turn field into maze ----
	maze = random_kruskal_maze(maze)
	# ---- Increase maze size ----
	big_maze = maze_expansion(maze, num_inside)
	# ---- Generate the starting fires
	big_maze, fires = generate_fires(big_maze, num_fires_smol, num_fires_med, num_fires_lrg)
	# ---- Generate the entrance to the maze
	big_maze, entrances = generate_entrances(big_maze, num_ent)
	# ---- Get the list of all walls in the maze
	hwalls, vwalls = get_list_walls(big_maze)
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
	[big_maze, fires, entrances] = generate_maze(num_rows, num_cols, num_inside, num_fires_smol, num_fires_med, num_fires_lrg, num_ent, plot_maze)

	# ---- Test code ----
		
if __name__ == "__main__":
	main()
