import random
import numpy as np
import matplotlib.pyplot as plt # Plotting our obstacle grid
from matplotlib.path import Path # Plotting our search algorithm path
from math import inf # Used for infinity

empty_list = np.array([])


class Node:
	def __init__(self, row, col, connected):
		self.row = row
		self.col = col
		self.connected = connected
		
	def Connect(self, node):
		self.connected = np.append(self.connected, node)
		
	def __str__(self):
		print(self.row, self.col)
		for node in self.connected:
			print(self.row, self.col, "-", node.row, node.col)
		return "Node printed"

# Given an occupancy grid turn it into a non looping maze
def random_kruskal_maze(field):
	num_rows = len(field)
	num_cols = len(field[0])
	nodes = []
	walls = []
	
	#Create a list of the nodes
	for i in range(num_rows):
		for j in range(num_cols):
			if field[i][j] == 0:
				node = Node(i, j, [])
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
		check_nodes = get_separated(check_wall, field)
		nodes_connected = check_connected(check_nodes[0], check_nodes[1], nodes)
		if not nodes_connected:
			field[check_wall[0]][check_wall[1]] = 0
			Node1 = find_node(check_nodes[0][0], check_nodes[0][1], nodes) 
			Node2 = find_node(check_nodes[1][0], check_nodes[1][1], nodes) 
			Node1.Connect(Node2)
			Node2.Connect(Node1)
	return field
	

# Given location of a node and a list of Nodes, return the full node from the list
def find_node(row, col, nodes):
	for i in range(len(nodes)):
		if nodes[i].row == row and nodes[i].col == col:
			return nodes[i]
	return 0 
		

# Return the row and columns of nodes separated by a wall				
def get_separated(wall, field):
	row = wall[0]
	col = wall[1]
	if field[row][col - 1] == 0 and field[row][col + 1] == 0:
		return np.array([[row, col - 1], [row, col + 1]])
	elif field[row - 1][col] == 0 and field[row + 1][col] == 0:
		return np.array([[row - 1, col], [row + 1, col]])
	else:
		return [0]

# Check if two nodes are connected based on their list of connected nodes
def check_connected(node1, node2, nodes):
	Node1 = find_node(node1[0], node1[1], nodes) 
	Node2 = find_node(node2[0], node2[1], nodes) 
	node1_list = Node1.connected
	nodes_checked = []
	while len(node1_list) > 0:
		iterate_node = node1_list[0]
		node1_list = np.delete(node1_list, 0)
		nodes_checked = np.append(nodes_checked, iterate_node)
		if iterate_node.row == Node2.row and iterate_node.col == Node2.col:
			return 1
		else:
			for next_node in iterate_node.connected:
				if next_node not in nodes_checked:
					node1_list = np.append(node1_list, next_node)
	return 0


# Connect two nodes and all of their connected nodes
def connect_list(node1, node2, nodes):
	node1_list = node1.connected
	node2_list = node2.connected
	node1.Connect(node2)
	node2.Connect(node1)
	for node in node1_list:
		node2.Connect(node)
	for node in node2_list:
 		node1.Connect(node)
 		
 # Check if two nodes are connected based on their connected nodes list but better
def check_connect_list(node1, node2, nodes):
	Node1 = find_node(node1[0], node1[1], nodes) 
	Node2 = find_node(node2[0], node2[1], nodes) 
	node1_list = Node1.connected
	for node in node1_list:
		if node.row == Node2.row and node.col == Node2.col:
			return 1
	return 0

def generate_fires(field, fires, num_fires): 
	num_rows = len(field)
	num_cols = len(field[0])
	fires_made = 0
	while fires_made < num_fires:
		fire_row = random.randint(1, num_rows-1)
		fire_col = random.randint(1, num_rows-1)
		if field[fire_row][fire_col] == 0 and not fires[fire_row][fire_col]:
			fires[fire_row][fire_col] = 1
			fires_made += 1
	return fires

		
def plot(field, fires):
	print("Plotting")
	num_rows = len(field)
	num_cols = len(field[0])
	# Plot all occupancy grid locations
	for j in range(num_rows): 
		for i in range(num_cols):
			if field[j, i] == 1:
				plt.plot(i, num_rows - j - 1, 'kx')
			elif fires[j, i] == 1:
				plt.plot(i, num_rows - j - 1, 'rx')
			else:
				plt.plot(i, num_rows - j - 1, 'bx')
	# Plot horizontal walls
	for j in range(num_rows): 
		for i in range(num_cols - 1):
			if field[j, i] and field[j, i+1]:
				line = np.array([[j, i], [j, i+1]])
				plt.plot(line[:, 1], num_rows - 1 - line[:, 0], 'k-')				
	# Plot vertical walls
	for j in range(num_rows - 1): 
		for i in range(num_cols):
			if field[j, i] and field[j+1, i]:
				line = np.array([[j, i], [j+1, i]])
				plt.plot(line[:, 1], num_rows - 1 - line[:, 0], 'k-')
	plt.axis([-1, num_cols, -1, num_rows])
	plt.title("Maze")
	plt.show()


def main():
	print("Run Maze Generation Main\n")
	# ---- Generate field with walls ----
	num_rows = 5 # Number of rows in the maze
	num_cols = 5 # Number of columns in the maze
	num_fires = 2 # Number of fires in the maze
	field = np.ones((num_rows * 2 + 1, num_cols * 2 + 1))
	fires = np.zeros((num_rows * 2 + 1, num_cols * 2 + 1))
	for i in range(num_rows):
		for j in range(num_cols):
			field[i*2+1][j*2+1] = 0
	
	# ---- Generate the starting fires
	fires = generate_fires(field, fires, num_fires)
	
	# ---- Turn field into maze ----
	random_kruskal_maze(field)
	plot(field, fires)


		# ---- Test code ----
	#node1 = Node(0, 0, [])
	#node1_loc = [0, 0]
	#node2 = Node(0, 1, [])
	#node2_loc = [0, 1]
	#node3 = Node(1, 0, [])
	#node3_loc = [1, 0]
	#node4 = Node(2, 2, [])
	#node4_loc = [2, 2]
	#node5 = Node(0, 2, [])
	#node5_loc = [0, 2]
	#nodes = [node1, node2, node3, node4, node5]
	#connect_list(node1, node3, nodes)
	#print(node1)
	#print("")
	#print(node3)
	#print("\n\n")
	#connect_list(node2, node1, nodes)
	#print(node1)
	#print("")
	#print(node2)
	#print("")
	#print(node3)
	#print("")
	#connect_list(node5, node2, nodes)
	#a = check_connected(node3_loc, node5_loc, nodes)
	#print(node1)
	#print("")
	#print(node2)
	#print("")
	#print(node3)
	#print("")
	#print(node4)
	#print("")
	#print(node5)
	#print("")
	#print(a)
	
	#node1.Connect(node2)
	#node1.Connect(node3)
	#node2.Connect(node1)
	#node3.Connect(node1)
	#node2.Connect(node5)
	
	#a = check_connected(node1_loc, node2_loc, nodes)
	#b = check_connected(node2_loc, node1_loc, nodes)
	#c = check_connected(node2_loc, node3_loc, nodes)
	#d = check_connected(node3_loc, node2_loc, nodes)
	#e = check_connected(node1_loc, node3_loc, nodes)
	#f = check_connected(node3_loc, node1_loc, nodes)
	
	#g = check_connected(node1_loc, node4_loc, nodes)
	#h = check_connected(node2_loc, node4_loc, nodes)
	#i = check_connected(node3_loc, node4_loc, nodes)
	#j = check_connected(node4_loc, node1_loc, nodes)
	#k = check_connected(node4_loc, node2_loc, nodes)
	#l = check_connected(node4_loc, node3_loc, nodes)
	#print(a, b, c, d, e, f)
	#print(g, h, i, j, k, l)	
		
if __name__ == "__main__":
	main()
