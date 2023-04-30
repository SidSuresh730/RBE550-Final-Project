import numpy as np
import math
MAX_NODES = 16384
# States of grid/obstacles
EMPTY = 0
OB_NORM = 1
OB_BURN = 2
OB_EXT = 3
OB_BURNED = 4
OBSTACLE_LIFE = 500
#Colors
GREEN = (0,255,0)
RED = (255,0,0)
DARK_GREEN = (0,153,0)
BLACK = (0,0,0)
TAN = (210,180,140)
WHITE = (255,255,255)
FIRE_ENGINE_RED = (206,32,41) 

# function for finding Euclidean distance between nodes
# Input: Nodes
# Output: float (Distance)
def distance(node1, node2):
	return math.sqrt((node1.col-node2.col)**2 + (node1.row-node2.row)**2)

# function for finding the direction from node 1 to node 2
# Input: Node 1 and Node 2
# Output: float (angle of direction)
def direction(node1, node2):
	return math.atan2(node2.row-node1.row, node2.col-node1.col)

def pn_distance(point, node, num_rows):
	return math.sqrt((point[0]-node.col)**2 + (point[1]-(num_rows-node.row-1))**2)

def pn_direction(point, node, num_rows):
	#print("Pt", point)
	#print("Nd", node)
	#print("Nd conv", node.col, num_rows-node.row-1)
	return math.atan2((num_rows-node.row-1)-point[1], node.col-point[0])
	
class Node:
		def __init__(self, row, col):
			# Row and col location of the Node
			self.row = row
			self.col = col
			
			# Heuristic variables
			self.g = math.inf
			self.h = math.inf
			self.f = math.inf
			
			# list of the neighbors of a particular node
			self.neighbors = None
			# List of all Nodes the Node is connected to. Each node is connected to itself
			self.connected = [self]
			# parent of the node in a graph
			self.parent = None

			# rrt conditions            
		
		def Connect(self, node):
			self.connected = np.append(self.connected, node)
		
		def __eq__(self, other):
			return (self.row, self.col) == (other.row, other.col)
			# return distance(self,other)<=0.0001
		
		def __str__(self):
			print(self.row, self.col, "-")#, end = ' ')
			return ""

class Graph:
	def __init__(self, start) -> None:
		self.V = []
		self.V.append(start)
		self.E = []
	
	def remove_edge(self, e):
		if e[0] and e[1]:
			for i in range(len(self.E)):
				if self.E[i][0] == e[0] and self.E[i][1] == e[1]:
					self.E.pop(i)
					break
	
	def remove_vertex(self, v):
		if v:
			for i in range(len(self.V)):
				if self.V[i] == v:
					self.V.pop(i)
					break
	
	def print_edge(self, e):
		print(e[0], e[1])

class VWall:
	def __init__(self, col, llim, ulim) -> None:
		self.col = col
		self.llim = llim
		self.ulim = ulim
		
	def __str__(self):
		print(self.col, self.llim, self.ulim)
		return ""

class HWall:
	def __init__(self, row, llim, ulim) -> None:
		self.row = row
		self.llim = llim
		self.ulim = ulim
	 
	def __str__(self):
		print(self.row, self.llim, self.ulim)
		return "" 
	 
# Priority Queue class for use in Djikstra implementation
class PriorityQueue:
	def __init__(self) -> None:
		# list representing the queue
		self.q = list()

		# attribute used for debugging purposes
		self.recursion_calls = 0
	
	# method for adding element to P Q
	def add(self, element):
		self.q.append(element)
		# self.mergesort(dict, self.q)
		# print("Added: ",end="")
		# print(element.index)
		
	def remove(self, element):
		self.q.remove(element)
	
	# method for getting min distance element
	# queue is first sorted by decreasing distance and 
	# the last element is accessed and removed
	def get_min_dist_element(self):
		self.mergesort(self.q)
		if(len(self.q)>0):
			return self.q.pop()
	
	def add_list(self, list):
		for element in list:
			self.q.append(element)

	# Method for sorting queue
	# Time: O(nlogn)
	# Space: O(n)
	def mergesort(self, lst):
		self.recursion_calls+=1
		# if self.recursion_calls<100:
		#     print('Recursion calls: %d' % (self.recursion_calls))
		#     print(self.q)
		if len(lst)>1:
			div = len(lst)//2
			left = lst[:div]
			right = lst[div:]
			self.mergesort(left)
			self.mergesort(right)
			i, j, k = (0,0,0) #self.row, self.col) == (other.row, other.col)
			while i<len(right) and j<len(left):
				if(right[i].f>=left[j].f):
					lst[k] = right[i]
					i+=1
				else:
					lst[k] = left[j]
					j+=1
				k+=1
			while i<len(right):
				lst[k]=right[i]
				i+=1
				k+=1
			while j<len(left):
				lst[k] = left[j]
				j+=1
				k+=1
				
	def __str__(self):
		print("PQ")
		for n in self.q:
			print(n)
		return ""

class Fire:
	def __init__(self, row, col, size):
		# Row and col location of the Node
		self.row = row
		self.col = col
		self.size = size
		
		self.active = True
		self.found = False
	def extinguish(self):
		self.active = False
		
	def __str__(self):
		print(self.row, self.col, self.size, "-", end = ' ')
		return ""

def generate_list_nodes(maze, end):
	num_rows = len(maze)
	num_cols = len(maze[0])
	nodes = []
	for i in range(num_rows):
		for j in range(num_cols):
			if maze[i][j]:
				node = Node(i, j)
				if(end):
					node.h = math.dist([i, j], end)
				else:
					node.h = 0
				nodes.append(node)
	return nodes


def get_neighbors(node, field, nodes, graph):
	neighbors = []
	row_mod = [-1, -1, -1, 0, 0, 1, 1, 1]
	col_mod = [-1, 0, 1, -1, 1, -1, 0, 1]
	num_neighbors = min(len(row_mod), len(col_mod))
	for i in range(num_neighbors):
		row = node.row + row_mod[i]
		col = node.col + col_mod[i]
		if row >= 0 and col >= 0 and row < len(field) and col < len(field[0]) and field[row][col]:
			Neighbor = find_node(row, col, nodes)
			alt = node.g + distance(node, Neighbor)
			if alt < Neighbor.g:
				graph.remove_edge((Neighbor.parent, Neighbor))
				Neighbor.g = alt
				Neighbor.f = Neighbor.g + Neighbor.h
				Neighbor.parent = node
				graph.E.append((Neighbor.parent, Neighbor))
				neighbors.append(Neighbor)
	return neighbors
	
	
def find_node(row, col, nodes):
	for node in nodes:
		if node.row == row and node.col == col:	
			return node
	return 0
