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
RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)
DARK_GREEN = (0,153,0)
BLACK = (0,0,0)
TAN = (210,180,140)
WHITE = (255,255,255)
FIRE_ENGINE_RED = (206,32,41) 
CYAN = (0,255,255)

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
	
	# Add one node to another's connected list. Full connection is handled outside
	def Connect(self, node):
		self.connected = np.append(self.connected, node)
	
	# Need to compare nodes a lot so we dont create new ones, only need their locations to be identical 
	def __eq__(self, other):
		return (self.row, self.col) == (other.row, other.col)
		# return distance(self,other)<=0.0001
	
	def __str__(self):
		print(self.row, self.col, "-")#, end = ' ')
		return ""

# ---- Helper functions for Node objects:
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
	
# Input for finding a node in a list of nodes.
# Since we dont want to make multiple nodes at the same location, we check for nodes by checking the row and col values
# Inputs:
#	row: Row of the node we are looking for
#	col: Col of the node we are looking for
#	nodes: List of nodes to search through
# Output:
#	node: The node in the list that matches the row and col given. 
# Note: This is only intended to be run where you know the node you are looking for exists, but functionality can be set up if it doesnt so we still return 0
def find_node(row, col, nodes):
	for node in nodes:
		if node.row == row and node.col == col:	
			return node
	return 0
# ---- End helper node functions


# Graph of nodes including vertices and edges
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

# Vertical wall inside the maze. Defined by its column, and lower and upper limits (rows it extends to)
class VWall:
	def __init__(self, col, llim, ulim) -> None:
		self.col = col
		self.llim = llim
		self.ulim = ulim
		
	def __str__(self):
		print(self.col, self.llim, self.ulim)
		return ""

# Horizontal wall inside the maze. Defined by its row, and lower and upper limits (columns it extends to)
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

# Fire object put inside the maze. Defined by its location and size
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
