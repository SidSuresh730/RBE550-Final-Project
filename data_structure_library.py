import numpy as np
import math
MAX_NODES = 16384

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

class Node:
        def __init__(self, row, col):
            # Row and col location of the Node
            self.row = row
            self.col = col
            
            # Heuristic variables
            self.g = float('Inf')
            self.h = float('Inf')
            self.f = float('Inf')
            
            # list of the neighbors of a particular node
            self.neighbors = None
            # List of all Nodes the Node is connected to. Each node is connected to itself
            self.connected = [self]
            # parent of the node in a graph
            self.parent = None            
		
        def Connect(self, node):
            self.connected = np.append(self.connected, node)
		
		def __eq__(self, other):
        	return (self.row, self.col) == (other.row, other.col)
		
        def __str__(self):
            print(self.row, self.col, "-", end = ' ')
            for node in self.connected:
                print(node.row, node.col, ',', end = ' ')
            return ""


class Graph:
    def __init__(self, start) -> None:
        self.V = []
        self.V.append(start)
        self.E = []

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
            self.mergesort(dict,left)
            self.mergesort(dict,right)
            i, j, k = (0,0,0)
            while i<len(right) and j<len(left):
                if(right[i].g>=left[j].g):
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
