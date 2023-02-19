from data_structure_library import Node, VWall, HWall
import random
import math
import matplotlib as mpl
import matplotlib.pyplot as plt

# function for finding Euclidean distance between nodes
# Input: Nodes
# Output: float (Distance)
def distance(node1, node2):
    return math.sqrt((node1.index[0]-node2.index[0])**2 + (node1.index[1]-node2.index[1])**2)

# function for finding the direction from node 1 to node 2
# Input: Node 1 and Node 2
# Output: float (angle of direction)
def direction(node1, node2):
    return math.atan2(node2.index[1]-node1.index[1], node2.index[0]-node1.index[0])

class Graph:
    def __init__(self, start) -> None:
        self.V = {start}
        self.E = {}

class RRTBot:
    def __init__(self, epsilon, start, nrow, ncol) -> None:
        self.epsilon = epsilon
        self.current_pos = start
        self.tree = Graph(start)
        self.nrow = nrow
        self.ncol = ncol
    
    # one iteration of the RRT algorithm
    # Input: None
    # Output: None
    def rrt_move(self):
        #random sample a node
        q_rand = Node((random(self.ncol), random(self.nrow)))
        while q_rand in self.tree.V:
            q_rand = Node((random(self.ncol), random(self.nrow)))
        #find closest node in tree to random node
        q_curr = self.find_closest_node(q_rand)
        direction = direction(q_curr, q_rand)
        #attempt to grow tree in direction of q_rand
        q_new = Node(index=(int(self.epsilon*math.cos(direction)),int(self.epsilon*math.sin(direction))))
        possible_edge = (q_curr, q_new)
        hwalls = 1 #TO DO
        vwalls = 1 #TO DO 
        if not self.will_collide(possible_edge):
            self.V.append(q_new)
            self.E.append(possible_edge)

    # method for finding closest node in tree to randomly generated node
    # Input: Randomly generated node
    # Output: Closest node in tree to the random node
    def find_closest_node(self, q_rand):
        min_dist = float('Inf')
        min_node = None
        for node in self.tree.V:
            new_dist = distance(node, q_rand)
            if new_dist < min_dist:
                min_dist = new_dist
                min_node = node
        return min_node

    # method to determine if a given edge will collide with an obstacle
    # Input: Edge, horizontal wall's list, vertical walls list
    # Output: Boolean (True: Will collide, False: Will NOT collide)
    def will_collide(self,edge, hwalls, vwalls) -> bool:
        #find slope of edge
        m = (edge[1].nrow-edge[0].nrow)/(edge[1].ncol-edge[0].ncol)
        #check collision with horizontal walls
        for wall in hwalls:
            if wall.row in range(min(edge[0].nrow, edge[1].nrow), max(edge[0].nrow, edge[1].nrow)+1):
                y = wall.row
                y1 = edge[0].nrow
                x1 = edge[0].ncol
                if (y-y1)/m+x1 in range(wall.llim, wall.ulim+1):
                    return True
        #check collision with vertical walls
        for wall in vwalls:
            if wall.col in range(min(edge[0].ncol, edge[1].ncol), max(edge[0].ncol, edge[1].ncol)+1):
                x = wall.coll
                y1 = edge[0].nrow
                x1 = edge[0].ncol
                if m*(x-x1)+y1 in range(wall.llim, wall.ulim+1):
                    return True
        # TO DO
        return False
    

