from data_structure_library import Node
import random
import math
import matplotlib as mpl
import matplotlib.pyplot as plt


def distance(node1, node2):
    return math.sqrt((node1.index[0]-node2.index[0])**2 + (node1.index[1]-node2.index[1])**2)

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
    def rrt_move(self):
        #random sample a node
        q_rand = Node((random(self.ncol), random(self.nrow)))
        #find closest node in tree to random node
        q_curr = self.find_closest_node(q_rand)
        direction = direction(q_curr, q_rand)
        #attempt to grow tree in direction of q_rand
        q_new = Node(index=(int(self.epsilon*math.cos(direction)),int(self.epsilon*math.sin(direction))))
        possible_edge = (q_curr, q_new)
        if not self.will_collide(possible_edge):
            self.V.append(q_new)
            self.E.append(possible_edge)

    # method for finding closest node in tree to randomly generated node
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
    def will_collide(self,edge) -> bool:
        # TO DO
        pass
    

