from data_structure_library import Node, VWall, HWall
import random
import math
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import maze_generation

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

class Graph:
    def __init__(self, start) -> None:
        self.V = []
        self.V.append(start)
        self.E = []

class RRTBot:
    def __init__(self, epsilon, start, nrow, ncol) -> None:
        self.epsilon = epsilon
        self.current_pos = start
        self.tree = Graph(start)
        self.nrow = nrow
        self.ncol = ncol
        # self.collisions = 0
        self.success = 0
    
    # one iteration of the RRT algorithm
    # Input: None
    # Output: None
    def rrt_move(self, hwalls, vwalls):
        #random sample a node
        q_rand = Node(random.uniform(1.0,self.ncol), random.uniform(1.0,self.nrow))
        # q_rand = Node(row=random.randint(self.current_pos.row-5, self.current_pos.row+5), col=random.randint(self.current_pos.row-5, self.current_pos.row+5))
        while q_rand in self.tree.V:
            print("Node already in tree! Try again")
            # q_rand = Node(row=random.randint(self.current_pos.row-5, self.current_pos.row+5), col=random.randint(self.current_pos.row-5, self.current_pos.row+5))
            q_rand = Node(random.uniform(1.0,self.ncol), random.uniform(1.0,self.nrow))

        #find closest node in tree to random node
        q_curr = self.find_closest_node(q_rand)
        dir = direction(q_curr, q_rand)
        dis = distance(q_curr, q_rand)
        #attempt to grow tree in direction of q_rand
        q_new = Node(row=round(q_curr.row + (self.epsilon%dis)*math.sin(dir),2),col=round(q_curr.col +(self.epsilon%dis)*math.cos(dir),2))
        possible_edge = (q_curr, q_new)
        if not self.will_collide(possible_edge, hwalls, vwalls) and not self.lies_on_edge(q_new):
            q_new.parent = q_curr
            self.tree.V.append(q_new)
            self.current_pos=q_new
            self.tree.E.append(possible_edge)
            self.success+=1
            # print(possible_edge[0])
            # print(possible_edge[1])
            # self.plot()

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
        # check if vertical edge
        if(edge[1].col == edge[0].col):
            for wall in vwalls:
                if edge[0].col == wall.col:
                    if edge[1].row in np.arange(wall.llim, wall.ulim+1, 0.01):
                        return True
            return False
        #find slope of edge
        m = (edge[1].row-edge[0].row)/(edge[1].col-edge[0].col)
        #check collision with horizontal walls
        for wall in hwalls:
            l = min(edge[0].row, edge[1].row)
            u = max(edge[0].row, edge[1].row)
            # if wall.row in np.arange(min(edge[0].row, edge[1].row), max(edge[0].row, edge[1].row)+1, 0.01):
            if wall.row >= l and wall.row <= u:
                print("Possible collision!")
                # lower_node = min(edge[0].col, edge[1].col)
                # upper_node = max(edge[0].col, edge[1].col)
                # case1 = lower_node>wall.llim and upper_node<wall.ulim
                # case2 = lower_node<wall.ulim and upper_node<wall.ulim
                # case3 = lower_node>wall.llim and upper_node>wall.ulim
                # if case1:
                #     print("Collision!")
                #     return True
                y = wall.row
                y1 = edge[0].row
                x1 = edge[0].col
                point = round((y-y1)/m+x1,2)
                # print(point)
                # if round((y-y1)/m+x1,2) in np.arange(wall.llim, wall.ulim+1, 0.01):
                if point >= wall.llim and point <= wall.ulim:
                    print("H Collision!")
                    # self.collisions += 1
                    return True
        #check collision with vertical walls
        for wall in vwalls:
            l = min(edge[0].col, edge[1].col)
            u = max(edge[0].col, edge[1].col)
            # if wall.col in np.arange(min(edge[0].col, edge[1].col), max(edge[0].col, edge[1].col)+1, 0.01):
            if wall.col >= l and wall.col <= u:
                # lower_node = min(edge[0].row, edge[1].row)
                # upper_node = max(edge[0].row, edge[1].row)
                # case1 = lower_node>wall.llim and upper_node<wall.ulim
                # if case1:
                #     print("Collision!")
                #     return True
                x = wall.col
                y1 = edge[0].row
                x1 = edge[0].col
                point = round(m*(x-x1)+y1,2)
                # print(point)
                # if round(m*(x-x1)+y1,2) in np.arange(wall.llim, wall.ulim+1, 0.01):
                if point >= wall.llim and point <= wall.ulim:
                    print("V Collision!")
                    return True
        return False

    # method to determine if a node lies on a current edge of the tree
    # Input: node to add to tree
    # Output: Boolean (True: there is an edge that the node would coincide with, False: No edge)
    def lies_on_edge(self, node):
        for edge in self.tree.E:
            # check if vertical edge
            if(edge[1].col == edge[0].col):
                if(node.col==edge[0].col):
                    if node.row in np.arange(min(edge[0].row, edge[1].row), max(edge[0].row, edge[1].row)+1, 0.01):
                            print("On edge vertical!")
                            return True
            elif node.col in np.arange(min(edge[0].col, edge[1].col), max(edge[0].col, edge[1].col)+1, 0.01):
                # find slope of edge
                m = (edge[1].row-edge[0].row)/(edge[1].col-edge[0].col)
                if node.row - edge[0].row == m*(node.col-edge[0].col):
                    print("On edge!")
                    return True
        return False
    
    
    def plot(self):
        # plot vertices
        for node in self.tree.V:
            plt.plot(node.col, self.nrow-node.row, 'rx')
        for edge in self.tree.E:
            x_arr = [edge[0].col, edge[1].col]
            y_arr = [self.conv(edge[0].row), self.conv(edge[1].row)]
            plt.plot(x_arr, y_arr, 'bo', linestyle="--")
        # plt.show()
    
    # method for converting row to numpy array reference frame
    def conv(self, row):
        return self.nrow - row
def main():
    num_rows = 6 # Number of rows in the maze
    num_cols = 6 # Number of columns in the maze
    num_fires_smol = 0 # Number of 1x1 in the maze
    num_fires_med = 0 # Number of 2x2 in the maze
    num_fires_lrg = 0 # Number of 3x3 in the maze
    num_inside = 5
    num_ent = 0
    if num_inside == 1 and (num_fires_med or num_fires_lrg):
        print("Maze insides too small")
        sys.exit()
    if num_inside == 2 and num_fires_lrg:
        print("Maze insides too small")
        sys.exit()
    field = np.zeros((num_rows * 2 + 1, num_cols * 2 + 1))
    for i in range(num_rows):
	    for j in range(num_cols):
	    	field[i*2+1][j*2+1] = 1
    maze = maze_generation.random_kruskal_maze(field)
	# ---- Increase maze size ----
    big_maze = maze_generation.maze_expansion(maze, num_inside)
    (hwalls, vwalls) = maze_generation.get_list_walls(big_maze)
    print('HWalls: ')
    for wall in hwalls:
        print(wall)
    print('VWalls: ')
    for wall in vwalls:
        print(wall)
	# ---- Generate the starting fires
    maze_generation.generate_fires(big_maze, num_fires_smol, num_fires_med, num_fires_lrg)
	# ---- Generate the entrance to the maze
    maze_generation.generate_entrances(big_maze, num_ent)
	# ---- Plot maze ----
    # maze_generation.plot(big_maze)
    bot = RRTBot(epsilon= 1, start=Node(row=1, col=1), nrow=36, ncol=36)
    while bot.success<500:
        # print(i)
        bot.rrt_move(hwalls=hwalls,vwalls=vwalls)
    # print("Collisions: %d" % bot.collisions)
    bot.plot()
    maze_generation.plot(big_maze)

if __name__ == "__main__":
	main()