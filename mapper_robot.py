from data_structure_library import Node, PriorityQueue, distance
import math
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import maze_generation
class MapperBot:
    def __init__(self, node_list) -> None:
        self.node_list = node_list

    def djikstra_map(self,s):
        # dist = {}
        # prev = {}
        queue = PriorityQueue()
        # for v in self.node_list:
        #     # Initialize the rest of the distances to Inf.
        #     v.g = float('Inf')
        #     # Initialize the prev{} dictionary
        #     v.parent = None
        #     # Add elements to P Q
        #     queue.add(v)
        s.g = 0
        # While queue not empty
        while len(queue.q)>0:
            # self.dji_iterations+=1 
            u = queue.get_min_dist_element() #get minimum dist element and remove from queue
            # if self.node_list[u] == goal:
            #     x=u
            #     lst = []
            #     while x!= s.index:
            #         lst.append(x)
            #         self.grid.grid[x]= value
            #         # Error check
            #         if not prev[x]:
            #             print('No path found!')
            #             break
            #         x = prev[x]
            #     break
            for v in u.neighbors: # update neighbor distances as necessary
                if v in queue.q:
                    alt = u.g + distance(v, u)
                    if alt<v.g:
                        v.g = alt
                        v.parent = u
    
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
                        