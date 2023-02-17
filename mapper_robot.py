from  data_structure_library import Node, PriorityQueue
from math import sqrt

class MapperBot:
    def __init__(self) -> None:
        pass

    def djikstra_map(self,s, value):
        dist = {}
        prev = {}
        queue = PriorityQueue()
        for v in self.node_list:
            # Initialize the rest of the distances to Inf.
            dist[v] = float('Inf')
            # Initialize the prev{} dictionary
            prev[v] = None
            # Add elements to P Q
            queue.add(v)
        dist[s.index] = 0
        # While queue not empty
        while len(queue.q)>0:
            self.dji_iterations+=1
            u = queue.get_min_dist_element(dist) #get minimum dist element and remove from queue
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
            for v in self.node_list[u].neighbors: # update neighbor distances as necessary
                if v in queue.q:
                    alt = dist[u] + sqrt((v[0]-u[0])**2 + (v[1]-u[1])**2)
                    if alt<dist[v]:
                        dist[v] = alt
                        prev[v] = u
                        