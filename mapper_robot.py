

class Node:
        def __init__(self, index, neighbors):
            # tuple representing the index, can be used to index into grid
            self.index = index
            # list of the neighbors of a particular node
            self.neighbors = neighbors

            #booleans for indicating if an algorithm has visited the node
            self.bfs_visited = False
            self.dfs_visited = False
            self.dji_visited = False
            self.rand_visited = False

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
    
    # method for getting min distance element
    # queue is first sorted by decreasing distance and 
    # the last element is accessed and removed
    def get_min_dist_element(self, dict):
        self.mergesort(dict, self.q)
        if(len(self.q)>0):
            return self.q.pop()

    # Method for sorting queue
    # Time: O(nlogn)
    # Space: O(n)
    def mergesort(self, dict, lst):
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
                if(dict[right[i]]>=dict[left[j]]):
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
            if self.node_list[u] == goal:
                x=u
                lst = []
                while x!= s.index:
                    lst.append(x)
                    self.grid.grid[x]= value
                    # Error check
                    if not prev[x]:
                        print('No path found!')
                        break
                    x = prev[x]
                break
            for v in self.node_list[u].neighbors: # update neighbor distances as necessary
                if v in queue.q:
                    alt = dist[u] + sqrt((v[0]-u[0])**2 + (v[1]-u[1])**2)
                    if alt<dist[v]:
                        dist[v] = alt
                        prev[v] = u
                        