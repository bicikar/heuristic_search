class Node:
    '''
    Node class represents a search node

    - i, j: coordinates of corresponding grid element
    - g: g-value of the node
    - h: h-value of the node // always 0 for Dijkstra
    - F: f-value of the node // always equal to g-value for Dijkstra
    - parent: pointer to the parent-node 

    '''
    

    def __init__(self, i, j, g = 0, h = 0, f = None, parent = None, tie_breaking_func = None):
        self.i = i
        self.j = j
        self.g = g
        self.h = h
        if f is None:
            self.f = self.g + h
        else:
            self.f = f        
        self.parent = parent

        
    
    def __eq__(self, other):
        '''
        Estimating where the two search nodes are the same,
        which is needed to detect dublicates in the search tree.
        '''
        return (self.i == other.i) and (self.j == other.j)
    
    def __hash__(self):
        '''
        To implement CLOSED as set of nodes we need Node to be hashable.
        '''
        ij = self.i, self.j
        return hash(ij)


    def __lt__(self, other): 
        '''
        Comparing the keys (i.e. the f-values) of two nodes,
        which is needed to sort/extract the best element from OPEN.
        
        This comparator is very basic. We will code a more plausible comparator further on.
        '''
        return (self.f, self.h, self.g) < (other.f, other.h, self.g)

import heapq

class SearchTreePQS: #Non list-based implementation of the search tree
    def __init__(self):
        self._heap: list[Node] = []
        self.nodes: dict[(int, int), Node] = dict()
        self.__closed = set()
        self._enc_open_dublicates = 0
        
    def __len__(self):
        return len(self._heap) + len(self.__closed)
                    
    '''
    open_is_empty should inform whether the OPEN is exhausted or not.
    In the former case the search main loop should be interrupted.
    '''
    def open_is_empty(self):
        return len(self.nodes) == 0
    
    '''
    Adding a (previously not expanded) node to the search-tree (i.e. to OPEN).
    It's either a totally new node (the one we never encountered before)
    or it can be a dublicate of the node that currently resides in OPEN.
    It's up to us how to handle dublicates in OPEN. We can either 
    detect dublicates upon adding (i.e. inside this method) or detect them
    lazily, when we are extracting a node for further expansion.
    Not detecting dublicates at all may work in certain setups but let's not
    consider this option.
    '''    
    def add_to_open(self, item):
        posisition = (item.i, item.j)
        if posisition in self.nodes:
            if self.nodes[posisition].g < item.g:
                return
        heapq.heappush(self._heap, item)
        self.nodes[posisition] = item
    
    
    '''
    Extracting the best node (i.e. the one with the minimal key 
    = min f-value = min g-value (for Dijkstra)) from OPEN.
    This node will be expanded further on in the main loop of the search.
    ''' 
    def get_best_node_from_open(self):
        while True:
            best = self._heap[0]
            posisition = (best.i, best.j)
            heapq.heappop(self._heap)
            if posisition in self.nodes:
                del self.nodes[posisition]
                return best

    def add_to_closed(self, item):
        self.__closed.add(item)

    def was_expanded(self, item):
        return item in self.__closed

    @property
    def OPEN(self):
        return self._heap
    
    @property
    def CLOSED(self):
        return list(self.__closed)

    @property
    def number_of_open_dublicates(self):
        return self._enc_open_dublicates

def make_path(goal):
    '''
    Creates a path by tracing parent pointers from the goal node to the start node
    It also returns path's length.
    '''

    length = goal.g
    current = goal
    path = []
    while current.parent:
        path.append(current)
        current = current.parent
    path.append(current)
    return path[::-1], length

def read_task_from_file(path):
    '''
    Reads map, start/goal positions and true value of path length between given start and goal from file by path. 
    '''

    tasks_file = open(path)
    height = int(tasks_file.readline())
    width = int(tasks_file.readline())
    cells = [[0 for _ in range(width)] for _ in range(height)]
    i = 0
    j = 0

    for l in tasks_file:
        j = 0
        for c in l:
            if c == '.':
                cells[i][j] = 0
            elif c == '#':
                cells[i][j] = 1
            else:
                continue
            j += 1
            
        if j != width:
            raise Exception("Size Error. Map width = ", j, ", but must be", width, "(map line: ", i, ")")
                
        i += 1
        if(i == height):
            break
    
    start_i = int(tasks_file.readline())
    start_j = int(tasks_file.readline())
    goal_i = int(tasks_file.readline())
    goal_j = int(tasks_file.readline())
    length = float(tasks_file.readline())
    return (width, height, cells, start_i, start_j, goal_i, goal_j, length)
