#code inspired from https://github.com/jbnerd/Path_Planning_Agent/blob/master/Uninformed_Search/Definitions_BFS.py
import copy

class State(object):
    """
    State (in state space) abstraction
    """
    def __init__(self, agent_position, dirt_positions):
        self.agent_position = agent_position
        self.dirt_positions = dirt_positions
    def __str__(self):
        message = "Agent position: "+ str(self.agent_position) + "\nDirt positions: " + str(self.dirt_positions) + "\n"
        return message
    def __eq__(self, other):
        return self.agent_position == other.agent_position and self.dirt_positions == other.dirt_positions
    def __hash__(self):
        return hash((tuple(self.agent_position), tuple([tuple(x) for x in self.dirt_positions])))
    
class Node(object):
    """
    Search tree node abtraction 
    """
    def __init__(self, state, parent, action, g, h=0, h_hat=0, d_hat =0, f=None, f_hat=None):
        """As specified in p. 78 of the course book (http://aima.cs.berkeley.edu/), requires:
            - state: state in state space.
            - parent: node in the search tree generating this node.
            - action: action applied to the parent node to generate the node.
            - g: from the initial node to the node.
        """
        self.state = state
        self.parent = parent
        self.action = action
        self.g = g
        self.h = h
        self.h_hat = h_hat
        self.d_hat = d_hat
        if f is None:
            self.f = self.g + self.h
        else:
            self.f = f   
        if f_hat is None:
            self.f_hat = self.g + h_hat
        else:
            self.f_hat = f   
        

    def __str__(self):
        return str(self.state)


    def compute_child_nodes(self, grid_size, obstacles_positions):
        """Implementation of the successor function

        Args:
            grid_size (int)
            obstacles_positions ( (n_obstacles,2) array-like)
        """
        child_nodes = []
        #we assume our vacumm cleaner only sucks when is located on a dirty location
        if self.state.dirt_positions and self.state.agent_position in self.state.dirt_positions:
            index = self.state.dirt_positions.index(self.state.agent_position)
            new_dirt_positions = copy.deepcopy(self.state.dirt_positions)
            new_dirt_positions = new_dirt_positions[:index]+new_dirt_positions[index+1:]
            position_copy = copy.deepcopy(self.state.agent_position)
            new_state = State(position_copy, new_dirt_positions)
            new_node = Node(new_state, self, "suck", self.g+1)
            child_nodes.append(new_node)
        else:

            #up
            new_position = self.state.agent_position.copy()
            new_position[0]-=1
            if new_position[0] >=0 and new_position not in obstacles_positions:
                new_state = State(new_position, self.state.dirt_positions)
                child_nodes.append(Node(new_state, self, "up", self.g+1))

            #down
            new_position = self.state.agent_position.copy()
            new_position[0]+=1
            if new_position[0] <grid_size and new_position not in obstacles_positions:
                new_state = State(new_position, self.state.dirt_positions)
                child_nodes.append(Node(new_state, self, "down", self.g+1))

            #left
            new_position = self.state.agent_position.copy()
            new_position[1]-=1
            if new_position[1] >=0 and new_position not in obstacles_positions:
                new_state = State(new_position, self.state.dirt_positions)
                child_nodes.append(Node(new_state, self, "left", self.g+1))

            #right
            new_position = self.state.agent_position.copy()
            new_position[1]+=1
            if new_position[1] < grid_size and new_position not in obstacles_positions:
                new_state = State(new_position, self.state.dirt_positions)
                child_nodes.append(Node(new_state, self, "right", self.g+1))
        return child_nodes

    def __eq__(self, other):
        return self.state == other.state

    def __hash__(self):
        return hash(self.state)

    def __lt__(self, other): 
        return (self.f, self.h, self.g) < (other.f, other.h, other.g)


class NodeAbs(object):
    def __init__(self, node):
        self.node: Node = node

    def __equal__(self, other):
        return self.node == other.node

    def __hash__(self):
        return hash(self.node)

class NodeF(NodeAbs):
    def __lt__(self, other): 
        return (self.node.f, self.node.h, self.node.g) < (other.node.f, other.node.h, other.node.g)

class NodeFHat(NodeAbs):
    def __lt__(self, other): 
        return (self.node.f_hat, self.node.h_hat, self.node.g) < (other.node.f_hat, other.node.h_hat, other.node.g)

class NodeDHat(NodeAbs):
    def __lt__(self, other): 
        return (self.node.d_hat, self.node.h_hat, self.node.g) < (other.node.d_hat, other.node.h_hat, other.node.g)


import heapq

class SearchTreePQS: #Non list-based implementation of the search tree
    def __init__(self, w):
        self._cleanup: list[NodeF] = []
        self._focal: list[NodeDHat] = []
        self._ready_to_focal: list[NodeFHat] = []
        self._open: list[NodeFHat] = []
        self.nodes: dict[str, Node] = dict()
        self.__closed: set[Node] = set()
        self._enc_open_dublicates = 0
        self.w = w
        
    def __len__(self):
        return len(self._cleanup) + len(self.__closed)
                    
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
    def add_to_cleanup(self, item):
        posisition = str(item.node)
        if posisition in self.nodes:
            if self.nodes[posisition].node.g < item.node.g:
                return
        heapq.heappush(self._cleanup, item)
        self.nodes[posisition] = item

    # def add_to_focal(self, item):
    #     posisition = str(item.node)
    #     if posisition in self.nodes:
    #         if self.nodes[posisition].g < item.g:
    #             return
    #     heapq.heappush(self._focal, item)
    #     self.nodes[posisition] = item

    def add_to_open(self, item):
        posisition = str(item.node)
        if posisition in self.nodes:
            if self.nodes[posisition].node.g < item.node.g:
                return
        heapq.heappush(self._open, item)
        heapq.heappush(self._ready_to_focal, item)
        self.nodes[posisition] = item

    def update_focal(self, bestf_hat):
        if(len(self._ready_to_focal) == 0):
            return
        open_node = self._ready_to_focal[0]
        while open_node.node.f_hat < self.w * bestf_hat:
            if not self.was_expanded(open_node.node):
                heapq.heappush(self._focal, NodeDHat(open_node.node))
            heapq.heappop(self._ready_to_focal)
            if(len(self._ready_to_focal) == 0):
                break
            open_node = self._ready_to_focal[0]
    
    
    '''
    Extracting the best node (i.e. the one with the minimal key 
    = min f-value = min g-value (for Dijkstra)) from OPEN.
    This node will be expanded further on in the main loop of the search.
    ''' 
    def get_best_node(self):
        while True:
            # print(self.nodes)
            best_f_hat = self._open[0]
            self.update_focal(best_f_hat.node.f_hat)
            best_f = self._cleanup[0]
            best = None
            if self._focal and self._focal[0].node.f_hat <= self.w * best_f.node.f:
                best = self._focal[0]
                # print("It's in focal")
                heapq.heappop(self._focal)
            elif best_f_hat.node.f_hat <= self.w * best_f.node.f:
                best = best_f_hat
                # print("It's in open")
                heapq.heappop(self._open)
            else:
                best = best_f
                # print("It's in cleanup")
                heapq.heappop(self._cleanup)

            posisition = str(best.node)

            if posisition in self.nodes:
                del self.nodes[posisition]
                # print(self.nodes)
                return best

    def add_to_closed(self, item):
        self.__closed.add(item)

    def was_expanded(self, item):
        return item in self.__closed

    @property
    def OPEN(self):
        return self._cleanup
    
    @property
    def CLOSED(self):
        return list(self.__closed)

    @property
    def number_of_open_dublicates(self):
        return self._enc_open_dublicates


class SearchTreePQSWA: #Non list-based implementation of the search tree
    def __init__(self):
        self._heap: list[Node] = []
        self.nodes = dict()
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
        posisition = str(item)
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
            posisition = str(best)
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