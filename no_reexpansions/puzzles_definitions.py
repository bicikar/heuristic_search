import numpy as np
import heapq
import math
import os

class GemPuzzleState:

    # Constructor. Sets tile positions + some basic checks.
    def __init__(self, tileList):
        self.tileList = tileList
        self.size = int(len(tileList) ** 0.5)
        blankValue = self.size ** 2
        if (blankValue != len(tileList)):
            raise Exception("The tile list must contain the number of elements which is equal to the square of an integer!")

        # Memorizing the position of a blank tile
        # Technically, there is no need to do so, but it makes to get the successors a bit faster
        self.blankPos = -1 
        for i in enumerate(tileList):
            if (i[1] == blankValue):
                self.blankPos = i[0]

        if (self.blankPos == -1):
            raise Exception("State should contains max value as position to blank tile")

    def __eq__(self, other):
        for a, b in zip(self.tileList, other.tileList):
            if a != b:
                return False
        return True

    # Printing the state as tile matrix
    def __str__(self):
        res = []
        charTileList = list(map(str, self.tileList))
        charTileList[self.blankPos] = '_'
        for rowStart in range(0, len(charTileList), self.size):
            res.append(charTileList[rowStart:rowStart+self.size])
        return '\n'.join([''.join(['{:4}'.format(item) for item in row]) for row in res]) + "\n"

class Node:
    def __init__(self, state, g = 0, h = 0, F = None, parent = None):
        self.state = state
        self.state_str = str(state)
        self.g = g
        self.h = h
        if F is None:
            self.F = self.g + h
        else:
            self.F = F
        self.parent = parent

    def __eq__(self, other):
        return self.state == other.state

    def __lt__(self, other):
        return (self.F, self.h, self.g) < (other.F, other.h, self.g)
    
    def __hash__(self):
        return hash(self.state_str)

import copy

def GetSuccessors(state):
    delta = [[0, 1], [1, 0], [0, -1], [-1, 0]]
    successors = []
    for d in delta:
        row = state.blankPos // state.size #identifying the row in which blank is located
        col = state.blankPos % state.size #identifying the column in which blank is located
        row += d[0] #computing new row for blank (corresponding to a particular move encoded via 'd')
        col += d[1] #computing new column for blank (corresponding to a particular move encoded via 'd')
        
        #if the new position of a blank is valid (i.e. it is still within the field) then
        #a corresponding sucessor should be added to the succesors' list
        if (0 <= row < state.size) and (0 <= col < state.size):
            newState = copy.deepcopy(state)
            newBlankPos = row * state.size + col
            newState.tileList[newState.blankPos] = newState.tileList[newBlankPos] #moving tile
            newState.tileList[newBlankPos] = newState.size ** 2 #setting blank
            newState.blankPos = newBlankPos

            successors.append(newState)

    return successors

class SearchTreePQS: #Non list-based implementation of the search tree
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
        posisition = item.state_str
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
            posisition = best.state_str
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

def ManhattanDistance(puzzleState1, puzzleState2):
    pos = dict()
    for i in range(puzzleState1.size):
        for j in range(puzzleState1.size):
            cur = puzzleState1.tileList[i * puzzleState1.size + j]
            pos[cur] = (i, j)
    res = 0
    for i in range(puzzleState2.size):
        for j in range(puzzleState2.size):
            cur = puzzleState2.tileList[i * puzzleState2.size + j]
            if cur == puzzleState2.size ** 2:
                continue
            res += (abs(pos[cur][0] - i) + abs(pos[cur][1] - j))
    return res


def AStar(startState, goalState, w, search_tree = None, prioritet = None, heuristicFunction = ManhattanDistance):
    ast = search_tree()
    steps = 0
    nodes_created = 0
    CLOSED = None

    h0 = heuristicFunction(startState, goalState)
    cur = Node(startState, h=h0, F=prioritet(h0, 0, w))
#     print("Start:", cur.state_str)
    ast.add_to_open(cur)
    nodes_created += 1
    
    while not ast.open_is_empty():
        steps += 1
        v = ast.get_best_node_from_open()
#         if steps % 1 == 0:
#             print("Current best:", v.state_str)
        ast.add_to_closed(v)
        if v.state == goalState:
            return True, v, steps, nodes_created
        
        for nxt_state in GetSuccessors(v.state):
            nxt = Node(nxt_state, v.g + 1, parent=v)
            if not ast.was_expanded(nxt):
                h = heuristicFunction(nxt_state, goalState)
                nxt.h = h
                nxt.F = prioritet(nxt.h, nxt.g, w)
                ast.add_to_open(nxt)
                nodes_created += 1
        
    
    return False, None, steps, nodes_created

def IsSolvable(tileList):
    inversions = 0
    empty_pos = 0
    puzzleExeptEmpty = []
    for i, v in enumerate(tileList):
        if v != len(tileList):
            puzzleExeptEmpty.append((i, v))
        else:
            empty_pos = i

    for i, tile in puzzleExeptEmpty:
        j = i + 1
        while j < len(tileList):
            if tileList[j] < tile:
                inversions += 1
            j += 1
    
    size = int(math.sqrt(len(tileList)))
    
    if size % 2 != 0 and inversions % 2 == 0:
        return True

    if size % 2 == 0:
        emptyrow = size - empty_pos // size
        return (emptyrow % 2 != 0) == (inversions % 2 == 0)

    return False

def GenerateTasks(taskFile, number, size, seed=None):
    if seed is not None:
        np.random.seed(seed)
    with open(taskFile, 'w') as f:
        for _ in range(number):
            while True:
                tiles = np.random.permutation(size ** 2) + 1
                if IsSolvable(tiles):
                    f.write(' '.join(tiles.astype(str)))
                    f.write('\n')
                    break


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

def save_puzzle(path, name):
    dir = os.path.dirname(os.path.abspath(__file__))

    with open(dir + "/../result_puzzle/" + name, 'w') as f:
        for puzzle in path:
            print(puzzle.state, file=f)