
import math

def phi(x, y, w):
    return y + w * x

def phi_xdp(x, y, w):
    return (y + (2 * w - 1) * x + math.sqrt((y-x) ** 2 + 4 * w * y * x)) / (2 * w)

def phi_xup(x, y, w):
    return (y + x + math.sqrt((y + x) ** 2 + 4 * w * (w-1) * x * x)) / (2 * w)

def manhattan_distance(node):
    """Computes Manhattan distance between the vacuum cleaner and the first dirt position in the list.

    Args:
        node (Node)
    """
    return abs(node.state.agent_position[0]-node.state.dirt_positions[0][0]) + abs(node.state.agent_position[1]-node.state.dirt_positions[0][1]) 

def manh_distance(vert1, vert2):
    """Computes Manhattan distance between the vacuum cleaner and the first dirt position in the list.

    Args:
        node (Node)
    """
    return abs(vert1[0]-vert2[0]) + abs(vert1[1]-vert1[1]) 

def diagonal_distance(vert1, vert2):
    x = abs(vert1[0]-vert2[0])
    y = abs(vert1[1]-vert2[1])
    return min(x, y) * math.sqrt(2) + abs(x - y)

def diagonal_distance_steps(vert1, vert2):
    x = abs(vert1[0]-vert2[0])
    y = abs(vert1[1]-vert2[1])
    return min(x, y) + abs(x - y)

class Graph:
    def __init__(self, num_of_nodes):
        self.m_num_of_nodes = num_of_nodes
        # Initialize the adjacency matrix with zeros
        self.m_graph = [[0 for column in range(num_of_nodes)] 
                    for row in range(num_of_nodes)]

    def add_edge(self, node1, node2, weight):
        self.m_graph[node1][node2] = weight
        self.m_graph[node2][node1] = weight

    def prims_mst(self):
        # Defining a really big number, that'll always be the highest weight in comparisons
        postitive_inf = float('inf')

        # This is a list showing which nodes are already selected 
        # so we don't pick the same node twice and we can actually know when stop looking
        selected_nodes = [False for node in range(self.m_num_of_nodes)]

        # Matrix of the resulting MST
        result = [[0 for column in range(self.m_num_of_nodes)] 
                    for row in range(self.m_num_of_nodes)]
        
        indx = 0
        # for i in range(self.m_num_of_nodes):
            # print(self.m_graph[i])
        
        # print(selected_nodes)

        # While there are nodes that are not included in the MST, keep looking:
        while(False in selected_nodes):
            # We use the big number we created before as the possible minimum weight
            minimum = postitive_inf

            # The starting node
            start = 0

            # The ending node
            end = 0

            for i in range(self.m_num_of_nodes):
                # If the node is part of the MST, look its relationships
                if selected_nodes[i]:
                    for j in range(self.m_num_of_nodes):
                        # If the analyzed node have a path to the ending node AND its not included in the MST (to avoid cycles)
                        if (not selected_nodes[j] and self.m_graph[i][j]>0):  
                            # If the weight path analized is less than the minimum of the MST
                            if self.m_graph[i][j] < minimum:
                                # Defines the new minimum weight, the starting vertex and the ending vertex
                                minimum = self.m_graph[i][j]
                                start, end = i, j
            
            # Since we added the ending vertex to the MST, it's already selected:
            selected_nodes[end] = True

            # Filling the MST Adjacency Matrix fields:
            result[start][end] = minimum
            
            if minimum == postitive_inf:
                result[start][end] = 0

            # print("(%d.) %d - %d: %d" % (indx, start, end, result[start][end]))
            indx += 1
            
            result[end][start] = result[start][end]

        # Print the resulting MST
        # for node1, node2, weight in result:
        sum = 0
        for i in range(len(result)):
            for j in range(0+i, len(result)):
                if result[i][j] != 0:
                    sum += result[i][j]
        return sum

def mst_heuristic(node, dist):
    if len(node.dirt_positions) == 1 and node.dirt_positions[0] == node.agent_position:
        return 0
    graph = Graph(len(node.dirt_positions) + 1)
    for i, dirt in enumerate(node.dirt_positions):
        graph.add_edge(0, i + 1, dist(dirt, node.agent_position))

    for i, dirt1 in enumerate(node.dirt_positions):
        for j, dirt2 in enumerate(node.dirt_positions):
            graph.add_edge(i + 1, j + 1, dist(dirt1, dirt2))

    res = graph.prims_mst()
    return res

def nearest(agent_pos, dirts, dist):
    min_dist = float('inf')
    min_ind = 0

    for i, dirt in enumerate(dirts):
        cur_dist = dist(agent_pos, dirt)
        if cur_dist < min_dist:
            min_dist = cur_dist
            min_ind = i
    return min_ind, min_dist

def d_heuristic(state, dist):
    agent_pos = state.agent_position.copy()
    dirts = state.dirt_positions.copy()
    sum = 0
    
    while dirts:
        nearest_index, min_dist = nearest(agent_pos, dirts, dist)
        sum += min_dist
        dirts.pop(nearest_index)

    return sum
