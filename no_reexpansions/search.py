from definitions import *
from map import *

def astar(grid_map, start_i, start_j, goal_i, goal_j, w, heuristic_func = None, search_tree = None, prioritet=None):
    ast = search_tree()
    steps = 0
    nodes_created = 0
    CLOSED = None

    h0 = heuristic_func(start_i, start_j, goal_i, goal_j)
    ast.add_to_open(Node(start_i, start_j, h=h0, f=prioritet(h0, 0, w)))
    nodes_created += 1
    
    while not ast.open_is_empty():
        steps += 1
        v = ast.get_best_node_from_open()
        ast.add_to_closed(v)
        if v.i == goal_i and v.j == goal_j:
            return True, v, steps, nodes_created, ast.OPEN, ast.CLOSED
        
        for nxt_i, nxt_j in grid_map.get_neighbors(v.i, v.j):
            nxt = Node(nxt_i, nxt_j, parent=v)
            if not ast.was_expanded(nxt):
                h = heuristic_func(nxt_i, nxt_j, goal_i, goal_j)
                nxt.g = v.g + compute_cost(v.i, v.j, nxt_i, nxt_j)
                nxt.h = h
                nxt.f = prioritet(nxt.h, nxt.g, w)
                nxt.parent = v
                ast.add_to_open(nxt)
                nodes_created += 1
        
    
    return False, None, steps, nodes_created, None, CLOSED


def wastar(grid_map, start_i, start_j, goal_i, goal_j, w, heuristic_func = None, search_tree = None, prioritet=None):
    ast = search_tree()
    steps = 0
    nodes_created = 0
    CLOSED = None

    h0 = heuristic_func(start_i, start_j, goal_i, goal_j)
    ast.add_to_open(Node(start_i, start_j, h=h0, f=prioritet(h0, 0, w)))
    nodes_created += 1
    
    while not ast.open_is_empty():
        steps += 1
        v = ast.get_best_node_from_open()
        ast.add_to_closed(v)
        if v.i == goal_i and v.j == goal_j:
            return True, v, steps, nodes_created, ast.OPEN, ast.CLOSED
        
        for nxt_i, nxt_j in grid_map.get_neighbors(v.i, v.j):
            nxt = Node(nxt_i, nxt_j, parent=v)
            if not ast.was_expanded(nxt):
                h = heuristic_func(nxt_i, nxt_j, goal_i, goal_j)
                nxt.g = v.g + compute_cost(v.i, v.j, nxt_i, nxt_j)
                nxt.h = h
                nxt.f = prioritet(nxt.h, nxt.g, w)
                nxt.parent = v
                ast.add_to_open(nxt)
                nodes_created += 1
        
    
    return False, None, steps, nodes_created, None, CLOSED