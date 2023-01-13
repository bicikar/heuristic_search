#inspired from https://github.com/jbnerd/Path_Planning_Agent/blob/master/Uninformed_Search/BFS.py
from collections import deque
import copy
from EES.definitions import *
import bisect
from EES.heuristcs import *

def write_solution(node):
    """Returns the sequence of actions and the cost once a solution has been found.

    Args:
        node (Node): Solution node.
    """
    action_list = []
    cost = node.g
    while (node.parent is not None):
        action_list.append(node.action)
        node=node.parent
    action_list.reverse()

    return action_list, cost

def bfs(initial_state, grid_size, obstacles_positions):
    """Performs breadth-first search on the vacuum cleaner world.

    Args:
        initial_state (State)
        grid_size (int)
        obstacles_positions (n_obstaclesx2 array-like)
    """
    solution_checks = 0
    solutions = []
    costs = []
    solutions_checks = []
    node = Node(initial_state, None, None, g = 0)
    if not initial_state.dirt_positions:
        print("Problem solved!")
        return [], 0
    else:
        frontier = deque()
        explored = set()
        frontier_check = set()

        frontier.append(node)
        frontier_check.add(node.state)

        while(True):
            if not frontier:
                return solutions, costs, solutions_checks
            else:
                node = frontier.popleft()
                frontier_check.discard(node.state)
                explored.add(node.state)
                child_nodes = node.compute_child_nodes(grid_size, obstacles_positions)

                for child_node in child_nodes:
                    if child_node.state not in explored and child_node.state not in frontier_check:
                        if not child_node.state.dirt_positions:
                            print("Found a solution!")
                            solution, cost = write_solution(child_node)
                            print(solution, cost)
                            print(f"Solution checks performed: {solution_checks}.")
                            solutions.append(solution)
                            costs.append(cost)
                            solutions_checks.append(solution_checks)

                        else:
                            frontier.append(child_node)
                            frontier_check.add(child_node.state)
                            solution_checks += 1

def a_star(initial_state, grid_size, obstacles_positions, heuristic):
    """Performs A* search on the vacuum cleaner world.

    Args:
        initial_state (State)
        grid_size (int)
        obstacles_positions (n_obstaclesx2 array-like)
    """
    solution_checks = 0
    solutions = []
    costs = []
    solutions_checks = []
    node = Node(initial_state, None, None, g = 0)
    if not initial_state.dirt_positions:
        print("Problem solved!")
        return [], 0
    else:
        frontier = deque()
        frontier_f_values = deque()
        explored = set()
        frontier_check = set()

        frontier.append(node)
        f = heuristic(node)
        frontier_f_values.append(f)
        frontier_check.add(node.state)

        while(True):
            if not frontier:
                return solutions, costs, solutions_checks
            else:
                node = frontier.popleft()
                frontier_f_values.popleft()
                frontier_check.discard(node.state)
                explored.add(node.state)
                child_nodes = node.compute_child_nodes(grid_size, obstacles_positions)

                for child_node in child_nodes:
                    if child_node.state not in explored and child_node.state not in frontier_check:
                        if not child_node.state.dirt_positions:
                            print("Found a solution!")
                            solution, cost = write_solution(child_node)
                            print(solution, cost)
                            print(f"Solution checks performed: {solution_checks}.")
                            solutions.append(solution)
                            costs.append(cost)
                            solutions_checks.append(solution_checks)
                        else:
                            f = child_node.g + heuristic(child_node)
                            index = bisect.bisect_left(frontier_f_values,f)
                            frontier.insert(index,child_node)
                            frontier_f_values.insert(index,f) 
                            frontier_check.add(child_node.state)
                            solution_checks += 1



def EES(initial_state, grid_size, obstacles_positions, w, search_tree=SearchTreePQS, dist=diagonal_distance, dist_step=diagonal_distance_steps):
    ast = search_tree(w)
    steps = 0
    nodes_created = 0

    h0 = mst_heuristic(initial_state, dist)
    h0_hat = mst_heuristic(initial_state, dist_step)
    d0_hat = d_heuristic(initial_state, dist_step)
    
    cur = Node(initial_state, None, None, g=0, h=h0, h_hat=h0_hat, d_hat=d0_hat)
    # cur.f = cur.g + w * cur.h
    # cur.f_hat = cur.g + w * cur.h_hat

    ast.add_to_open(NodeFHat(cur))
    ast.add_to_cleanup(NodeF(cur))
    nodes_created += 1
    
    while not ast.open_is_empty():
        # print(nodes_created)
        steps += 1
        v = ast.get_best_node()
        ast.add_to_closed(v.node)

        
        if not v.node.state.dirt_positions:
            return True, v.node, v.node.g, steps, nodes_created
        
        child_nodes = v.node.compute_child_nodes(grid_size, obstacles_positions)

        for child_node in child_nodes:
            if not child_node.state.dirt_positions:
                return True, v.node, v.node.g, steps, nodes_created
            
            if not ast.was_expanded(child_node):
                
                child_node.h = mst_heuristic(child_node.state, dist)
                
                child_node.h_hat = mst_heuristic(child_node.state, dist_step)
                
                child_node.d_hat = d_heuristic(child_node.state, dist_step)
                
                child_node.f = child_node.g + child_node.h
                child_node.f_hat = child_node.g + child_node.h_hat
                ast.add_to_open(NodeFHat(child_node))
                ast.add_to_cleanup(NodeF(child_node))
                nodes_created += 1
        
    
    return False, None, None, steps, nodes_created

def wastar(initial_state, grid_size, obstacles_positions, w, prioritet=phi, search_tree=SearchTreePQSWA, dist=diagonal_distance):
    ast = search_tree()
    steps = 0
    nodes_created = 0

    h0 = mst_heuristic(initial_state, dist)
    
    cur = Node(initial_state, None, None, g=0, h=h0)
    cur.f = prioritet(cur.h, cur.g, w)

    ast.add_to_open(cur)
    nodes_created += 1
    
    while not ast.open_is_empty():
        steps += 1
        v = ast.get_best_node_from_open()
        ast.add_to_closed(v)

        
        if not v.state.dirt_positions:
            return True, v, v.g, steps, nodes_created
        
        child_nodes = v.compute_child_nodes(grid_size, obstacles_positions)

        for child_node in child_nodes:
            if not child_node.state.dirt_positions:
                return True, v, v.g, steps, nodes_created
            
            if not ast.was_expanded(child_node):
                
                child_node.h = mst_heuristic(child_node.state, dist)
                child_node.f = prioritet(child_node.h, child_node.g, w)
                # child_node.f_hat = child_node.g + child_node.h_hat
                ast.add_to_open(child_node)
                nodes_created += 1
        
    
    return False, None, None, steps, nodes_created