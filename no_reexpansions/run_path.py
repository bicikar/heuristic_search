from complexity import *
import sys
import os
import time

Berlin_map = read_map_from_file("Data/Berlin_0_256.map")
Berlin_scen = read_tasks_from_file("Data/Berlin_0_256.map.scen")

# draw(Berlin_map, None, None)

# nodes_norm, steps_norm, tasks_hard, nodes, steps, task_lens, correct_tasks = count_complexity(Berlin_map, Berlin_scen, 7)

if __name__ == '__main__':
    argc = len(sys.argv)

    if argc < 6:
        raise ValueError("Map name and positions should be set")

    map = read_map_from_file("Data/" + sys.argv[1])

    x_start, y_start = int(sys.argv[2]), int(sys.argv[3])
    x_end, y_end = int(sys.argv[4]), int(sys.argv[5])

    if argc <= 6:
        w = 1
    else:
        w = int(sys.argv[6])

    if not map.in_bounds(x_start, y_start):
        raise ValueError("Start point is not in bounds")

    if not map.in_bounds(x_end, y_end):
        raise ValueError("End point is not in bounds")

    if not map.traversable(x_start, y_start):
        raise ValueError("Start point is an obstacle")

    if not map.traversable(x_end, y_end):
        raise ValueError("End point is an obstacle")

    cur_time = time.time()
    path_found, res_node, steps, nodes_created, open, closed = astar(map, x_start, y_start, x_end, y_end, 1, diagonal_distance, SearchTreePQS, phi)
    res_time = time.time() - cur_time

    if not path_found:
        print("Path not found!")
        exit()

    path, length = make_path(res_node)

    print("A* summary:")
    print("Working time: {:.5f} ".format(res_time), "; Nodes created:", nodes_created, "; Path length: {:.5f}".format(length))
    
    draw(map, Node(x_start, y_start), Node(x_end, y_end), 'a_star', path, open, closed)

    cur_time = time.time()
    path_found, res_node, steps, nodes_created, open, closed = wastar(map, x_start, y_start, x_end, y_end, w, diagonal_distance, SearchTreePQS, phi)
    res_time = time.time() - cur_time

    path, length = make_path(res_node)
    print("\nWA* summary, weight {}:".format(w))
    print("Working time: {:.5f} ".format(res_time), "; Nodes created:", nodes_created, "; Path length: {:.5f}".format(length))
    
    draw(map, Node(x_start, y_start), Node(x_end, y_end), 'wa_star', path, open, closed)

    cur_time = time.time()
    path_found, res_node, steps, nodes_created, open, closed = wastar(map, x_start, y_start, x_end, y_end, w, diagonal_distance, SearchTreePQS, phi_xdp)
    res_time = time.time() - cur_time

    path, length = make_path(res_node)
    print("\nPhi XDP summary, weight {}:".format(w))
    print("Working time: {:.5f} ".format(res_time), "; Nodes created:", nodes_created, "; Path length: {:.5f}".format(length))
    
    draw(map, Node(x_start, y_start), Node(x_end, y_end), 'phi_xdp', path, open, closed)

    cur_time = time.time()
    path_found, res_node, steps, nodes_created, open, closed = wastar(map, x_start, y_start, x_end, y_end, w, diagonal_distance, SearchTreePQS, phi_xup)
    res_time = time.time() - cur_time

    path, length = make_path(res_node)
    print("\nPhi XUP summary, weight {}:".format(w))
    print("Working time: {:.5f} ".format(res_time), "; Nodes created:", nodes_created, "; Path length: {:.5f}".format(length))
    
    draw(map, Node(x_start, y_start), Node(x_end, y_end), 'phi_xup', path, open, closed)

    
