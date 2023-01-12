from puzzles_definitions import *
from map import *
from heuristics import *
import sys
import os
import time

# draw(Berlin_map, None, None)

# nodes_norm, steps_norm, tasks_hard, nodes, steps, task_lens, correct_tasks = count_complexity(Berlin_map, Berlin_scen, 7)

if __name__ == '__main__':
    argc = len(sys.argv)

    if argc < 3:
        raise ValueError("File name should be presented")

    tasks = []

    w = int(sys.argv[2])
    dir = os.path.dirname(os.path.abspath(__file__))
    if argc > 3 and sys.argv[3] == '1':
        GenerateTasks(dir + '/Data/' + sys.argv[1], 1, 4)

    tasks = []
    
    with open(dir + '/Data/' + sys.argv[1]) as f:
        for line in f:
            tiles = list(map(int, line.split()))
            if not IsSolvable(tiles):
                raise ValueError("Puzzle is not solvable")
            startState = GemPuzzleState(tiles)
            goalState = GemPuzzleState(range(1, len(tiles) + 1))
            tasks.append((startState, goalState))


    startState, goalState = tasks[0][0], tasks[0][1]
    cur_time = time.time()
    path_found, res_state, steps, nodes_created = AStar(startState, goalState, w, SearchTreePQS, phi)
    res_time = time.time() - cur_time

    path, length = make_path(res_state)

    print("WA* summary:")
    print("Working time: {:.5f} ".format(res_time), "; Nodes created:", nodes_created, "; Path length: {:.5f}".format(length))

    save_puzzle(path, "wa_star.txt")

    cur_time = time.time()
    path_found, res_state, steps, nodes_created = AStar(startState, goalState, w, SearchTreePQS, phi_xdp)
    res_time = time.time() - cur_time

    path, length = make_path(res_state)

    print("\nPhi XDP summary:")
    print("Working time: {:.5f} ".format(res_time), "; Nodes created:", nodes_created, "; Path length: {:.5f}".format(length))

    save_puzzle(path, "phi_xdp.txt")

    cur_time = time.time()
    path_found, res_state, steps, nodes_created = AStar(startState, goalState, w, SearchTreePQS, phi_xup)
    res_time = time.time() - cur_time

    path, length = make_path(res_state)

    print("\nPhi XUP summary:")
    print("Working time: {:.5f} ".format(res_time), "; Nodes created:", nodes_created, "; Path length: {:.5f}".format(length))

    save_puzzle(path, "phi_xup.txt")
    
    # draw(map, Node(x_start, y_start), Node(x_end, y_end), 'a_star', path, open, closed)

    # cur_time = time.time()
    # path_found, res_node, steps, nodes_created, open, closed = wastar(map, x_start, y_start, x_end, y_end, w, diagonal_distance, SearchTreePQS, phi)
    # res_time = time.time() - cur_time

    # path, length = make_path(res_node)
    # print("\nWA* summary, weight {}:".format(w))
    # print("Working time: {:.5f} ".format(res_time), "; Nodes created:", nodes_created, "; Path length: {:.5f}".format(length))
    
    # draw(map, Node(x_start, y_start), Node(x_end, y_end), 'wa_star', path, open, closed)

    # cur_time = time.time()
    # path_found, res_node, steps, nodes_created, open, closed = wastar(map, x_start, y_start, x_end, y_end, w, diagonal_distance, SearchTreePQS, phi_xdp)
    # res_time = time.time() - cur_time

    # path, length = make_path(res_node)
    # print("\nPhi XDP summary, weight {}:".format(w))
    # print("Working time: {:.5f} ".format(res_time), "; Nodes created:", nodes_created, "; Path length: {:.5f}".format(length))
    
    # draw(map, Node(x_start, y_start), Node(x_end, y_end), 'phi_xdp', path, open, closed)

    # cur_time = time.time()
    # path_found, res_node, steps, nodes_created, open, closed = wastar(map, x_start, y_start, x_end, y_end, w, diagonal_distance, SearchTreePQS, phi_xup)
    # res_time = time.time() - cur_time

    # path, length = make_path(res_node)
    # print("\nPhi XUP summary, weight {}:".format(w))
    # print("Working time: {:.5f} ".format(res_time), "; Nodes created:", nodes_created, "; Path length: {:.5f}".format(length))
    
    # draw(map, Node(x_start, y_start), Node(x_end, y_end), 'phi_xup', path, open, closed)

    
