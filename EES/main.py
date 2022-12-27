from search import a_star, bfs, EES
from definitions import State
from heuristcs import manhattan_distance
import time

grid_size = 10
obstacles_positions = [[0,1],[0,2],[1,1],[1,2],[4,0],[4,2],[6,2],[2,4],[3,4],[4,4],[6,4],[2,5],[5,8],[3,6],[8,0],[9,0],[8,1],[9,1],[7,5],[7,6],[8,6]]
dirt_positions = [[8,2],[6,7]]

initial_position = [0,0]

initial_state = State(initial_position, dirt_positions)

start_time = time.time()
bfs(initial_state, grid_size, obstacles_positions) #, manhattan_distance
print("Execution time %s seconds." % (time.time() - start_time))

#compare methods
import random
import numpy as np
import os, sys

class HiddenPrints:
    def __enter__(self):
        self._original_stdout = sys.stdout
        sys.stdout = open(os.devnull, 'w')

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout.close()
        sys.stdout = self._original_stdout

grid_size = 10
n_obstacles = 20
initial_position = [0,0]

bfs_times=[]
bfs_checks=[]
a_star_times=[]
a_star_checks=[]

repetitions = 100

with HiddenPrints():
    for i_rep in range(repetitions):
        obstacles_positions = [[random.randint(1,10),random.randint(1,10)] for _ in range(n_obstacles) ]
        dirt_positions = [[random.randint(0,10),random.randint(0,10)]]
        while dirt_positions in obstacles_positions:
            dirt_positions = [[random.randint(0,10),random.randint(0,10)]]

        start_time = time.time()
        solutions, costs, checks = bfs(initial_state, grid_size, obstacles_positions)
        bfs_times.append(time.time() - start_time)
        if checks:
            bfs_checks.append(np.min(checks))

        start_time = time.time()
        solutions, costs, checks = a_star(initial_state, grid_size, obstacles_positions, manhattan_distance)
        a_star_times.append(time.time() - start_time)
        if checks:
            a_star_checks.append(np.min(checks))

        print(EES(initial_state, grid_size, obstacles_positions, w=3))

print(f"Solution checks for bfs: {np.average(bfs_checks)} +- {(np.average(np.array(bfs_checks)**2)-np.average(bfs_checks)**2)/np.sqrt(repetitions-1)} seconds.")
print(f"Solution checks for A*: {np.average(a_star_checks)}  +- {(np.average(np.array(a_star_checks)**2)-np.average(a_star_checks)**2)/np.sqrt(repetitions-1)} seconds.")

print(f"Execution time for bfs: {np.average(bfs_times)} +- {(np.average(np.array(bfs_times)**2)-np.average(bfs_times)**2)/np.sqrt(repetitions-1)} seconds.")
print(f"Execution time for A*: {np.average(a_star_times)}  +- {(np.average(np.array(a_star_times)**2)-np.average(a_star_times)**2)/np.sqrt(repetitions-1)} seconds.")

print(EES(initial_state, grid_size, obstacles_positions, w=3))