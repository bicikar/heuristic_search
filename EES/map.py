from PIL import Image, ImageDraw
import numpy as np
import matplotlib.pyplot as plt
import math
from heapq import heappop, heappush
import os

class Map:

    def __init__(self):
        '''
        Default constructor
        '''

        self._width = 0
        self._height = 0
        self._cells = []
        self._dust = []
        self._obstacles = []
        self.visited_draw = {}
    

    def read_from_string(self, cell_str, width, height):
        '''
        Converting a string (with '#' representing obstacles and '.' representing free cells) to a grid
        '''
        self._width = width
        self._height = height
        self._cells = [[0 for _ in range(width)] for _ in range(height)]
        cell_lines = cell_str.split("\n")
        i = 0
        j = 0
        for l in cell_lines:
            if len(l) != 0:
                j = 0
                for c in l:
                    if c == '.':
                        self._cells[i][j] = 0
                    elif c == '#':
                        self._cells[i][j] = 1
                    else:
                        continue
                    j += 1
                if j != width:
                    raise Exception("Size Error. Map width = ", j, ", but must be", width )
                
                i += 1

        if i != height:
            raise Exception("Size Error. Map height = ", i, ", but must be", height )
    
     
    def set_grid_cells(self, width, height, grid_cells, dust, obstacles):
        '''
        Initialization of map by list of cells.
        '''
        self._width = width
        self._height = height
        self._cells = grid_cells
        self._dust = dust
        self._obstacles = obstacles


    def in_bounds(self, i, j):
        '''
        Check if the cell is on a grid.
        '''
        return (0 <= j < self._width) and (0 <= i < self._height)
    

    def traversable(self, i, j):
        '''
        Check if the cell is not an obstacle.
        '''
        return not self._cells[i][j]


    def get_neighbors(self, i, j):
        neighbors = []

        for di in [-1, 0, 1]:
            for dj in [-1, 0, 1]:
                if di == 0 and dj == 0:
                    continue
                n_i, n_j = i + di, j + dj
                if self.in_bounds(n_i, n_j) and \
                    self.traversable(n_i, n_j) and \
                    self.traversable(n_i, j) and \
                    self.traversable(i, n_j):
                    neighbors.append((n_i, n_j))

        return neighbors

    def get_size(self):
        return (self._height, self._width)

def compute_cost(i1, j1, i2, j2):
    '''
    Computes cost of simple moves between cells
    '''
    if abs(i1 - i2) + abs(j1 - j2) == 1:
        return 1
    elif abs(i1 - i2) + abs(j1 - j2) == 2:
        return math.sqrt(2)
    else:
        raise Exception('Trying to compute the cost of a non-supported move!')


def draw(grid_map, start = None, name = 'tmp.png', path = None, nodes_opened = None, nodes_expanded = None, nodes_reexpanded = None):
    '''
    Auxiliary function that visualizes the environment, the path and 
    the open/expanded/re-expanded nodes.
    
    The function assumes that nodes_opened/nodes_expanded/nodes_reexpanded
    are iterable collestions of SearchNodes
    '''
    k = 15
    height, width = grid_map.get_size()
    h_im = height * k
    w_im = width * k
    im = Image.new('RGB', (w_im, h_im), color = 'white')
    draw = ImageDraw.Draw(im)
    
    for i in range(height):
        for j in range(width):
            if(not grid_map.traversable(i, j)):
                draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=( 70, 80, 80 ))

    if nodes_opened is not None:
        for node in nodes_opened:
            draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(213, 219, 219), width=0)
    
    if nodes_expanded is not None:
        for node in nodes_expanded:
            draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(131, 145, 146), width=0)
    
    if nodes_reexpanded is not None:
        for node in nodes_reexpanded:
                draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(255, 145, 146), width=0)
    
    if path is not None:
        for step_n in path:
            if (step_n is not None):
                step = step_n.state.agent_position
                if (grid_map.traversable(step[0], step[1])):
                    if ' '.join(str(e) for e in step) not in grid_map.visited_draw.keys():
                        draw.rectangle((step[1] * k, step[0] * k, (step[1] + 1) * k - 1, (step[0] + 1) * k - 1), fill=(52, 152, 219), width=0)
                        grid_map.visited_draw[' '.join(str(e) for e in step)] = 2
                    else:
                        coef = grid_map.visited_draw[' '.join(str(e) for e in step)]
                        draw.rectangle((step[1] * k, step[0] * k, (step[1] + 1) * k - 1, (step[0] + 1) * k - 1), fill=(100 + 10 * coef, 20 + 20 * coef, 150 + 8 * coef), width=0)
                        grid_map.visited_draw[' '.join(str(e) for e in step)] += 1
                else:
                    draw.rectangle((step[1] * k, step[0] * k, (step[1] + 1) * k - 1, (step[0] + 1) * k - 1), fill=(230, 126, 34), width=0)

    if (start is not None) and (grid_map.traversable(start[0], start[1])):
        draw.rectangle(((start[1]) * k, (start[0]) * k, (start[1] + 1) * k - 1, (start[0] + 1) * k - 1), fill=(40, 180, 99), width=0)

    for i, j in grid_map._dust:
        draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=( 128, 55, 0 ))
    
    _, ax = plt.subplots(dpi=250)
    ax.axes.xaxis.set_visible(False)
    ax.axes.yaxis.set_visible(False)
    # plt.imshow(np.asarray(im))
    # plt.show()
    dir = os.path.dirname(os.path.abspath(__file__))
    im.save(dir + '/result_vacuum/' + name, "PNG")
    grid_map.visited_draw.clear()
    # plt.savefig(dir + '/result_vacuum/' + name)


def draw_gif(grid_map, start = None, name = 'tmp.png', path = None, nodes_opened = None, nodes_expanded = None, nodes_reexpanded = None):
    '''
    Auxiliary function that visualizes the environment, the path and 
    the open/expanded/re-expanded nodes.
    
    The function assumes that nodes_opened/nodes_expanded/nodes_reexpanded
    are iterable collestions of SearchNodes
    '''
    k = 15
    height, width = grid_map.get_size()
    h_im = height * k
    w_im = width * k
    im = Image.new('RGB', (w_im, h_im), color = 'white')
    draw = ImageDraw.Draw(im)
    
    for i in range(height):
        for j in range(width):
            if(not grid_map.traversable(i, j)):
                draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=( 70, 80, 80 ))

    if nodes_opened is not None:
        for node in nodes_opened:
            draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(213, 219, 219), width=0)
    
    if nodes_expanded is not None:
        for node in nodes_expanded:
            draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(131, 145, 146), width=0)
    
    if nodes_reexpanded is not None:
        for node in nodes_reexpanded:
                draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(255, 145, 146), width=0)
    
    if path is not None:
        for step_n in path:
            if (step_n is not None):
                step = step_n.state.agent_position
                if (grid_map.traversable(step[0], step[1])):
                    hsh = ' '.join(str(e) for e in step)
                    if hsh not in grid_map.visited_draw.keys() or (hsh in grid_map.visited_draw.keys() and grid_map.visited_draw[hsh] == 0):
                        draw.rectangle((step[1] * k, step[0] * k, (step[1] + 1) * k - 1, (step[0] + 1) * k - 1), fill=(52, 152, 219), width=0)
                    else:
                        coef = grid_map.visited_draw[hsh]
                        draw.rectangle((step[1] * k, step[0] * k, (step[1] + 1) * k - 1, (step[0] + 1) * k - 1), fill=(100 + 10 * coef, 20 + 20 * coef, 150 + 8 * coef), width=0)
                else:
                    draw.rectangle((step[1] * k, step[0] * k, (step[1] + 1) * k - 1, (step[0] + 1) * k - 1), fill=(230, 126, 34), width=0)

    if (start is not None) and (grid_map.traversable(start[0], start[1])):
        draw.rectangle(((start[1]) * k, (start[0]) * k, (start[1] + 1) * k - 1, (start[0] + 1) * k - 1), fill=(40, 180, 99), width=0)

    for i, j in grid_map._dust:
        draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=( 128, 55, 0 ))

    return im

def make_gif(map, start, path, name='tmp'):
    images = []

    for i in range(1, len(path)):
        tmp_path = path[:i]
        images.append(draw_gif(map, start=start, path=tmp_path[:i], name=name))
        step_n = tmp_path[-1]
        step = step_n.state.agent_position
        if ' '.join(str(e) for e in step) not in map.visited_draw.keys():
            map.visited_draw[' '.join(str(e) for e in step)] = 0
        else:
            map.visited_draw[' '.join(str(e) for e in step)] += 2

    dir = os.path.dirname(os.path.abspath(__file__))
    images[0].save(dir + '/result_vacuum/' + name + '.gif', save_all=True, append_images=images[1:], duration=40, loop=0)
    map.visited_draw.clear()

def read_map_from_file(path):
    dir = os.path.dirname(os.path.abspath(__file__))
    with open(dir + "/" + path) as f:
        h = int(f.readline().split()[-1])
        w = int(f.readline().split()[-1])
        dust_size = int(f.readline().split()[-1])
        dust = []
        for _ in range(dust_size):
            y, x = map(int, f.readline().split())
            dust.append([y, x])
        grid = [[0] * w for _ in range(h)]
        obstacles = []
        for i in range(h):
            line = f.readline()
            for j in range(w):
                if line[j] in '.G':
                    grid[i][j] = 0
                else:
                    grid[i][j] = 1
                    obstacles.append([i, j])
    res = Map()
    res.set_grid_cells(w, h, grid, dust, obstacles)
    return res

def read_tasks_from_file(path):
    tasks = []
    dir = os.path.dirname(os.path.abspath(__file__))

    with open(dir + '/' + path) as f:
        f.readline()
        for line in f.readlines():
            line = line.split()
            tasks.append([
                int(line[0]),
                line[1],
                int(line[2]),
                int(line[3]),
                int(line[5]),
                int(line[4]),
                int(line[7]),
                int(line[6]),
                float(line[8]),
            ])

    return tasks 



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