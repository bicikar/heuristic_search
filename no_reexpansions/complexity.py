from search import *
from map import *
from heuristics import *
from definitions import *

from IPython.core.display import display
import pandas as pd
import numpy as np

'''
Функция, принимающая map (карта), scen (задания) и iter, и применяющая a-star к каждому iter-ому заданию по всем
доступным эвристикам. Возвращает массивы:
nodes_norm[i] - набор нормализованных значений количества вершин для всех A* и WA* с заданными весами для текущего задания
steps_norm[i] - набор нормализованных значений количества шагов для всех A* и WA* с заданными весами для текущего задания
task_hard[i] - сложность текущего задания, вычисляется как длина кратчайшего пути
nodes[i] - набор не нормализованных значений количества вершин для всех A* и WA* с заданными весами для текущего задания
steps[i] - набор не нормализованных значений количества шагов для всех A* и WA* с заданными весами для текущего задания
task_len[i] - набор нормализованных длин найденных путей для всех A* и WA* с заданными весами для текущего задания
correct_tasks - подсчитывает количество найденных кратчайших путей для A* и WA* с заданными весами
'''

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


def count_complexity(task_map, scen, iter):
    nodes_norm = []
    nodes = []
    steps_norm = []
    steps = []
    task_n = 0
    task_hard = []
    task_len = []
    correct_tasks = [0, 0, 0, 0]
    weights = [1.25, 1.5, 3, 5, 10]
#     distances = [manhattan_distance, chebyshev_distance, diagonal_distance, euclidean_distance]
    w = 10
    for i, task in enumerate(scen):
        if i % iter == 0:
            cur_task_nodes = []
            cur_task_steps = []
            cur_task_len = []
            task_n += 1
            
            result = astar(task_map, task[4], task[5], task[6], task[7], 1, diagonal_distance, SearchTreePQS, phi)
            number_of_steps = result[2]
            nodes_created = result[3]
            length = result[1].g
            correct = (length - task[8] < 0.01)
            if correct:
                correct_tasks[0] += 1
            cur_task_nodes.append(nodes_created)
            cur_task_steps.append(number_of_steps)
            task_hard.append(length)
            cur_task_len.append(length / task[8])
            
            for j in range(3):
                if j == 0:
                    result = astar(task_map, task[4], task[5], task[6], task[7], w, diagonal_distance, SearchTreePQS, phi)
                if j == 1:
                    result = astar(task_map, task[4], task[5], task[6], task[7], w,  diagonal_distance, SearchTreePQS, phi_xdp)
                if j == 2:
                    result = astar(task_map, task[4], task[5], task[6], task[7], w,  diagonal_distance, SearchTreePQS, phi_xup)
                number_of_steps = result[2]
                nodes_created = result[3]
                length = result[1].g
                correct = (length - task[8] < 0.01)
                if correct:
                    correct_tasks[j + 1] += 1
                cur_task_nodes.append(nodes_created)
                cur_task_steps.append(number_of_steps)
                cur_task_len.append(length / task[8])
                
            max_steps = max(cur_task_steps)
            max_nodes = max(cur_task_nodes)
            nodes.append(cur_task_nodes)
            steps.append(cur_task_steps)
            cur_task_nodes = [i / max_nodes for i in cur_task_nodes]
            cur_task_steps = [i / max_steps for i in cur_task_steps]
            task_len.append(cur_task_len)
            nodes_norm.append(cur_task_nodes)
            steps_norm.append(cur_task_steps)
            if i % (iter * 10) == 0:
                print("Going through task {}".format(i))
    print("Tasks: {}".format(task_n))
    return nodes_norm, steps_norm, task_hard, nodes, steps, task_len, correct_tasks

def count_nodes_dif_w(task_map, scen, iter):
    nodes_norm = [[], [], [], [], []]
    task_n = 0
    task_len = [[], [], [], [], []]
    correct_tasks = [0, 0, 0, 0]
    weights = [1.25, 1.5, 3, 5, 10]
    
#     distances = [manhattan_distance, chebyshev_distance, diagonal_distance, euclidean_distance]
    for i, task in enumerate(scen):
        if i % iter == 0:
            
            task_n += 1
            
            result = astar(task_map, task[4], task[5], task[6], task[7], 1, diagonal_distance, SearchTreePQS, phi)
            number_of_steps = result[2]
            nodes_a_star = result[3]
            length = result[1].g
#             correct = (length - task[8] < 0.01)
#             if correct:
#                 correct_tasks[0] += 1
#             cur_task_nodes.append(nodes_created)
#             task_hard.append(length)
#             cur_task_len.append(length / task[8])
            
            for k, w in enumerate(weights):
                cur_task_nodes = []
                cur_task_len = []
                for j in range(3):
                    if j == 0:
                        result = astar(task_map, task[4], task[5], task[6], task[7], w, diagonal_distance, SearchTreePQS, phi)
                    if j == 1:
                        result = astar(task_map, task[4], task[5], task[6], task[7], w,  diagonal_distance, SearchTreePQS, phi_xdp)
                    if j == 2:
                        result = astar(task_map, task[4], task[5], task[6], task[7], w,  diagonal_distance, SearchTreePQS, phi_xup)
                    nodes_created = result[3]
                    length = result[1].g
#                     correct = (length - task[8] < 0.01)
#                     if correct:
#                         correct_tasks[j + 1] += 1
                    cur_task_nodes.append(nodes_created)
#                     cur_task_steps.append(number_of_steps)
                    cur_task_len.append(length / task[8])
                
                max_node = max(cur_task_nodes)
#                 nodes.append(cur_task_nodes)
#                 steps.append(cur_task_steps)
                cur_task_nodes = [i / max_node for i in cur_task_nodes]
#                 cur_task_steps = [i / max_steps for i in cur_task_steps]
                task_len.append(cur_task_len)
                nodes_norm[k].append(cur_task_nodes)
#                 steps_norm.append(cur_task_steps)
#                 cur_task_nodes = cur_task_nodes[:1]
#                 print("aoaoaoao", i, cur_task_nodes)
#                 cur_task_len = cur_task_len[:1]
            if i % (iter * 10) == 0:
                print("Going through task {}".format(i))
    print("Tasks: {}".format(task_n))
    
    nodes_norm_mean = []
    for w_res in nodes_norm:
        print(w_res)
        cur_nodes_mean = []
        for alg_ind in range(len(w_res[0])):
            w_i = [task[alg_ind] for task in w_res]
            cur_nodes_mean.append(np.mean(w_i))
        nodes_norm_mean.append(cur_nodes_mean)
    
    print(nodes_norm_mean)
    return nodes_norm_mean

def draw_df(nodes_norm, steps_norm, correct_tasks, task_lens, tasks_num):
    df = pd.DataFrame(index=['correct', 'length', 'nodes_created', 'steps'])
    a_nodes = [i[0] for i in nodes_norm]
    a_steps = [i[0] for i in steps_norm]
    a_lens = [i[0] for i in task_lens]
    df["A*"] = [
        correct_tasks[0] / tasks_num,
        np.mean(a_lens),
        np.mean(a_nodes),
        np.mean(a_steps),
    ]
    
    w_nodes = [i[1] for i in nodes_norm]
    w_steps = [i[1] for i in steps_norm]
    w_lens = [i[1] for i in task_lens]
    df["WA*"] = [
        correct_tasks[1] / tasks_num,
        np.mean(w_lens),
        np.mean(w_nodes),
        np.mean(w_steps),
    ]
    
    w_nodes = [i[2] for i in nodes_norm]
    w_steps = [i[2] for i in steps_norm]
    w_lens = [i[2] for i in task_lens]
    df["XDP"] = [
        correct_tasks[2] / tasks_num,
        np.mean(w_lens),
        np.mean(w_nodes),
        np.mean(w_steps),
    ]
    
    w_nodes = [i[3] for i in nodes_norm]
    w_steps = [i[3] for i in steps_norm]
    w_lens = [i[3] for i in task_lens]
    df["XUP"] = [
        correct_tasks[3] / tasks_num,
        np.mean(w_lens),
        np.mean(w_nodes),
        np.mean(w_steps),
    ]
    
    df.iloc[2] /= np.max(df.iloc[2])
    df.iloc[3] /= np.max(df.iloc[3])
    

def draw_df_dif_w(nodes_norm):
    df = pd.DataFrame(index=['w=1.25', 'w=1.5', 'w=3', 'w=5', 'w=10'])
    
    df.style.set_table_attributes('style="font-size: 22px"')
#     a_nodes = [i[0] for i in nodes_norm]
#     a_steps = [i[0] for i in steps_norm]
#     a_lens = [i[0] for i in task_lens]
#     df["A*"] = [
#         correct_tasks[0] / tasks_num,
#         np.mean(a_lens),
#         np.mean(a_nodes),
#         np.mean(a_steps),
#     ]
    
    w_nodes = [i[0] for i in nodes_norm]
    df["WA*"] = [
        i[0] for i in nodes_norm
    ]
    
    df["XDP"] = [
        i[1] for i in nodes_norm
    ]
    
    df["XUP"] = [
        i[2] for i in nodes_norm
    ]
    

    df.iloc[2] /= np.max(df.iloc[2])
    df.iloc[3] /= np.max(df.iloc[3])
    return df.style.set_properties(**{
    'font-size': '17pt',
})

def draw_box_plot(nodes_norm, task_lens):
    fig = plt.figure(figsize =(8, 5))
    fig.suptitle('Nodes count')
    ax = fig.add_axes([0, 0, 1, 1])
    data_nodes = []
    for i in range(4):
        data_nodes.append([el[i] for el in nodes_norm])
    bp = ax.boxplot(data_nodes, labels=["A*", "WA*", "XDP", "XUP"])
    plt.ylabel('Normalised nodes count')
    plt.show()
    
    fig = plt.figure(figsize =(8, 5))
    fig.suptitle('Path length')
    ax = fig.add_axes([0, 0, 1, 1])
    data_nodes = []
    for i in range(4):
        data_nodes.append([el[i] for el in task_lens])
    bp = ax.boxplot(data_nodes, labels=["A*", "WA*", "XDP", "XUP"])
    plt.ylabel('Normalised path length')
    plt.show()

def draw_scatter(nodes_norm, tasks_hard, task_lens):
    fig = plt.figure(figsize =(8, 5))
    fig.suptitle('Nodes count / path length')
    ax = fig.add_axes([0, 0, 1, 1])
    names = ["A*", "WA* 1.05", "WA* 1.1", "WA* 1.5", "WA* 2", "WA* 3", "WA* 5"]
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    for i in range(7):
        z = np.polyfit(tasks_hard, [el[i] for el in nodes_norm], 1)
        p = np.poly1d(z)
        ax.scatter(tasks_hard, [el[i] for el in nodes_norm], alpha=0.2, color=colors[i])
        ax.plot(tasks_hard, p(tasks_hard), label=names[i], linewidth=2.0, color=colors[i])
    plt.xlabel('Shortest path')
    plt.ylabel('Nodes created')
    ax.legend()
    plt.show()
    
    fig = plt.figure(figsize =(8, 5))
    fig.suptitle('Task lens / path length')
    ax = fig.add_axes([0, 0, 1, 1])
    names = ["A*", "WA* 1.05", "WA* 1.1", "WA* 1.5", "WA* 2", "WA* 3", "WA* 5"]
    for i in range(7):
        z = np.polyfit(tasks_hard, [el[i] for el in task_lens], 1)
        p = np.poly1d(z)
        ax.plot(tasks_hard, p(tasks_hard), linewidth=1.5, label=names[i], color=colors[i])
    plt.xlabel('Shortest path')
    plt.ylabel('Length of found path')
    ax.legend()
    plt.show()