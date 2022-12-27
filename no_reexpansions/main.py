from complexity import *

Berlin_map = read_map_from_file("Data/Berlin_0_256.map")
Berlin_scen = read_tasks_from_file("Data/Berlin_0_256.map.scen")

draw(Berlin_map, None, None)

nodes_norm, steps_norm, tasks_hard, nodes, steps, task_lens, correct_tasks = count_complexity(Berlin_map, Berlin_scen, 7)

different_w_mean = count_nodes_dif_w(Berlin_map, Berlin_scen, 133)

draw_box_plot(nodes_norm, task_lens)