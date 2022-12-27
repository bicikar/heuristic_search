# Bounded suboptimal algorithms

## Description

A software project at the SPBU. 
2 bounded suboptimal algorithms are presented.

1. [Conditions for Avoiding Node Re-expansions in Bounded Suboptimal Search](https://www.ijcai.org/proceedings/2019/0170.pdf)
2. [Explicit Estimation Search](https://www.aaai.org/ocs/index.php/IJCAI/IJCAI11/paper/viewFile/3366/3588)

This project uses Python and libraries:

- matplotlib
- numpy
- pandas
- Pillow

## 1. Conditions for Avoiding Node Re-expansions in Bounded Suboptimal Search

The main goal of this approach is to develop a set of conditions on the priority function used by a best-first search that ensures that the best-first search will find a bounded sub-optimal solution without re-expanding states. Two specific priority functions were developed from a larger class that have this property.

### Input

Search algorithms takes on input:
- `grid_map` - path to `.map` and `.scen` files which decribe the field and the tasks. Examples can be seen in `Data/` folder. All maps and tasks were taken from the [website](https://movingai.com/benchmarks/grids.html)
- `start_i, start_j` - starting position of the agent
- `goal_i, goal_j` - ending postion of the agent
- `w` - suboptimality bound
- `heuristic_func` - admissible heuristic function. Example can be seen in `heuristics.py `
- `search_tree` - searching tree. Default presented in `definitions.py`, 
- `prioritet` - prioritet function. Examples presented in `heuristics.py`

As output:
`path_was_found (True / False), ending_node, steps, nodes_created, opened_vertices, closed_vertices`

And a drawing of founded path:

![Пример отрисовки](/images/no-reop.jpeg)

