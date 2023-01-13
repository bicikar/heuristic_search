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

All input files should be presented in `Data/` folder

## 1. Conditions for Avoiding Node Re-expansions in Bounded Suboptimal Search

The main goal of this approach is to develop a set of conditions on the priority function used by a best-first search that ensures that the best-first search will find a bounded sub-optimal solution without re-expanding states. Two specific priority functions were developed from a larger class that have this property.

Algorithm runs WA* with three different priority functions:
- $\Phi(x, y) = x + \frac{1}{w}\cdot y$
- $\Phi_{XDP}(x, y) = \frac{1}{2w} \left( y + (2w-1)x + \sqrt{(y-x)^2} + 4wyx \right)$
- $\Phi_{XUP} (x,y) = \frac{1}{2w}\left(y + x + \sqrt{(y+x)^2 + 4w(w-1)x^2}\right)$

### 1.1 Path finding problem
Run with command `python3 run_path file_name start_i start_j goal_i goal_j w`

Search algorithms takes on input:
- `file_name` - path to the field file. Correct represenation of the file below.
- `start_i, start_j` - starting position of the agent
- `goal_i, goal_j` - ending postion of the agent
- `w` - suboptimality bound. If not presented then equals to 2

Field represantation:
```
height *map_height*
width *map_width*
map
*Map represtation*
```
Map representation consists of `*map_height*` rows, each of which contatins `*map_width*` symbols. Each symbol should be encoded as `@` for obstacle position and as `.` for free position

#### Output:

Algorithm returns small statistic about each algorithm result. Example:

```
A* summary:
Working time: 0.21379  ; Nodes created: 27757 ; Path length: 253.63456

WA* summary, weight 2:
Working time: 0.02316  ; Nodes created: 3278 ; Path length: 267.71782

Phi XDP summary, weight 2:
Working time: 0.05396  ; Nodes created: 3394 ; Path length: 266.06097

Phi XUP summary, weight 2:
Working time: 0.02333  ; Nodes created: 2944 ; Path length: 267.71782
```

Drawn founded paths are saved in `result_path/` folder.

![Пример отрисовки](/result_path/phi_xdp)

### 1.2 15 puzzle problem
Run with command `python3 run_puzzle file_name w random_flag`
Search algorithms takes on input:
- `file_name` - path to the field file. Correct represenation of the file below.
- `w` - suboptimality bound. If not presented then equals to 2
- `random_flag` - generate random 15-puzzle in the file if flag presented.

`file_name` should consist of 1 row with shifted numbers from 1 to $n^2$, where $n$ is the size of 15-puzzle. If 15-puzzle is not solvable, then the algorithm will panic. Example of `file_name` content:

`11 12 1 10 9 6 2 5 3 4 16 15 14 8 7 13`

#### Ouput:

Algorithm returns small statistic about each priority function result.

The solution for each priority function is saved in result_puzzle 



## 2. EES

Algotihm is implemented as standard Best First Search algorithm, but uses 3 queues with different metrics to take best node. Using $\hat{h}$ and $\hat{d}$ functions, it explicitly estimates the cost and distance-to-go for search guidance, relying on lower bounds solely for providing sub-optimality guarantees. 

Implemented for a Vacuum world problem.

Algorithm runs WA* with three different priority functions from part 1 of this repository and also runs EES algorithm.

Run with command `python3 run_puzzle file_name start_i, start_j w gif_flag`

Search algorithms takes on input:
- `file_name` - path to the field file. Correct represenation of the file below.
- `start_i, start_j` - starting position of the agent
- `w` - suboptimality bound.
- `gif_flag` - if set, then algorithms also draws gif's of how agent goes through the field. If not set, algorithm only presents final images with full path.

#### Ouput:

Algorithm returns small statistic about each priority function result.

Pictures of the path are saved in `result_vacuum` directory. If `gif_flag` was active, animations will be saved in the same directory

![Пример отрисовки](/result_vacuum/ees.gif)