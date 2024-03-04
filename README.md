# Obstacle Avoidance

This mini-project implements the A* path-finding algorithm on a 2D grid array.

## Assumptions

1. The grid has 0-based indexing. All row and column indices begin with zero.
2. The robot is thus placed at (0,0) initially.
3. The diameter of an obstacle affects only the x-coordinates and y-coordinates in a fashion mentioned below:

```
Grid size: 5x5
obstacle position: (2,2)
obstacle diameter: 2

initial grid:                                           grid after obstacle is placed:
 0 0 0 0 0                                              0 0 0 0 0 
 0 0 0 0 0                                              0 1 1 1 0 
 0 0 0 0 0                                              0 1 1 1 0 
 0 0 0 0 0                                              0 1 1 1 0 
 0 0 0 0 0                                              0 0 0 0 0

```
4. The robot is allowed to traverse diagonally between elements.
5. Obstacles placed on the grid having diameters overflowing will be considered half-in, half-out, as follows:
```
Grid size: 5x5
obstacle position: (0,2)
obstacle diameter: 2
 
 0 1 1 1 0
 0 1 1 1 0
 0 0 0 0 0
 0 0 0 0 0
 0 0 0 0 0

```

## Compilation Instructions
- Run the ```main.cpp``` file
- Enter the grid dimensions as follows: for a 10x10 grid, enter ```10 10```
- Enter the number of obstacles
- For each obstacle, enter the coordinates: If you want an obstacle at (3,4), enter ```3 4```
- Enter the diameter of the obstacle in an integer format.

6. The waypoints are generated in order, and are stored in a vector of integers.
Sample Waypoint: {3, 4, 6, 8}
Where the first two elements are the x and y coordinates of the waypoints, the third and fourth elements is the Manhattan Distance and the Euclidean Distance between the start position and the waypoint, respectively.

## Sample Code
Input:
```
Enter grid size in row and column form: 
10 10
Enter number of obstacles to be considered: 2
Enter obstacle position 1 in spaced format: 3 4
Enter obstacle diameter 1 in spaced format: 1

Enter obstacle position 2 in spaced format: 6 6
Enter obstacle diameter 2 in spaced format: 2
```
Output:
```
Grid with Obstacles:
 0 0 0 0 0 0 0 0 0 0
 0 0 0 0 0 0 0 0 0 0
 0 0 0 0 0 0 0 0 0 0
 0 0 0 0 1 0 0 0 0 0
 0 0 0 0 0 0 0 0 0 0
 0 0 0 0 0 1 1 1 0 0
 0 0 0 0 0 1 1 1 0 0
 0 0 0 0 0 1 1 1 0 0
 0 0 0 0 0 0 0 0 0 0
 0 0 0 0 0 0 0 0 0 0
Waypoints: 
Waypoint: (0, 0)
Manhattan Distance from Start: 0
Euclidean Distance from Start: 0
Waypoint: (2, 1)
Manhattan Distance from Start: 3
Euclidean Distance from Start: 2
Waypoint: (8, 6)
Manhattan Distance from Start: 14
Euclidean Distance from Start: 10
Waypoint: (1, 8)
Manhattan Distance from Start: 9
Euclidean Distance from Start: 8
Waypoint: (4, 2)
Manhattan Distance from Start: 6
Euclidean Distance from Start: 4

The Robot is marked with the letter R. 
final_path : 
(0, 0)
(1, 1)
(2, 1)
(3, 2)
(4, 3)
(5, 4)
(6, 4)
(7, 4)
(8, 5)
(8, 6)
(8, 7)
(7, 8)
(6, 8)
(5, 8)
(4, 8)
(3, 8)
(2, 8)
(1, 8)
(2, 7)
(3, 6)
(4, 5)
(4, 4)
(4, 3)
(4, 2)

Grid with final_path: 

 R 0 0 0 0 0 0 0 0 0
 0 R 0 0 0 0 0 0 R 0
 0 R 0 0 0 0 0 R R 0
 0 0 R 0 1 0 R 0 R 0
 0 0 R R R R 0 0 R 0
 0 0 0 0 R 1 1 1 R 0
 0 0 0 0 R 1 1 1 R 0
 0 0 0 0 R 1 1 1 R 0
 0 0 0 0 0 R R R 0 0
 0 0 0 0 0 0 0 0 0 0 

```


## To Be Done
- [x] Add provision of selecting waypoints in the grid such that they don't lie in an obstacle, and aid in exploring the entire grid
- [x] Improve the way grid sizes and obstacle positions are read
- [ ] Optimization of the algorithm
- [ ] Division into multiple files for readability
