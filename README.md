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
Grid with obstacles: 
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
Path found.
The Robot is marked with the letter R. 
Path : 
(0, 0)
(1, 1)
(2, 2)
(3, 3)
(4, 4)
(5, 4)
(6, 4)
(7, 4)
(8, 5)
(9, 6)
(9, 7)
(9, 8)
(9, 9)

Grid with Path: 

 R 0 0 0 0 0 0 0 0 0
 0 R 0 0 0 0 0 0 0 0
 0 0 R 0 0 0 0 0 0 0
 0 0 0 R 1 0 0 0 0 0
 0 0 0 0 R 0 0 0 0 0
 0 0 0 0 R 1 1 1 0 0
 0 0 0 0 R 1 1 1 0 0
 0 0 0 0 R 1 1 1 0 0
 0 0 0 0 0 R 0 0 0 0
 0 0 0 0 0 0 R R R R 
 ```

