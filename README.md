# A* Path Planning Algorithm for Robotics

![a_-search-algorithm-1](https://github.com/user-attachments/assets/6f9e30d7-1dac-47d7-867a-090eb9791c93)
> Example of A pathfinding in a 2D grid environment*

## Overview
This C++ implementation of the A* algorithm enables optimal path planning for robots in 2D grid environments with obstacles. The solution includes:

- Grid-based navigation with obstacle avoidance
- Manhattan distance heuristic
- 4-direction movement (up/down/left/right)
- ASCII visualization of paths
- Error checking for invalid positions

## Features

- **Grid Configuration**: Define custom grid layouts with obstacles
- **Optimal Pathfinding**: Guarantees shortest path in 4-direction grids
- **Visualization**: ASCII display showing:
  - Start [`S`] and goal [`G`] positions
  - Obstacles [`#`]
  - Calculated path [`*`]
- **Extendable**: Easy to modify for:
  - 8-direction movement
  - Different heuristic functions
  - Dynamic obstacles
## Dependencies
- C++11 or newer
- Standard Template Library (STL)

## Usage

1. **Define Grid Environment:**
```cpp
vector<vector<bool>> grid = {
    {false, false, false, false, false},
    {false, true,  true,  false, false},
    {false, true,  true,  false, false},
    {false, false, false, false, false},
    {false, false, false, false, false}
};
```

2. **Initialize Planner:**
```cpp
AStarPlanner planner(grid, {0, 0}, {4, 4});  // Start(0,0), Goal(4,4)
```

3. **Find Path:**
```cpp
auto path = planner.findPath();
```

4. **Visualize Results:**
```cpp
planner.visualizePath(path);
```

## Example Output
```
Path found:
(0, 0) (0, 1) (0, 2) (0, 3) (0, 4) (1, 4) (2, 4) (3, 4) (4, 4) 

Visualization:
S . . . . 
# # . . . 
# # . . . 
. . . . . 
. . . . G 
```

## Customization Options

1. 8-Direction Movement
```cpp
// In findPath() method:
vector<pair<int, int>> dirs = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},  // Original 4 directions
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1} // Diagonal directions
};
```

2. Different Heuristic
```cpp
// Euclidean distance:
int heuristic(int x1, int y1, int x2, int y2) {
    return static_cast<int>(sqrt(pow(x1-x2, 2) + pow(y1-y2, 2)));
}
```

---
![a_-search-algorithm-2](https://github.com/user-attachments/assets/67f19771-85a3-4cf9-9f9c-355df105d2bd)

---
![967daf50-5334-4f6b-9527-af4308a3b2f1](https://github.com/user-attachments/assets/0eead2bc-a250-4a05-a145-75162b77aaa8)

---
![Graph](https://github.com/user-attachments/assets/9cc94eca-49c7-4e50-9e1a-b01a82d3e787)

---

## Possible Improvements

1. Add path smoothing algorithms (e.g., Bézier curves)
2. Implement dynamic obstacle updating
3. Include costmap support for varied terrain costs
4. Add ROS integration for robotic applications
5. Implement more efficient priority queue handling

### License
GNU General Public License v3.0 - Free for academic and commercial use with attribution.
