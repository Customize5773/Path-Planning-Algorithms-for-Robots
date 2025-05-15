#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <cmath>
#include <unordered_map>
#include <algorithm>

using namespace std;

// Node structure for A* algorithm
struct Node {
    int x, y;         // Grid coordinates
    int g;            // Cost from start to current node
    int h;            // Heuristic cost from current node to goal
    Node* parent;     // Parent node

    Node(int x, int y, int g, int h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}

    // Total cost
    int f() const { return g + h; }
};

// Comparator for priority queue
struct CompareNode {
    bool operator()(const Node* a, const Node* b) {
        return a->f() > b->f();
    }
};

class AStarPlanner {
private:
    vector<vector<bool>> grid;       // Grid map (true = obstacle)
    int rows, cols;                 // Grid dimensions
    pair<int, int> start, goal;     // Start and goal positions

    // Heuristic function (Manhattan distance)
    int heuristic(int x1, int y1, int x2, int y2) {
        return abs(x1 - x2) + abs(y1 - y2);
    }

    // Check if a cell is valid
    bool isValid(int x, int y) {
        return x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y];
    }

public:
    AStarPlanner(const vector<vector<bool>>& grid, pair<int, int> start, pair<int, int> goal)
        : grid(grid), start(start), goal(goal) {
        rows = grid.size();
        cols = (rows > 0) ? grid[0].size() : 0;
    }

    // Generate path using A*
    vector<pair<int, int>> findPath() {
        if (!isValid(start.first, start.second) || !isValid(goal.first, goal.second)) {
            cerr << "Invalid start/goal position!" << endl;
            return {};
        }

        priority_queue<Node*, vector<Node*>, CompareNode> open_list;
        vector<vector<bool>> closed_list(rows, vector<bool>(cols, false));

        // Initialize start node
        Node* start_node = new Node(start.first, start.second, 0,
                                    heuristic(start.first, start.second, goal.first, goal.second));
        open_list.push(start_node);

        // Movement directions (up, down, left, right)
        vector<pair<int, int>> dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

        while (!open_list.empty()) {
            Node* current = open_list.top();
            open_list.pop();

            // Check if goal is reached
            if (current->x == goal.first && current->y == goal.second) {
                vector<pair<int, int>> path;
                while (current != nullptr) {
                    path.emplace_back(current->x, current->y);
                    current = current->parent;
                }
                reverse(path.begin(), path.end());
                return path;
            }

            closed_list[current->x][current->y] = true;

            // Explore neighbors
            for (const auto& dir : dirs) {
                int new_x = current->x + dir.first;
                int new_y = current->y + dir.second;

                if (isValid(new_x, new_y) && !closed_list[new_x][new_y]) {
                    int new_g = current->g + 1;
                    int new_h = heuristic(new_x, new_y, goal.first, goal.second);
                    Node* neighbor = new Node(new_x, new_y, new_g, new_h, current);

                    open_list.push(neighbor);
                    closed_list[new_x][new_y] = true; // Simplified for example; use a proper open list check in practice
                }
            }
        }

        cerr << "No path found!" << endl;
        return {};
    }

    // Visualize the grid and path
    void visualizePath(const vector<pair<int, int>>& path) {
        vector<vector<char>> display(rows, vector<char>(cols, '.'));

        // Mark obstacles
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                if (grid[i][j]) display[i][j] = '#';

        // Mark path
        for (const auto& p : path)
            display[p.first][p.second] = '*';

        // Mark start (S) and goal (G)
        display[start.first][start.second] = 'S';
        display[goal.first][goal.second] = 'G';

        // Print the grid
        for (const auto& row : display) {
            for (char c : row) cout << c << " ";
            cout << endl;
        }
    }
};

int main() {
    // Example grid (true = obstacle)
    vector<vector<bool>> grid = {
        {false, false, false, false, false},
        {false, true,  true,  false, false},
        {false, true,  true,  false, false},
        {false, false, false, false, false},
        {false, false, false, false, false}
    };

    AStarPlanner planner(grid, {0, 0}, {4, 4});
    auto path = planner.findPath();

    if (!path.empty()) {
        cout << "Path found:" << endl;
        for (const auto& p : path)
            cout << "(" << p.first << ", " << p.second << ") ";
        cout << "\n\nVisualization:\n";
        planner.visualizePath(path);
    }

    return 0;
}
