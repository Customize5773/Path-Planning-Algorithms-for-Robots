#include "a_star.h"
#include <queue>
#include <cmath>

struct Node {
    int x, y;
    float g, h;
    Node* parent;

    float f() const { return g + h; }

    bool operator>(const Node& other) const {
        return f() > other.f();
    }
};

std::vector<Position> AStar::findPath(const Grid& grid, Position start, Position goal) {
    std::priority_queue<Node*, std::vector<Node*>, 
                        std::function<bool(Node*, Node*)>> openSet(
        [](Node* a, Node* b) { return *a > *b; });

    std::vector<std::vector<bool>> closed(grid.rows, std::vector<bool>(grid.cols, false));
    std::vector<Position> path;

    Node* startNode = new Node{start.x, start.y, 0, heuristic(start, goal), nullptr};
    openSet.push(startNode);

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current->x == goal.x && current->y == goal.y) {
            while (current) {
                path.insert(path.begin(), {current->x, current->y});
                current = current->parent;
            }
            break;
        }

        if (closed[current->x][current->y]) continue;
        closed[current->x][current->y] = true;

        for (const auto& dir : directions) {
            int nx = current->x + dir[0];
            int ny = current->y + dir[1];

            if (grid.isWalkable(nx, ny) && !closed[nx][ny]) {
                Node* neighbor = new Node{
                    nx, ny,
                    current->g + 1,
                    heuristic({nx, ny}, goal),
                    current
                };
                openSet.push(neighbor);
            }
        }
    }

    return path;
}

float AStar::heuristic(Position a, Position b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y); // Manhattan distance
}
