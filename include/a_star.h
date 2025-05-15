#ifndef A_STAR_H
#define A_STAR_H

#include <vector>

struct Position {
    int x;
    int y;
};

class Grid {
public:
    int rows, cols;
    std::vector<std::vector<int>> map;

    Grid(int r, int c) : rows(r), cols(c), map(r, std::vector<int>(c, 0)) {}

    bool isWalkable(int x, int y) const {
        return x >= 0 && y >= 0 && x < rows && y < cols && map[x][y] == 0;
    }
};

class AStar {
public:
    static std::vector<Position> findPath(const Grid& grid, Position start, Position goal);

private:
    static float heuristic(Position a, Position b);
    inline static const int directions[4][2] = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0}  // 4-way movement (NESW)
    };
};

#endif // A_STAR_H
