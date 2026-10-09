#include "third_party/nlohmann/json.hpp"

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

struct Position {
    int x;
    int y;
};

struct Mission {
    int width;
    int height;
    Position start;
    Position target;
    std::vector<Position> obstacles;
};

struct PlanningResult {
    std::vector<Position> path;
    std::size_t explored_nodes = 0;
};

int read_integer(const nlohmann::json& value, const std::string& field_name) {
    if (!value.is_number_integer()) {
        throw std::runtime_error(field_name + " must be an integer.");
    }

    // Check the range before conversion; get<int>() alone can narrow values.
    if (value.is_number_unsigned()) {
        const auto number = value.get<unsigned long long>();
        if (number > static_cast<unsigned long long>(std::numeric_limits<int>::max())) {
            throw std::runtime_error(field_name + " is outside the int range.");
        }
    } else {
        const auto number = value.get<long long>();
        if ((number < std::numeric_limits<int>::min())
            || (number > std::numeric_limits<int>::max())) {
            throw std::runtime_error(field_name + " is outside the int range.");
        }
    }

    return value.get<int>();
}

std::vector<Position> read_obstacles(const nlohmann::json& terrain) {
    // Older mission files may omit obstacles and use an empty grid.
    if (!terrain.contains("obstacles")) {
        return {};
    }

    const auto& values = terrain.at("obstacles");
    if (!values.is_array()) {
        throw std::runtime_error("terrain.obstacles must be an array.");
    }

    std::vector<Position> obstacles;
    for (const auto& value : values) {
        const Position obstacle{
            read_integer(value.at("x"), "terrain.obstacles.x"),
            read_integer(value.at("y"), "terrain.obstacles.y")
        };
        obstacles.push_back(obstacle);
    }

    return obstacles;
}

Mission load_mission(const std::string& file_path) {
    std::ifstream input(file_path);
    if (!input.is_open()) {
        throw std::runtime_error("Cannot open mission file: " + file_path);
    }

    const nlohmann::json data = nlohmann::json::parse(input);
    const auto& terrain = data.at("terrain");
    const auto& start = data.at("start");
    const auto& target = data.at("target");

    // at() rejects missing keys or non-object parents.
    return {
        read_integer(terrain.at("width"), "terrain.width"),
        read_integer(terrain.at("height"), "terrain.height"),
        {
            read_integer(start.at("x"), "start.x"),
            read_integer(start.at("y"), "start.y")
        },
        {
            read_integer(target.at("x"), "target.x"),
            read_integer(target.at("y"), "target.y")
        },
        read_obstacles(terrain)
    };
}

bool is_inside_grid(
    const Position position,
    const int width, 
    const int height
) {
    return (position.x >= 0 && position.x < width)
        && (position.y >= 0 && position.y < height);
}

void print_position(const Position position) {
    std::cout << "Rover position: (" << position.x << ", " << position.y << ")\n";
}

bool same_position(const Position first, const Position second) {
    return (first.x == second.x) && (first.y == second.y);
}

bool is_blocked(const Position position, const std::vector<Position>& obstacles) {
    for (const Position obstacle : obstacles) {
        if (same_position(position, obstacle)) {
            return true;
        }
    }

    return false;
}

// The caller must validate dimensions, coordinates and unblocked endpoints.
PlanningResult plan_path_bfs(const Mission& mission) {
    const Position undiscovered{-1, -1};
    std::vector<std::vector<Position>> parents(
        mission.height,
        std::vector<Position>(mission.width, undiscovered)
    );
    std::queue<Position> frontier;
    PlanningResult result;

    parents[mission.start.y][mission.start.x] = mission.start;
    frontier.push(mission.start);

    const Position directions[] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    while (!frontier.empty()) {
        const Position current = frontier.front();
        frontier.pop();
        // Count cells removed from the queue, including the goal if reached.
        ++result.explored_nodes;

        if (same_position(current, mission.target)) {
            break;
        }

        for (const Position direction : directions) {
            const Position next{current.x + direction.x, current.y + direction.y};
            if (!is_inside_grid(next, mission.width, mission.height)) {
                continue;
            }
            if (is_blocked(next, mission.obstacles)) {
                continue;
            }
            if (!same_position(parents[next.y][next.x], undiscovered)) {
                continue;
            }

            // Record discovery before enqueueing, so each cell is queued once.
            parents[next.y][next.x] = current;
            frontier.push(next);
        }
    }

    if (same_position(parents[mission.target.y][mission.target.x], undiscovered)) {
        return result;
    }

    // Follow predecessors from goal to start, then reverse for execution.
    Position current = mission.target;
    while (!same_position(current, mission.start)) {
        result.path.push_back(current);
        current = parents[current.y][current.x];
    }
    result.path.push_back(mission.start);
    std::reverse(result.path.begin(), result.path.end());

    return result;
}

void print_grid(
    const int width,
    const int height,
    const Position start,
    const Position current,
    const Position target,
    const std::vector<Position>& obstacles
) {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const Position cell{x, y};
            char symbol = '.';

            // Show the rover first when multiple positions share a cell.
            if (same_position(cell, current)) {
                symbol = 'R';
            } else if (same_position(cell, target)) {
                symbol = 'G';
            } else if (same_position(cell, start)) {
                symbol = 'S';
            } else if (is_blocked(cell, obstacles)) {
                symbol = 'X';
            }

            std::cout << symbol << ' ';
        }

        std::cout << '\n';
    }
}

int main(const int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <mission.json>\n";
        return 2;
    }

    Mission mission{};
    try {
        mission = load_mission(argv[1]);
    } catch (const std::exception& error) {
        std::cerr << "Mission error: " << error.what() << '\n';
        return 1;
    }

    const int width = mission.width;
    const int height = mission.height;
    const Position start = mission.start;
    const Position target = mission.target;
    const std::vector<Position>& obstacles = mission.obstacles;
    Position rover = start;

    if ((width <= 0) || (height <= 0)) {
        std::cerr << "Mission error: grid dimensions must be positive.\n";
        return 1;
    }

    if ((!is_inside_grid(rover, width, height))
        || (!is_inside_grid(target, width, height))) {
        std::cerr << "Mission error: start and target must be inside the grid.\n";
        return 1;
    }

    for (const Position obstacle : obstacles) {
        if (!is_inside_grid(obstacle, width, height)) {
            std::cerr << "Mission error: obstacles must be inside the grid.\n";
            return 1;
        }
    }

    if ((is_blocked(start, obstacles)) || (is_blocked(target, obstacles))) {
        std::cerr << "Mission error: start and target must not be blocked.\n";
        return 1;
    }

    std::cout << "R = Rover, S = Start, G = Goal, X = Obstacle, . = Terrain\n";
    std::cout << "Initial grid:\n";
    print_grid(width, height, start, rover, target, obstacles);
    print_position(rover);

    const PlanningResult plan = plan_path_bfs(mission);
    std::cout << "Planner: BFS\n";
    std::cout << "Explored nodes: " << plan.explored_nodes << '\n';
    if (plan.path.empty()) {
        std::cerr << "Mission failed: no path to target.\n";
        return 1;
    }
    std::cout << "Path length: " << plan.path.size() - 1 << '\n';

    long long steps = 0;
    // path[0] is the start; each following element is one cardinal move.
    for (std::size_t index = 1; index < plan.path.size(); ++index) {
        rover = plan.path[index];
        ++steps;
        print_position(rover);
    }

    std::cout << "Mission complete. Steps: " << steps << '\n';
    std::cout << "Final grid:\n";
    print_grid(width, height, start, rover, target, obstacles);

    return 0;
}
