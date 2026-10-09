#include "third_party/nlohmann/json.hpp"

#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

struct Position {
    int x;
    int y;
};

struct Mission {
    int width;
    int height;
    Position start;
    Position target;
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
        }
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

// Returns true after moving one cell, or false if already at the target.
// The caller must validate both positions against the same obstacle-free grid.
bool step_toward(Position& current, const Position target) {
    if (current.x < target.x) {
        ++current.x;
    } else if (current.x > target.x) {
        --current.x;
    } else if (current.y < target.y) {
        ++current.y;
    } else if (current.y > target.y) {
        --current.y;
    } else {
        return false;
    }

    return true;
}

void print_position(const Position position) {
    std::cout << "Rover position: (" << position.x << ", " << position.y << ")\n";
}

bool same_position(const Position first, const Position second) {
    return (first.x == second.x) && (first.y == second.y);
}

void print_grid(
    const int width,
    const int height,
    const Position start,
    const Position current,
    const Position target
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

    std::cout << "R = Rover, S = Start, G = Goal, . = Terrain\n";
    std::cout << "Initial grid:\n";
    print_grid(width, height, start, rover, target);
    print_position(rover);

    long long steps = 0;
    while (step_toward(rover, target)) {
        ++steps;
        print_position(rover);
    }

    std::cout << "Mission complete. Steps: " << steps << '\n';
    std::cout << "Final grid:\n";
    print_grid(width, height, start, rover, target);

    return 0;
}
