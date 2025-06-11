#include <iostream>
#include <vector>
#include <tuple>
#include <utility>
#include <string>
#include <cstdlib>

// Define WINDOW_SIZE (the radius of the local window)
#ifndef WINDOW_SIZE
#define WINDOW_SIZE 1
#endif

// Helper function: Extracts a local window of cell values from the maze grid.
// Returns a 2D vector of ints representing the subgrid.
std::vector<std::vector<int>> getLocalWindow(const std::vector<std::vector<int>>& mazeData,
                                               const std::pair<int,int>& currentState,
                                               int windowSize) {
    std::vector<std::vector<int>> localWindow;
    int gridX = currentState.first, gridY = currentState.second;
    for (int dy = -windowSize; dy <= windowSize; dy++) {
        std::vector<int> row;
        for (int dx = -windowSize; dx <= windowSize; dx++) {
            int nx = gridX + dx;
            int ny = gridY + dy;
            if (nx < 0 || nx >= mazeData[0].size() || ny < 0 || ny >= mazeData.size())
                row.push_back(1);  // treat out-of-bounds as wall
            else
                row.push_back(mazeData[ny][nx]);
        }
        localWindow.push_back(row);
    }
    return localWindow;
}

// Helper function: Converts an action string to a delta in grid coordinates.
std::pair<int,int> actionToDelta(const std::string &action) {
    if (action == "N") return {0, -1};
    if (action == "E") return {1, 0};
    if (action == "S") return {0, 1};
    if (action == "W") return {-1, 0};
    return {0, 0}; // default: no movement
}

// MazeEnvironment definition (using grid coordinates)
class MazeEnvironment {
public:
    std::vector<std::vector<int>> mazeData; // Maze grid: 0 = free, 1 = wall
    std::pair<int, int> startPos; // Start cell (grid coordinates)
    std::pair<int, int> exitPos;  // Exit cell (grid coordinates)
    std::pair<int, int> currentPos; // Current cell (grid coordinates)

    MazeEnvironment(const std::vector<std::vector<int>>& mazeData,
                    const std::pair<int, int>& startPos,
                    const std::pair<int, int>& exitPos)
        : mazeData(mazeData), startPos(startPos), exitPos(exitPos) {
        currentPos = startPos;
    }

    // Resets the environment and returns the local observation around the agent.
    std::vector<std::vector<int>> reset() {
        currentPos = startPos;
        return getLocalWindow(mazeData, currentPos, WINDOW_SIZE);
    }

    // Takes a step based on the action, returns (observation, reward, done)
    std::tuple<std::vector<std::vector<int>>, double, bool> takeStep(const std::string& action) {
        std::pair<int, int> delta = actionToDelta(action);
        std::pair<int, int> nextState = { currentPos.first + delta.first, currentPos.second + delta.second };

        // Check for out-of-bounds or wall collisions.
        if (nextState.first < 0 || nextState.first >= mazeData[0].size() ||
            nextState.second < 0 || nextState.second >= mazeData.size() ||
            mazeData[nextState.second][nextState.first] == 1) {
            // Illegal move: return current observation, a negative reward, not done.
            return { getLocalWindow(mazeData, currentPos, WINDOW_SIZE), -1.0, false };
        }

        // Update agent's position.
        currentPos = nextState;
        bool done = (currentPos == exitPos);
        double reward = done ? 10.0 : -0.1; // Small step penalty; reward for exit.
        return { getLocalWindow(mazeData, currentPos, WINDOW_SIZE), reward, done };
    }
};

int main() {
    // Define a simple 5x5 maze:
    // 1 represents wall, 0 free space.
    // Maze layout (grid coordinates):
    // 1 1 1 1 1
    // 1 0 0 0 1
    // 1 0 1 0 1
    // 1 0 0 0 1
    // 1 1 1 1 1
    std::vector<std::vector<int>> maze = {
        {1,1,1,1,1},
        {1,0,0,0,1},
        {1,0,1,0,1},
        {1,0,0,0,1},
        {1,1,1,1,1}
    };

    // Set start at (1,1) and exit at (3,3) (grid coordinates).
    std::pair<int,int> start = {1, 1};
    std::pair<int,int> exit = {3, 3};

    MazeEnvironment env(maze, start, exit);

    // Reset the environment and print the initial local observation.
    auto initialObs = env.reset();
    std::cout << "Initial Observation:\n";
    for (const auto& row : initialObs) {
        for (int cell : row) {
            std::cout << cell << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    // Define a test sequence of actions.
    // For example, try moving East, East, South, South.
    std::vector<std::string> actions = {"E", "E", "S", "S"};

    // Simulate the agent taking actions.
    for (const auto& action : actions) {
        auto [obs, reward, done] = env.takeStep(action);
        std::cout << "Action: " << action 
                  << ", Reward: " << reward 
                  << ", Done: " << std::boolalpha << done << "\n";
        std::cout << "Observation:\n";
        for (const auto& row : obs) {
            for (int cell : row) {
                std::cout << cell << " ";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
        if (done) break;
    }

    return 0;
}
