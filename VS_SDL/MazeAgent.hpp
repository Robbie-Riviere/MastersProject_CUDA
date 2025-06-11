#pragma once
#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <queue>
#include <utility>
#include <tuple>
#include <algorithm>
#include <set>
#include <cmath>
#include <limits>
#include <cctype>
#include <sstream>
#include <deque>
#include <random>
//#include 
#include "CA.h"
#include "torch/torch.h"

#define WINDOW_SIZE 1

// MazeSearchProblem Class
// This class defines the search problem for the maze

class MazeSearchProblem {
    public:
        // Constructor: initialize with maze dimensions and optionally the start and goal
        MazeSearchProblem(const std::vector<std::vector<int>>& mazeMap, const std::pair<int,int>& startState, const std::pair<int,int>& goalState);

        bool isGoalState(const std::pair<int, int>& state) const;

        std::pair<int, int> getStartState() const;

        // Returns true if a cell is traversable (open, start, or exit)
        bool isTraversable(int cellVal) const;

        std::vector<std::tuple<std::pair<int,int>, std::string, int>> getSuccessors(const std::pair<int, int>& CurrentState ) const;

        // Cost of taking an action
        int getCostofActions(const std::vector<std::string>& actions) const;

        // Maze Grid state
        std::vector<std::vector<int>> map;
        std::pair<int,int> state, goal;
};

struct Node
{
    std::pair<int, int> state;
    std::vector<std::string> actions;
    int cost;
};

struct CompareNode {
    bool operator()(const std::pair<int, Node>& a, const std::pair<int, Node>& b) const {
        return a.first > b.first; // smallest cost has highest priority
    }
};

// Helper Functions
int manhattanDistance(std::pair<int, int> a, std::pair<int, int> b);
std::vector<std::string> aStarSearch(MazeSearchProblem& problem);
std::vector<std::pair<int, int>> randomSearch(MazeSearchProblem& problem);
std::vector<std::vector<int>> getLocalWindow(const std::vector<std::vector<int>> &mazeData,const std::pair<int,int>& currentState, int radius = 1);

class MazeEnvironment{
    public:
    MazeEnvironment( const std::vector<std::vector<int>>& mazeData, const std::pair<int, int>& startPos, const std::pair<int, int>& exitPos);
    std::vector<std::vector<int>> reset();
    std::pair<int, int> actionToDelta(const std::string &action);
    std::tuple<std::vector<std::vector<int>>, double, bool> takeStep(const std::string& action);
    std::vector<std::vector<int>> mazeData; // The maze data (0 = free, 1 = wall)
    std::pair<int, int> startPos; // Starting position of the agent
    std::pair<int, int> exitPos; // Exit position of the agent
    std::pair<int, int> currentPos; // Current position of the agent
};

using State = std::vector<float>;  // e.g. {0,1,0,0, ...}
enum Action { NORTH=0, EAST=1, SOUTH=2, WEST=3 };
constexpr int ACTION_SIZE = 4;

struct ReplayBuffer {
    // Experience tuple: (state, action, reward, next_state, done)
    using Experience = std::tuple<State, int, float , State, bool>;
    public:
    ReplayBuffer(std::size_t max_size, std::size_t batch_size);

    void add(const Experience& exp);

    // Helper function to check if the buffer is full
    bool can_sample() const;

    // Sample a batch of experiences from the buffer
    std::vector<Experience> sample();

    private:
        std::deque<Experience> replay_buffer;
        std::size_t max_size;
        std::size_t batch_size;
        std::mt19937 rng{std::random_device{}()}; // Random number generator

};

// Defining a QNetwork structure to test The Self learning Agent through ReLU
//torch::Device device = torch::kCUDA; // Set the device to CUDA (GPU) if available

struct QNetworkImpl : torch::nn::Module {
public:
    // Constructor to initialize the layers
    QNetworkImpl(int64_t state_size, int64_t action_size);

    // Forward pass through the network
    torch::Tensor forward(torch::Tensor x);

    // Return the number of output features
    int out_features() const;

private:
    // Three layers of fully connected neural network
    torch::nn::Linear fc1{ nullptr }, fc2{ nullptr }, fc3{ nullptr };
};
TORCH_MODULE(QNetwork); // Register the module with TorchScript

struct DQNAgent {
    public:
    
    //Constructor
    DQNAgent(int state_size,
            int action_size,
            float   lr,
            float   gamma,
            size_t  buffer_size,
            size_t  batch_size,
            int     update_every);

    // Make an action based on the state and epsilon greedy
    int act(const State& state, float eps);

    // Get the action as a string
    std::string getAction(int action);

    // Store the experience in the replay buffer
    void step(const State& s, int a, float r, const State& s2, bool done);

    // Helper to flatten the observation to 1D
    State flattenObs(const std::vector<std::vector<int>>& obs);

    private:
    // Sample/update nets
    void learn();    

    // networks
    QNetwork  local_net, target_net;
    torch::optim::Adam optimizer;

    // Replay and RL hyperparams
    ReplayBuffer  replay;
    float         gamma;
    int           update_every, t_step; 
};