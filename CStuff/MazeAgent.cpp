/* Reinforcement Learning Agent using A* and Random searches
   to solve maze puzzles and provide data for the Nueral Network
*/
#pragma once
#include "MazeAgent.hpp"
#include <iostream>
#include <vector>
#include <sstream>
#include <tuple>
#include <queue>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cctype>

using namespace std;


/*
    Maze Search Problem Class
    This class defines the search problem for the maze.
    It includes methods to check if a state is a goal state,
    get the start state, check if a cell is traversable,
    and get successors of a given state.
*/

// Maze Grid state
//std::vector<std::vector<int>> map;
//std::pair<int, int> state, goal;


// Constructor: initialize with maze dimensions and optionally the start and goal
MazeSearchProblem::MazeSearchProblem(const std::vector<std::vector<int>>& mazeMap, const std::pair<int, int>& startState,
    const std::pair<int, int>& goalState)
    : map(mazeMap), state(startState), goal(goalState) {
}

bool MazeSearchProblem::isGoalState(const std::pair<int, int>& state) const {
    return state == goal;
}

std::pair<int, int> MazeSearchProblem::getStartState() const {
    return state;
}

// Returns true if a cell is traversable (open, start, or exit)
bool MazeSearchProblem::isTraversable(int cellVal) const {
    return cellVal != 1;
}

std::vector<std::tuple<std::pair<int, int>, std::string, int>> MazeSearchProblem::getSuccessors(const std::pair<int, int>& CurrentState) const {

    // Vectors for 4-directional movement (N, E, S, W)
    int dx[4] = { 0, 1, 0, -1 };
    int dy[4] = { -1, 0, 1, 0 };

    std::string actions[4] = { "N", "E", "S", "W" };

    //initialize the vector of successors
    std::vector<std::tuple<std::pair<int, int>, std::string, int>> successors;

    // For each action, determine the resulting state and cost
    for (int dir = 0; dir < 4; dir++) {

        // get coordinates of the new state
        int nx = CurrentState.first + dx[dir];
        int ny = CurrentState.second + dy[dir];

        // Check bounds and that the cell is "open".
        if (nx >= 0 && nx < map[0].size() && ny >= 0 && ny < map.size()) {
            if (isTraversable(map[ny][nx])) {
                // Valid state, add to successors
                successors.push_back(std::make_tuple(std::make_pair(nx, ny), actions[dir], 1));
            }
        }
    }

    return successors;
}

// Cost of taking an action
int MazeSearchProblem::getCostofActions(const std::vector<std::string>& actions) const {
    return actions.size();
}

//Free Fucntions 

/*
    Manhattan Distance - Computes the Manhattan distance between two points
*/
int manhattanDistance(std::pair<int, int> a, std::pair<int, int> b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

/*
    A* Search Algorithm - Finds the shortest path in a maze using A* search
*/
std::vector<std::string> aStarSearch(MazeSearchProblem& problem) {
    // Define a priority queue for the frontier
    typedef std::pair<int, Node> PQElement; // (priority, Node)

    // Compare elements to reverse C++ priority queue natural order 
    auto cmp = [](const PQElement& a, const PQElement& b) { return a.first > b.first; };
    // Create the priority queue
    std::priority_queue<PQElement, std::vector<PQElement>, decltype(cmp)> frontier(cmp);

    //Get starting state
    std::pair<int, int> startState = problem.getStartState();
    Node startNode = { startState, {}, 0 };

    // Calculate the heuristic value for the starting state
    int h = manhattanDistance(startState, problem.goal);
    frontier.push({ h, startNode });

    //Create the visited vector
    std::vector<std::vector<bool>> visited(problem.map.size(), std::vector<bool>(problem.map[0].size(), false));
    visited[startState.second / 12][startState.first / 12] = true;

    while (!frontier.empty()) {

        // Get the current node
        PQElement currentPair = frontier.top();
        frontier.pop();
        Node current = currentPair.second;
        std::pair<int, int> state = current.state;

        if (problem.isGoalState(state)) {
            // Found the exit, return the actions
            return current.actions;
        }

        // Get successors according to A* Search (Thank you, EEEE 647)
        std::vector<std::tuple<std::pair<int, int>, std::string, int>> successors = problem.getSuccessors(state);
        for (auto& succ : successors) {
            // Get the next state, action, and cost
            std::pair<int, int> nextState = std::get<0>(succ);
            std::string action = std::get<1>(succ);
            int stepCost = std::get<2>(succ);

            // If the next state has not been visited, add it to the frontier
            if (!visited[nextState.second][nextState.first]) {
                visited[nextState.second][nextState.first] = true;

                // Create the next node
                Node nextNode;
                nextNode.state = nextState;
                nextNode.actions = current.actions;
                nextNode.actions.push_back(action);
                nextNode.cost = current.cost + stepCost;
                int priority = nextNode.cost + manhattanDistance(nextState, problem.goal);
                frontier.push({ priority, nextNode });
            }
        }
    }
    return {};  // no solution found
}

/*
    Random Search Algorithm - Finds a path in a maze using random search
*/
std::vector<std::pair<int, int>> randomSearch(MazeSearchProblem& problem) {
    std::vector<std::pair<int, int>> path;
    std::pair<int, int> currentState = problem.getStartState();
    path.push_back(currentState);

    // possible moves
    int dx[4] = { 0, 1, 0, -1 };
    int dy[4] = { -1, 0, 1, 0 };

    // while the current state is not the goal state
    while (!problem.isGoalState(currentState)) {
        std::vector<std::pair<int, int>> possibilities;
        for (int i = 0; i < 4; i++) {
            int nx = currentState.first + dx[i];
            int ny = currentState.second + dy[i];

            //Build a list of possible moves to randomly choose from
            if (nx >= 0 && nx < problem.map[0].size() && ny >= 0 && ny < problem.map.size()) {
                if (problem.isTraversable(problem.map[ny][nx])) {
                    possibilities.push_back({ nx, ny });
                }
            }
        }

        if (!possibilities.empty()) {
            currentState = possibilities[rand() % possibilities.size()];
            path.push_back(currentState);
        }
        else {
            break;
        }
    }
    return path;
}


/*
    Get Local Window - Returns a local window of the maze around the current state
    The local window is a square of size (2*radius + 1) x (2*radius + 1)
*/
std::vector<std::vector<int>> getLocalWindow(const std::vector<std::vector<int>>& mazeData, const std::pair<int, int>& currentState, int radius) {
    // Local window size : (2*radius + 1) x (2*radius + 1)
    std::vector<std::vector<int>> localWindow;
    int gridX = currentState.first, gridY = currentState.second;
    for (int dy = -radius; dy <= radius; dy++) {
        std::vector<int> row;
        for (int dx = -radius; dx <= radius; dx++) {
            // Calculate the coordinates of the cell in the maze
            int nx = gridX + dx, ny = gridY + dy;
            if (nx < 0 || nx >= mazeData[0].size() || ny < 0 || ny >= mazeData.size()) {
                row.push_back(1); // out-of-bounds: treat as wall
            }
            else {
                row.push_back(mazeData[ny][nx]); // Get the value of the cell in the maze
            }
        }
        localWindow.push_back(row); // populate the window
    }
    return localWindow;
}


/*
    Maze Environment Class
    This class defines the environment for the maze agent.
    It includes methods to reset the agent's position, take steps in the maze,
    and check for collisions with walls or boundaries.
*/

// Constructor to initialize the maze environment
MazeEnvironment::MazeEnvironment(const std::vector<std::vector<int>>& mazeData, const std::pair<int, int>& startPos, const std::pair<int, int>& exitPos)
    : mazeData(mazeData), startPos(startPos), exitPos(exitPos) {
    currentPos = startPos; // Initialize current position to start position
}

// Function to reset the agent's position to the start
std::vector<std::vector<int>> MazeEnvironment::reset() {
    currentPos = startPos; // Reset current position to start position
    // Return the local window around the current position
    return getLocalWindow(mazeData, currentPos, WINDOW_SIZE); // 1-cell radius for local window
}

std::pair<int, int> MazeEnvironment::actionToDelta(const std::string& action) {
    if (action == "N") return { 0, -1 };
    if (action == "E") return { 1, 0 };
    if (action == "S") return { 0, 1 };
    if (action == "W") return { -1, 0 };
    return { 0, 0 }; // default: no movement
}

// Function to take a step in the environment based on an action
std::tuple<std::vector<std::vector<int>>, double, bool> MazeEnvironment::takeStep(const std::string& action) {
    // Convert current state from grid to coordiante
    std::pair<int, int> delta = actionToDelta(action); // Convert action to delta (change in position)
    std::pair<int, int> nextState = { currentPos.first + delta.first, currentPos.second + delta.second }; // Calculate next state

    // Check collision/bound
    if (nextState.first < 0 || nextState.first >= mazeData[0].size() ||
        nextState.second < 0 || nextState.second >= mazeData.size() ||
        mazeData[nextState.second][nextState.first] == 1) {
        // Hit a wall or out of bounds, return current state, negative reward, and not done
        return { getLocalWindow(mazeData, currentPos, WINDOW_SIZE), -1.0, false };
    }

    // Valid move, update current position
    currentPos = nextState;

    // Are we done?
    bool done = (currentPos == exitPos); // Check if the agent has reached the exit
    double prize = done ? 10.0 : -0.1; // Reward for reaching the exit, small penalty for each step taken

    return { getLocalWindow(mazeData, currentPos, WINDOW_SIZE), prize, done }; // Return the new state, reward, and done flag
}

// QNetwork local_net, target_net;
// torch::optim::Adam optimizer;
// ReplayBuffer replay;
// float gamma;
// int update_every, t_step = 0;


/*
    QNetwork Class
    This class implements a simple feedforward neural network with three fully connected layers.
    It is used to approximate the Q-values for the actions in the maze environment.
*/
QNetworkImpl::QNetworkImpl(int64_t state_size, int64_t action_size) {
    fc1 = register_module("fc1", torch::nn::Linear(state_size, 64));
    fc2 = register_module("fc2", torch::nn::Linear(64, 64));
    fc3 = register_module("fc3", torch::nn::Linear(64, action_size));

    // Verify initialization
    if (!fc1 || !fc2 || !fc3) {
        throw std::runtime_error("Failed to initialize network layers");
    }
}

// Forward pass through the network
torch::Tensor QNetworkImpl::forward(torch::Tensor x) {
    x = torch::relu(fc1(x));
    x = torch::relu(fc2(x));
    x = fc3(x);
    return x;
}

// Return the number of output features
int QNetworkImpl::out_features() const {
    return fc3->options.out_features();
}


/*
    DQNAgent Class
    This class implements the Deep Q-Network (DQN) agent.
    It includes methods for action selection, experience replay, and learning.
*/

static constexpr float TAU = 1e-3f;  //Soft update
// Constructor
DQNAgent::DQNAgent(int state_size, int action_size, float lr, float gamma, size_t buffer_size, size_t batch_size, int update_every)
    :   local_net(state_size, action_size),
        target_net(state_size, action_size),
        optimizer(local_net->parameters(), lr),
    //local_net(std::make_shared<QNetworkImpl>(state_size, action_size)),
    //target_net(std::make_shared<QNetworkImpl>(state_size, action_size)),
    //optimizer(local_net->parameters(), lr),
    replay(buffer_size, batch_size),
    gamma(gamma),
    update_every(update_every),
    t_step(0)
{
    // Copy weights to target
        // Verify network creation
    if (!local_net || !target_net) {
        throw std::runtime_error("Network creation failed");
    }

    // Test forward pass
    try {
        auto test_input = torch::ones({ 1, state_size });
        auto test_output = local_net->forward(test_input);
        std::cout << "Network test passed. Output shape: "
            << test_output.sizes() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Network test failed: " << e.what() << std::endl;
    }
    // Serialize the local network to an output archive
    torch::serialize::OutputArchive out_ar;
    local_net->save(out_ar);

    //Dump archive to a string stream
    std::ostringstream oss;
    out_ar.save_to(oss);

    //Load back into Input archive
    torch::serialize::InputArchive ar;
    ar.load_from(oss.str());

    // Load the serialized data into the target network
    target_net->load(ar);
}

std::string DQNAgent::getAction(int action) {
    switch (action) {
    case NORTH: return "N";
    case EAST:  return "E";
    case SOUTH: return "S";
    case WEST:  return "W";
    default:    return "N"; // Default action
    }
}

State DQNAgent::flattenObs(const std::vector<std::vector<int>>& obs) {
    // Flatten the 2D observation into a 1D vector
    State flat_obs;
    for (const auto& row : obs) {
        for (int cell : row) {
            flat_obs.push_back(float(cell)); // Convert to float
        }
    }
    return flat_obs;
}

// e greedy sekection
int DQNAgent::act(const State& state, float eps) {
    if ((float)std::rand() / RAND_MAX < eps)
        return std::rand() % local_net->out_features();

    // forward pass
    torch::NoGradGuard no_grad;
    auto x = torch::from_blob((float*)state.data(), { 1,(int)state.size() }).to(torch::kCUDA);
    auto q = local_net->forward(x).cpu();
    auto max_idx = std::get<1>(q.max(1));
    return max_idx.item<int>();
}

void DQNAgent::step(const State& s, int a, float r, const State& s2, bool done) {
    replay.add({ s,a,r,s2,done });
    t_step = (t_step + 1) % update_every;
    if (t_step == 0 && replay.can_sample()) learn();
}

void DQNAgent::learn() {
    /*
    auto batch = replay.sample(); // sample from replay buffer
    if (batch.empty()) return; // no samples to learn from (Error checking)

    // build tensors
    std::vector<float> vs, vs2, rs, ds;
    std::vector<int64_t> as;

    //Iterate through baatches 
    for (auto& [s, a, r, s2, d] : batch) {
        vs.insert(vs.end(), s.begin(), s.end());      // flatten state
        as.push_back(a);                              // actions
        rs.push_back(r);                              // rewards   
        vs2.insert(vs2.end(), s2.begin(), s2.end());  // flatten next state
        ds.push_back(d ? 1.f : 0.f);                      // done flag
    }

    int batch_sz = batch.size();
    int st_sz = vs.size() / batch_sz;


    
    // Create tensors from the vectors
    auto states = torch::from_blob(vs.data(), { batch_sz,st_sz }).to(torch::kCUDA); //from_blob is cheap and funny name
    auto actions = torch::tensor(as, torch::dtype(torch::kInt64)).unsqueeze(1).to(torch::kCUDA);
    auto rewards = torch::from_blob(rs.data(), { batch_sz,1 }).to(torch::kCUDA);
    auto next_states = torch::from_blob(vs2.data(), { batch_sz,st_sz }).to(torch::kCUDA);
    auto dones = torch::from_blob(ds.data(), { batch_sz,1 }).to(torch::kCUDA);

    // Q_expected, picks the taken action
    auto q_expected = local_net->forward(states).gather(1, actions);

    // Q_targets
    auto tup = target_net->forward(next_states).max(1);          // get max Q value
    auto q_next = std::get<0>(tup).unsqueeze(1).detach();       // detach from graph
    auto q_targets = rewards + gamma * q_next * (1 - dones);    // Bellman equation

    // Compute loss and backpropagate
    auto loss = torch::mse_loss(q_expected, q_targets);
    optimizer.zero_grad();
    loss.backward();
    optimizer.step();

    // soft update
    for (auto& kv : local_net->named_parameters()) {
        auto& name = kv.key();
        auto& local_p = kv.value();
        auto& target_p = target_net->named_parameters()[name];
        target_p.mul_(1 - TAU).add_(local_p, TAU);
    }
    */


    if (!replay.can_sample()) return;

    auto batch = replay.sample();
    if (batch.empty()) return;

    // Convert batch to tensors - using vectors that will stay in scope
    std::vector<float> states_vec, next_states_vec;
    std::vector<int64_t> actions_vec;
    std::vector<float> rewards_vec, dones_vec;

    for (const auto& [state, action, reward, next_state, done] : batch) {
        states_vec.insert(states_vec.end(), state.begin(), state.end());
        actions_vec.push_back(action);
        rewards_vec.push_back(reward);
        next_states_vec.insert(next_states_vec.end(), next_state.begin(), next_state.end());
        dones_vec.push_back(done ? 1.0f : 0.0f);
    }

    const int batch_size = batch.size();
    const int state_size = states_vec.size() / batch_size;

    // Create tensors - using .clone() to ensure ownership
    auto states = torch::from_blob(states_vec.data(), { batch_size, state_size }).clone().to(torch::kCUDA);
    auto actions = torch::tensor(actions_vec, torch::kInt64).unsqueeze(1).to(torch::kCUDA);
    auto rewards = torch::tensor(rewards_vec, torch::kFloat32).unsqueeze(1).to(torch::kCUDA);
    auto next_states = torch::from_blob(next_states_vec.data(), { batch_size, state_size }).clone().to(torch::kCUDA);
    auto dones = torch::tensor(dones_vec, torch::kFloat32).unsqueeze(1).to(torch::kCUDA);

    // Compute Q values
    auto q_values = local_net->forward(states).gather(1, actions);

    // Compute target Q values
     // Compute target Q values - corrected max() handling
    torch::Tensor next_q_values;
    {
        torch::NoGradGuard no_grad;
        auto max_result = target_net->forward(next_states).max(1);
        next_q_values = std::get<0>(max_result).unsqueeze(1);  // Get the values from max tuple
    }

    auto target_q_values = rewards + gamma * next_q_values * (1 - dones);

    // Compute loss
    auto loss = torch::mse_loss(q_values, target_q_values);

    // Backpropagation
    optimizer.zero_grad();
    loss.backward();
    torch::nn::utils::clip_grad_norm_(local_net->parameters(), 1.0);  // Gradient clipping
    optimizer.step();

    // Soft update target network
    for (const auto& param : local_net->named_parameters()) {
        target_net->named_parameters()[param.key()].data().mul_(1.0f - TAU).add_(param.value().data(), TAU);
    }

}

/*
    ReplayBuffer Class
    This class implements a replay buffer for storing experiences.
    It includes methods for adding experiences, sampling a batch,
    and checking if the buffer can be sampled.
*/
// Experience tuple: (state, action, reward, next_state, done)
using Experience = std::tuple<State, int, float, State, bool>;
std::deque<Experience> replay_buffer;
size_t max_size;
size_t batch_size;
std::mt19937 rng{ std::random_device{}() }; // Random number generator  

ReplayBuffer::ReplayBuffer(std::size_t max_size, std::size_t batch_size) : max_size(max_size), batch_size(batch_size) {}

void ReplayBuffer::add(const Experience& exp) {
    if (replay_buffer.size() == max_size) {
        replay_buffer.pop_front(); // Remove the oldest experience
    }
    replay_buffer.push_back(exp); // Add the new experience
}

// Helper function to check if the buffer is full
bool ReplayBuffer::can_sample() const {
    return replay_buffer.size() >= batch_size;
}

// Sample a batch of experiences from the buffer
std::vector<Experience> ReplayBuffer::sample() {
    std::uniform_int_distribution<size_t> dist(0, replay_buffer.size() - 1);
    std::vector<Experience> batch;

    for (size_t i = 0; i < batch_size; ++i) {
        batch.push_back(replay_buffer[dist(rng)]); // Sample a random experience
    }
    return batch;
}


//--------------------- Main Test Program ------------------------------
//int main() {
    // srand((unsigned int)time(nullptr)); // Seed random number generator

    // // Hardcoded maze text (11x11)
    // string mazeText = R"(  11111111111
    //                        10001000021
    //                        10101011111
    //                        10101000001
    //                        10111110111
    //                        10001010001
    //                        10101011101
    //                        10101000101
    //                        10111010101
    //                        13000010001
    //                        11111111111)";

    // // Parse the maze text into a 2D vector of ints.
    // vector<vector<int>> mazeMap;
    // istringstream iss(mazeText);
    // string line;
    // while(getline(iss, line)) {
    //     vector<int> row;
    //     for (char c : line) {
    //         if (isdigit(c)) {
    //             row.push_back(c - '0');
    //         }
    //     }
    //     if (!row.empty())
    //         mazeMap.push_back(row);
    // }

    // // Scan the maze for the start and goal.
    // // We assume '2' is the start and '3' is the goal.
    // pair<int,int> startState, goalState;
    // for (int i = 0; i < mazeMap.size(); i++) {
    //     for (int j = 0; j < mazeMap[i].size(); j++) {
    //         if (mazeMap[i][j] == 2) {
    //             startState = {j, i};  // (x, y)
    //         } else if (mazeMap[i][j] == 3) {
    //             goalState = {j, i};
    //         }
    //     }
    // }

    // // Create the search problem
    // MazeSearchProblem problem(mazeMap, startState, goalState);

    // // Run A* search and print the resulting action sequence.
    // vector<string> astarActions = aStarSearch(problem);
    // cout << "A* action path:" << endl;
    // for (const auto& act : astarActions) {
    //     cout << act << " ";
    // }
    // cout << endl;

    // // Run random search and print the path (as grid coordinates)
    // vector<pair<int,int>> randomPath = randomSearch(problem);
    // cout << "Random search path:" << endl;
    // for (const auto& coord : randomPath) {
    //     cout << "(" << coord.first << "," << coord.second << ") ";
    // }
    // cout << endl;

//    return 0;
//}

