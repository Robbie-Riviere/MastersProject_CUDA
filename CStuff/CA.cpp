/*
* This file provides supporting functions to main.cpp
* It is responsible for defining the functions and rules
* for cellular automata for the game controller
*/

#include "CA.h"


// MazeMap Class
// Maze grid

// Constructor for a grid of cells (all start as Disconnected)
MazeMap::MazeMap(int width, int height) : width(width), height(height) {
    map = std::vector<std::vector<Cell>>(height, std::vector<Cell>(width));
    startX = rand() % width;  // Random starting X position
    startY = rand() % height; // Random starting Y position
}

// Print the state of each cell
void MazeMap::printMap() const {
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            std::cout << map[i][j].state << " ";
        }
        std::cout << std::endl;
    }
}

/* Generate the maze using cellular automata
    The branching and turning probabilities come from the documentation
    */
void MazeMap::generateMaze(uint16_t branchProbability = 5, uint16_t turnProbability = 10) {

    // Choose a random starting cell to be the seed

    map[startY][startX].state = Cell::Seed;
    map[startY][startX].parentDirection = 255;  // 255 means "no parent"

    bool mazeComplete = false;
    int generation = 0;

    // Define neighbor vectors: 0 = North, 1 = East, 2 = South, 3 = West
    int dx[4] = { 0, 1, 0, -1 };
    int dy[4] = { -1, 0, 1, 0 };

    while (!mazeComplete) {
        mazeComplete = true;
        bool foundSeed = false;

        // Iterate over each cell in the maze
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                Cell& cell = map[y][x];
                if (cell.state == Cell::Disconnected) {
                    mazeComplete = false;  // Still have disconnected cells
                }
                if (cell.state == Cell::Seed) {
                    foundSeed = true;

                    // Build a candidate list of neighbor directions that are Disconnected
                    std::vector<int> candidateDirections;
                    for (int d = 0; d < 4; d++) {
                        int nx = x + dx[d];
                        int ny = y + dy[d];
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            if (map[ny][nx].state == Cell::Disconnected) {
                                candidateDirections.push_back(d);
                            }
                        }
                    }

                    // If there are eligible neighbors, choose one at random
                    if (!candidateDirections.empty()) {
                        int chosenIndex = rand() % candidateDirections.size();
                        int chosenDir = candidateDirections[chosenIndex];
                        int nx = x + dx[chosenDir];
                        int ny = y + dy[chosenDir];

                        // The neighbor cell becomes a seed
                        // Its parent should be the current cell; so store the opposite of the chosen direction
                        uint8_t oppositeDir = (chosenDir + 2) % 4;
                        map[ny][nx].becomeSeed(oppositeDir);

                        // Mark the current cell as having sent an invitation
                        cell.sendInvite();

                        // Based on branch probability, decide whether this cell becomes Connected or remains a Seed to allow further branching
                        if ((rand() % 101) > branchProbability) { //Will be adjusted by nueral network
                            cell.state = Cell::Connected;
                        }
                    }
                    else {
                        // No eligible neighbor; this seed cell becomes Connected
                        cell.state = Cell::Connected;
                    }
                }
                else if (cell.state == Cell::Invite) {
                    // For cells in Invite state, finish the invitation process
                    cell.finishInvite(branchProbability);
                }
            }
        }

        // Fail-safe: if no cell is a seed and the maze is not complete, revive some Connected cells that border a Disconnected cell
        if (!foundSeed && !mazeComplete) {
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (map[y][x].state == Cell::Connected) {
                        bool hasDisconnectedNeighbor = false;
                        for (int d = 0; d < 4; d++) {
                            int nx = x + dx[d];
                            int ny = y + dy[d];
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                if (map[ny][nx].state == Cell::Disconnected) {
                                    hasDisconnectedNeighbor = true;
                                    break;
                                }
                            }
                        }
                        if (hasDisconnectedNeighbor && (rand() % 101 < branchProbability)) {
                            map[y][x].state = Cell::Seed;
                        }
                    }
                }
            }
        }

        generation++;
        // // print iteration info every few cycles
        // if (iteration % 50 == 0) {
        //     std::cout << "Iteration: " << iteration << std::endl;
        // }
    }

    std::cout << "Maze generation complete in " << generation << " iterations." << std::endl;
}

void MazeMap::renderMaze(const MazeMap& maze) {
    // Get maze dimensions and start position
    int mazeRows = maze.height;
    int mazeCols = maze.width;
    int startY = maze.startY;
    int startX = maze.startX;

    // The rendered maze will have extra rows and columns for walls
    int renderRows = mazeRows * 2 + 1;
    int renderCols = mazeCols * 2 + 1;

    // Create a render grid filled with 1's (walls)
    std::vector<std::vector<int>> renderGrid(renderRows, std::vector<int>(renderCols, 1));

    // For each cell in the CA maze:
    for (int i = 0; i < mazeRows; i++) {
        for (int j = 0; j < mazeCols; j++) {
            // Compute the cell's position in the render grid
            // Map cell (i, j) to render position (2*i+1, 2*j+1)
            int r = i * 2 + 1;
            int c = j * 2 + 1;
            renderGrid[r][c] = 0; // Mark the cell as floor

            // Remove the wall between this cell and its parent, if applicable
            // Get the parent's direction is stored as an index: 0 = North, 1 = East, 2 = South, 3 = West
            uint8_t pd = maze.map[i][j].parentDirection;
            if (pd < 4) { // Only remove a wall if the cell has a valid parent direction
                if (pd == 0) {
                    // Parent is to the North: remove the wall immediately above this cell
                    renderGrid[r - 1][c] = 0;
                }
                else if (pd == 1) {
                    // Parent is to the East: remove the wall immediately to the right
                    renderGrid[r][c + 1] = 0;
                }
                else if (pd == 2) {
                    // Parent is to the South: remove the wall immediately below
                    renderGrid[r + 1][c] = 0;
                }
                else if (pd == 3) {
                    // Parent is to the West: remove the wall immediately to the left
                    renderGrid[r][c - 1] = 0;
                }
            }
        }
    }

    // Mark start and exit cells using unique values:
    // Convert start and exit from maze coordinates to render grid coordinates.
    int startR = startY * 2 + 1;
    int startC = startX * 2 + 1;

    findFurthestCell(startX, startY);
    int exitX = findFurthestCell(startX, startY).first;
    int exitY = findFurthestCell(startX, startY).second;

    int exitR = exitY * 2 + 1;
    int exitC = exitX * 2 + 1;

    renderGrid[startR][startC] = 2;  // e.g. 2 means start.
    renderGrid[exitR][exitC] = 3;      // e.g. 3 means exit.

    // print the rendered maze
    // for (const auto &row : renderGrid) {
    //     for (int cell : row) {
    //         std::cout << cell << "";
    //     }
    //     std::cout << std::endl;
    // }


    /* Sends maze to file to be converted to bitmap*/
    std::ofstream out("C:/Users/Robbie/RIT/Masters/MastersProj/CStuff/MastersNew/GeneratedMazes/genmaze.txt");
    if (!out) {
        std::cerr << "Error: Could not open file for writing!" << std::endl;
        return;
    }
    for (const auto& row : renderGrid) {
        for (int cell : row) {
            out << cell << ""; // or however you want to represent the cell
        }
        out << "\n";
    }
    out.close();
}



std::pair<int, int> MazeMap::findFurthestCell(int startX, int startY) {
    // Create a 2D vector to store distances; initialize to -1 (unvisited)
    std::vector<std::vector<int>> dist(height, std::vector<int>(width, -1));

    // Use a queue for BFS
    std::queue<std::pair<int, int>> q;
    q.push({ startX, startY });
    dist[startY][startX] = 0;

    int maxDist = 0;
    std::pair<int, int> farthest = { startX, startY };

    // Vectors for 4-directional movement (N, E, S, W)
    int dx[4] = { 0, 1, 0, -1 };
    int dy[4] = { -1, 0, 1, 0 };

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d];
            int ny = y + dy[d];

            // Check bounds and that the cell is "open".
            // Here, we assume a cell is open if its state is Connected.
            if (nx >= 0 && nx < width && ny >= 0 && ny < height && map[ny][nx].state == Cell::Connected && dist[ny][nx] == -1) {
                dist[ny][nx] = dist[y][x] + 1;
                q.push({ nx, ny });

                if (dist[ny][nx] > maxDist) {
                    maxDist = dist[ny][nx];
                    farthest = { nx, ny };
                }
            }
        }
    }

    return farthest;
}

void runGeneration(MazeMap maze, uint16_t branchProb, uint16_t turnProb)
{
    /* Cellular Automata Maze Generator Test program*/
    srand(static_cast<unsigned int>(time(NULL)));
    // Initialize the maze map
    //MazeMap maze(20, 20);
    maze.generateMaze(branchProb, turnProb);
    std::pair<int, int> goal = maze.findFurthestCell(0, 0);
    std::cout << "Maze start: (" << 0 << ", " << 0 << ")\n";
    std::cout << "Maze exit: (" << goal.first << ", " << goal.second << ")\n";
    //maze.printMap();
    maze.renderMaze(maze);
}

//int main(int argc, char* argv[]) {
//    return 0;
//}




