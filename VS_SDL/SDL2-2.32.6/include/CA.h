#pragma once
#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <queue>
#include <utility>

//--------------------------------------------------
// Cell Class
//--------------------------------------------------
class Cell {
public:
    // Cell state as described in the document:
    // 0 = Disconnected
    // 1 = Seed
    // 2 = Invite
    // 3 = Connected
    enum CellState { Disconnected = 0, Seed = 1, Invite = 2, Connected = 3 };

    // Bitfield for neighbor directions using a 4-cell (N, E, S, W) neighborhood
    // We define the bits as follows:
    // Bit 0: North, Bit 1: East, Bit 2: South, Bit 3: West
    enum CellDirections { N = 1, E = 2, S = 4, W = 8 };

    // Properties
    uint8_t neighborhood = 0;         // Bitfield: start with no neighbors
    CellState state = Disconnected;   // Initially, every cell is disconnected
    uint8_t parentDirection = 0;      // Parent's direction (where did we come from)

    // Neighborhood Bitfield Manipulation Functions
    inline void addNeighbor(CellDirections dir) {
        neighborhood |= dir;
    }

    inline void removeNeighbor(CellDirections dir) {
        neighborhood &= ~dir;
    }

    inline bool hasNeighbor(CellDirections dir) const {
        return (neighborhood & dir) != 0;
    }

    // State Transition Functions

    // When a disconnected cell receives an invitation, it becomes a seed.
    inline void becomeSeed(uint8_t inviteDirection) {
        parentDirection = inviteDirection;
        state = Seed;
    }

    // A seed cell sends an invitation to one of its disconnected neighbors.
    inline void sendInvite() {
        state = Invite;
    }

    int getStateValue() const { 
        return static_cast<int>(state); 
    }


    // Based on a branching probability, finish the invitation:
    // if random > branchProb: Connected
    // else revert to Seed
    inline void finishInvite(uint16_t branchProbability) {
        if ((rand() % 101) > branchProbability) {
            state = Connected;
        } else {
            state = Seed;
        }
    }
    
    // Print the neighborhood bitfield as directions for debugging
    inline void printNeighborhood() const {
        std::cout << "Neighborhood: ";
        if (hasNeighbor(N)) std::cout << "N ";
        if (hasNeighbor(E)) std::cout << "E ";
        if (hasNeighbor(S)) std::cout << "S ";
        if (hasNeighbor(W)) std::cout << "W ";
        std::cout << std::endl;
    }
};

// CA (Cellular Automata) Helper Class
class CA {
public:
    //Directions: 0 = North, 1 = East, 2 = South, 3 = West

    // Turning can be implemented with shifts and ors (for FPGA)
    static uint8_t turnRight(uint8_t direction) {
        return (direction + 1) % 4;
    }
    
    static uint8_t turnLeft(uint8_t direction) {
        return (direction + 3) % 4; // Equivalent to (direction - 1) mod 4
    }
    
    static uint8_t turnAround(uint8_t direction) {
        return (direction + 2) % 4;
    }
};


class MazeMap{
    public:
        // Maze grid
        MazeMap(int width, int height);

        void printMap() const;
        void generateMaze(uint16_t branchProbability, uint16_t turnProbability);
        void renderMaze(const MazeMap &maze);
        std::pair<int, int> findFurthestCell(int startX, int startY);

        std::vector<std::vector<Cell>> map;
        int width;
        int height;
        int startX;
        int startY;
};

void runGeneration(MazeMap maze, uint16_t branchProb = 5, uint16_t turnProb = 10);