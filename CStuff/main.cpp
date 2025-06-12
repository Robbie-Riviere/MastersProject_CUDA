#pragma once
#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include "CA.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <iomanip>
#include "MazeAgent.hpp"
#include <torch/torch.h>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif
#include <fstream>


using namespace std;

#define TILE_SIZE 12

const int SCREEN_WIDTH = 800, SCREEN_HEIGHT = 600;

enum ScreenState { MENU, CONTROLS, GAME, GAMEOVER, WIN, LOADING, AGENT };

// Declare SDL variables
SDL_Renderer* renderer;
SDL_Window* window;
TTF_Font* font;
SDL_Color color;
SDL_Event event;
ScreenState screenstate = MENU;
int text_width, text_height;

// Main loop flag
bool quit = false;

// Maze Cell vector
std::vector<std::vector<int>> mazeData;

//Mazemap object
MazeMap maze(20, 20);

std::pair<int, int> startPos;
std::pair<int, int> exitPos;

// Player statistics (used for difficulty scaling)
int wallsHit = 0;
time_t startTime;
time_t endTime;

// Maze Agent Globals
vector<string> solution;
int solutionIndex = 0;
uint32_t lastMoveTime = 0;
//MazeEnvironment mazeEnv; // Maze environment

pair<int, int> actionToDelta(const string& action) {
    if (action == "N") return { 0, -1 };
    if (action == "E") return { 1, 0 };
    if (action == "S") return { 0, 1 };
    if (action == "W") return { -1, 0 };
    return { 0, 0 }; // default: no movement
}



void cleanup(SDL_Window* window, SDL_Renderer* renderer, TTF_Font* font, const std::vector<SDL_Texture*>& textures) {
    /* Cleans up the SDL resources, protecting agaisnt memory leaks
    *  Parameters: SDL_Window* window, SDL_Renderer* renderer, TTF_Font* font, const std::vector<SDL_Texture*>& textures
    *  Returns: void
    */

    // free the textures stored in the vector
    for (auto texture : textures) {
        SDL_DestroyTexture(texture);
    }

    SDL_DestroyWindow(window);
    SDL_DestroyRenderer(renderer);
    TTF_CloseFont(font);
    TTF_Quit();
    SDL_Quit();
}

int initSDL() {
    /* Initializes the SDL2 library and creates a window and renderer
    *  for the game to be displayed in.
    *  Parameters: None
    *  Returns: 1 on error, 0 on success
    */
    // Initialize SDL2, returns zero on success else non-zero
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
        std::cout << "Error initializing SDL: " << SDL_GetError() << std::endl;
    }

    // Initialize SDL2_image, returns zero on success else non-zero
    if (TTF_Init() != 0) {
        std::cout << "Error initializing SDL_ttf: " << TTF_GetError() << std::endl;
    }

    // Create the window where we will draw, pass in the title and window size
    window = SDL_CreateWindow("Maze Game",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

    // Error Checking for the window
    if (window == NULL) {
        std::cout << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }


    // Create a renderer that will allow us to draw to the window
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // Error Checking for the renderer
    if (renderer == NULL) {
        std::cout << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    font = TTF_OpenFont("fonts\\arial.ttf", 24);

    if (font == NULL) {
        std::cout << "Font could not be created! SDL_Error: " << TTF_GetError() << std::endl;
        return 1;
    }

    color = { 255, 255, 255 };

    return 0;
}


SDL_Texture* loadTexture(const char* filePath) {
    SDL_Surface* surface = SDL_LoadBMP(filePath);
    if (!surface) {
        std::cerr << "Unable to load image " << filePath << "! SDL_Error: " << SDL_GetError() << std::endl;
        return nullptr;
    }

    std::cout << "Pixel Format is: " << SDL_GetPixelFormatName(surface->format->format) << std::endl;

    // From testing the python gen, we know that the maze is 8-bit indexed
    // So, Check if the surface is 8-bit indexed
    if (surface->format->format != SDL_PIXELFORMAT_INDEX8) {
        std::cerr << "Error: Expected 8-bit indexed BMP!" << std::endl;
        SDL_FreeSurface(surface);
        return nullptr;
    }

    // Get surface dimensions using SDL_Surface properties
    int width = surface->w;
    int height = surface->h;

    // Resize mazeData
    mazeData.resize(height, std::vector<int>(width, 0));
    //std::cout << "Maze Grid Size: " << mazeData[0].size() << " x " << mazeData.size() << std::endl;

    // Get the palette, the maze is differentiated by colors instead of values
    SDL_Palette* palette = surface->format->palette;
    if (!palette) {
        std::cerr << "Error: No palette found in BMP!" << std::endl;
        SDL_FreeSurface(surface);
        return nullptr;
    }

    Uint8* pixels = (Uint8*)surface->pixels;
    int pitch = surface->pitch;  // Bytes per row

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Uint8 pixelIndex = pixels[y * pitch + x];  // Get the palette index
            SDL_Color color = palette->colors[pixelIndex];  // Lookup actual RGB color

            // Load the Maze pixels based on the palette color
            // 0 = floor (white), 1 = wall (black), 2 = start (green), 3 = exit (blue)

            if (color.r == 255 && color.g == 255 && color.b == 255) {
                mazeData[y][x] = 0;  // Floor
            }
            else if (color.r == 0 && color.g == 0 && color.b == 0) {
                mazeData[y][x] = 1;  // Wall
            }
            else if (color.r == 0 && color.g == 255 && color.b == 0) {
                mazeData[y][x] = 2;  // Start
                startPos = { x * TILE_SIZE, y * TILE_SIZE };
            }
            else if (color.r == 0 && color.g == 0 && color.b == 255) {
                mazeData[y][x] = 3;  // Exit
                exitPos = { x * TILE_SIZE, y * TILE_SIZE };
            }
            //mazeData[y][x] = (color.r == 255 && color.g == 255 && color.b == 255) ? 0 : 1;
        }
    }

    // Print the maze for debugging
    // for (const auto& row : mazeData) {
    //     for (int cell : row) {
    //         std::cout << cell << "";
    //     }
    //     std::cout << std::endl;
    // }

    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_FreeSurface(surface);

    return texture;
}


void renderPlayer(int x, int y) {
    /* Renders the player on the screen
    *  Parameters: x and y coordinates
    *  Returns: void
    */

    SDL_Rect playerRect = { x , y , TILE_SIZE, TILE_SIZE };
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderFillRect(renderer, &playerRect);
}


void movePlayer(int dx, int dy) {
    if (screenstate != GAME && screenstate != AGENT) return;

    // Convert player's pixel position to grid position
    int gridX = startPos.first / TILE_SIZE;
    int gridY = startPos.second / TILE_SIZE;

    int newGridX = gridX + dx;  // Move in grid units
    int newGridY = gridY + dy;

    //std::cout << "Trying to move to Grid: (" << newGridX << ", " << newGridY << ")\n";

    // Ensure movement stays within bounds
    if (newGridX >= 0 && newGridX < 41 &&
        newGridY >= 0 && newGridY < 41 &&
        mazeData[newGridY][newGridX] != 1) {  // Only move if it's a floor, start, or exit

        startPos.first = newGridX * TILE_SIZE;  // Convert back to pixel position
        startPos.second = newGridY * TILE_SIZE;

        //std::cout << "Moved to Pixel Position: (" << startPos.first << ", " << startPos.second << ")\n";
    }
    else {
        std::cout << "Hit a wall or out of bounds!\n";
        wallsHit++;
    }

    // Get the local windoe (Test block)
    vector<vector<int>> localWindow = getLocalWindow(mazeData, { gridX, gridY }, 1);
    std::cout << "Local Window:\n";
    for (const auto& row : localWindow) {
        for (int cell : row) {
            std::cout << cell << " ";
        }
        std::cout << "\n";
    }

    // Check if the player has reached the exit
    if (startPos.first == exitPos.first && startPos.second == exitPos.second) {
        std::cout << "You win!\n";
        screenstate = WIN;
    }
}

void regenMaze(SDL_Texture*& texture) {
    /* Regenerates the maze to continue playing.
       In the future the regeneration will be influenced
       by the nueral Network
    */
    //reset the maze 
    maze = MazeMap(20, 20);
    maze.generateMaze(5, 10);
    maze.renderMaze(maze);

    system("python C:/Users/Robbie/RIT/Masters/MastersProj/Pythonstuff/BitMapGen.py");
    SDL_Texture* newTexture = loadTexture("GeneratedMazes/genmaze.bmp");
    if (!newTexture) return;
    // Free the old texture if necessary
    if (texture) {
        SDL_DestroyTexture(texture);
    }
    texture = newTexture;

    // Destroy the old Ai maze environment
    //mazeEnv = MazeEnvironment(mazeData, startPos, exitPos);

    startTime = SDL_GetTicks();
}


void WriteLog(const std::string& message) {
    // Write to both console and file
    std::cout << message << std::endl;
    std::ofstream log("C:\\Users\\Robbie\\RIT\\Masters\\MastersProj\\CStuff2\\src", std::ios::app);
    if (log) log << message << std::endl;
}

int main(int argc, char* argv[])
{
    /* Main Game Controller Logic
    */

    if (initSDL()) {
        std::cout << "Error initializing SDL" << std::endl;
        std::cout << "SDL Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    //initalize torch
    torch::manual_seed(6);
    torch::init();  // Explicit initialization

    int scaledMazeWidth = TILE_SIZE * 41;
    int scaledMazeHeight = TILE_SIZE * 41;

    int offsetX = (SCREEN_WIDTH - scaledMazeWidth) / 2;
    int offsetY = (SCREEN_HEIGHT - scaledMazeHeight) / 2;

    SDL_Rect dstRect;
    std::cout << "SDL Initialized Successfully!" << std::endl;

    // Game Startup Text
    SDL_Surface* instr1 = TTF_RenderText_Solid(font, "*Game Name Pending*", color);
    SDL_Surface* instr2 = TTF_RenderText_Solid(font, "Press 'Enter' to start the game", color);

    SDL_Texture* texture1 = SDL_CreateTextureFromSurface(renderer, instr1);
    SDL_Texture* texture2 = SDL_CreateTextureFromSurface(renderer, instr2);

    // Instruction Screen Text
    SDL_Surface* instr3 = TTF_RenderText_Solid(font, "Instructions:", color);
    SDL_Surface* instr4 = TTF_RenderText_Solid(font, "Use W/A/S/D to move", color);
    SDL_Surface* instr5 = TTF_RenderText_Solid(font, "Press ESC to quit", color);
    SDL_Surface* instr6 = TTF_RenderText_Solid(font, "Press Enter to start", color);
    SDL_Surface* instr7 = TTF_RenderText_Solid(font, "Press SHIFT to switch to Agent Mode", color);

    SDL_Texture* texture3 = SDL_CreateTextureFromSurface(renderer, instr3);
    SDL_Texture* texture4 = SDL_CreateTextureFromSurface(renderer, instr4);
    SDL_Texture* texture5 = SDL_CreateTextureFromSurface(renderer, instr5);
    SDL_Texture* texture6 = SDL_CreateTextureFromSurface(renderer, instr6);
    SDL_Texture* texture7 = SDL_CreateTextureFromSurface(renderer, instr7);


    SDL_QueryTexture(texture1, NULL, NULL, &text_width, &text_height);
    SDL_QueryTexture(texture2, NULL, NULL, &text_width, &text_height);

    // Generate the maze
    runGeneration(maze, 5, 10);
    system("python C:/Users/Robbie/RIT/Masters/MastersProj/Pythonstuff/BitMapGen.py");
    SDL_Texture* texture = loadTexture("GeneratedMazes/genmaze.bmp");  // Load a test bitmap


    // Create the AI Maze environment after python generation
    MazeEnvironment mazeEnv(mazeData, startPos, exitPos);

    //Load the hyperparameters for the maze agent
    int64_t state_size = 9; // 3x3 window
    int64_t action_size = 4; // 4 possible actions (N, E, S, W)
    float lr = 0.001; // Learning rate
    float gamma = 0.99; // Discount factor
    size_t buffer_size = 1000; // Replay buffer size
    size_t batch_size = 64; // Batch size for training
    int update_every = 4; // Update target network every 4 steps

	QNetworkImpl l_net(state_size, action_size);

    // Maze agent reset signals
    bool done = false;
    float eps = 1.0; // Epsilon for exploration
    int steps = 0; // Step counter
    std::vector<std::vector<int32_t>> localObs;
    State curObs;

    // Create the DQN agent
    DQNAgent agent(state_size, action_size, lr, gamma, buffer_size, batch_size, update_every);
    State prevObs;
    std::vector<std::vector<int>> rawNext;

    if (!texture) return 1;
    std::vector<SDL_Texture*> textures = { texture, texture1, texture2, texture3, texture4, texture5, texture6, texture7 };

    // Game input control loop
    while (!quit) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) { quit = true; }
            //SDL_RenderPresent(renderer);
            // Keyboard Input Events
            if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                case SDLK_RETURN: {
                    if (screenstate == MENU) {
                        SDL_RenderClear(renderer);
                        screenstate = CONTROLS;  // Change to instruction screen
                    }
                    else if (screenstate == CONTROLS)
                    {
                        SDL_RenderClear(renderer);
                        screenstate = GAME;  // Change to game screen
                        startTime = SDL_GetTicks();
                        //std::cout << "Start time set: " << startTime << std::endl;
                    }
                    else if (screenstate == WIN) {
                        endTime = SDL_GetTicks();
                        std::cout << "Time taken: " << (double)(endTime - startTime) / 1000.0 << " seconds" << std::endl;
                        //SDL_DestroyTexture(texture);
                        //SDL_RenderClear(renderer);
                        regenMaze(texture);

                        // New Maze new Maze Environment
                        mazeEnv = MazeEnvironment(mazeData, startPos, exitPos);

                        // dstRect = { offsetX, offsetY, scaledMazeWidth, scaledMazeHeight };
                        // SDL_RenderCopy(renderer, texture, NULL, &dstRect);
                        screenstate = GAME;
                    }

                    break;
                }
                case SDLK_a:
                    //std::cout << "A Pressed" << std::endl;
                    movePlayer(-1, 0);
                    break;

                case SDLK_w:
                    //std::cout << "W Pressed" << std::endl;
                    movePlayer(0, -1);
                    break;

                case SDLK_s:
                    //std::cout << "S Pressed" << std::endl;
                    movePlayer(0, 1);
                    break;

                case SDLK_d:
                    //std::cout << "D Pressed" << std::endl;
                    movePlayer(1, 0);
                    break;

                case SDLK_ESCAPE:
                    cleanup(window, renderer, font, textures);
                    quit = true;
                    break;

                case SDLK_LSHIFT: {
                    if (screenstate == CONTROLS) {
                        SDL_RenderClear(renderer);
                        screenstate = AGENT;

                        // reset environment and agent
                        localObs = mazeEnv.reset();
                        curObs = agent.flattenObs(localObs);

                        bool done = false;
                        float eps = 1.0; // Epsilon for exploration
                        int steps = 0; // Step counter

                        // Convert pixel coordinates to grid coordinates:
                        // std::pair<int,int> startCell = { startPos.first / TILE_SIZE, startPos.second / TILE_SIZE };
                        // std::pair<int,int> exitCell  = { exitPos.first / TILE_SIZE, exitPos.second / TILE_SIZE };
                        // MazeSearchProblem mazeSearchProblem(mazeData, startCell, exitCell);
                        // solution = aStarSearch(mazeSearchProblem);
                        // //std::cout << "Solution length: " << solution.size() << std::endl;
                        // solutionIndex = 0;
                        // lastMoveTime = SDL_GetTicks();
                    }
                    break;
                }
                }
            }
        }
        // Update game state independent of events.
        if (screenstate == AGENT && !done) {
            int actionIndex = agent.act(curObs, eps);
            std::string actionString = agent.getAction(actionIndex);

            auto [nextState, reward, done] = mazeEnv.takeStep(actionString);

            State nextObs = agent.flattenObs(nextState);
            agent.step(curObs, actionIndex, reward, nextObs, done);

            curObs = nextObs;
            done = done;
            eps = std::max(0.01f, eps * 0.995f); // Decay epsilon
            steps++;

            renderPlayer(startPos.first, startPos.second);

            // Uint32 currentTime = SDL_GetTicks();
            // if (!solution.empty() && solutionIndex < solution.size() && (currentTime - lastMoveTime > 100)) {
            //     string action = solution[solutionIndex];
            //     pair<int,int> delta = actionToDelta(action);
            //     movePlayer(delta.first, delta.second);
            //     // int gridX = startPos.first / TILE_SIZE + delta.first;
            //     // int gridY = startPos.second / TILE_SIZE + delta.second;
            //     // startPos.first = gridX * TILE_SIZE;
            //     // startPos.second = gridY * TILE_SIZE;
            //     solutionIndex++;
            //     lastMoveTime = currentTime;
            // }
        }

        // Render the current frame.
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Render based on screen state.
        if (screenstate == MENU) {
            // Render menu
            int text_width, text_height;
            SDL_QueryTexture(texture1, NULL, NULL, &text_width, &text_height);
            SDL_Rect titleCentered = { SCREEN_WIDTH / 2 - text_width / 2, 100, text_width, text_height };
            SDL_QueryTexture(texture2, NULL, NULL, &text_width, &text_height);
            SDL_Rect centerScreen = { SCREEN_WIDTH / 2 - text_width / 2, 300, text_width, text_height };
            SDL_RenderCopy(renderer, texture1, NULL, &titleCentered);
            SDL_RenderCopy(renderer, texture2, NULL, &centerScreen);
        }
        else if (screenstate == CONTROLS) {
            // Instruction Screen Render
            int text_width, text_height;
            SDL_QueryTexture(texture3, NULL, NULL, &text_width, &text_height);
            SDL_Rect instrPos1 = { SCREEN_WIDTH / 2 - text_width / 2, 0, text_width, text_height };

            SDL_QueryTexture(texture4, NULL, NULL, &text_width, &text_height);
            SDL_Rect instrPos2 = { SCREEN_WIDTH / 2 - text_width / 2, 100, text_width, text_height };

            SDL_QueryTexture(texture5, NULL, NULL, &text_width, &text_height);
            SDL_Rect instrPos3 = { SCREEN_WIDTH / 2 - text_width / 2, 200, text_width, text_height };

            SDL_QueryTexture(texture6, NULL, NULL, &text_width, &text_height);
            SDL_Rect instrPos4 = { SCREEN_WIDTH / 2 - text_width / 2, 300, text_width, text_height };

            SDL_QueryTexture(texture7, NULL, NULL, &text_width, &text_height);
            SDL_Rect instrPos5 = { SCREEN_WIDTH / 2 - text_width / 2, 400, text_width, text_height };

            SDL_RenderCopy(renderer, texture3, NULL, &instrPos1);
            SDL_RenderCopy(renderer, texture4, NULL, &instrPos2);
            SDL_RenderCopy(renderer, texture5, NULL, &instrPos3);
            SDL_RenderCopy(renderer, texture6, NULL, &instrPos4);
            SDL_RenderCopy(renderer, texture7, NULL, &instrPos5);
        }
        else if (screenstate == GAME) {
            dstRect = { offsetX, offsetY, scaledMazeWidth, scaledMazeHeight };
            SDL_RenderCopy(renderer, texture, NULL, &dstRect);
            int screenX = offsetX + startPos.first;
            int screenY = offsetY + startPos.second;
            renderPlayer(screenX, screenY);
        }
        else if (screenstate == WIN) {
            // Render win screen
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
            SDL_RenderClear(renderer);
            SDL_Surface* winSurface = TTF_RenderText_Solid(font, "YOU WIN!", color);
            SDL_Surface* continueSurface = TTF_RenderText_Solid(font, "Press Enter to continue, or ESC to quit", color);
            SDL_Texture* winTexture = SDL_CreateTextureFromSurface(renderer, winSurface);
            SDL_Texture* continueTexture = SDL_CreateTextureFromSurface(renderer, continueSurface);
            SDL_Rect winRect = { SCREEN_WIDTH / 2 - 100, SCREEN_HEIGHT / 2 - 20, 200, 40 };
            SDL_Rect continueRect = { SCREEN_WIDTH / 2 - 200, SCREEN_HEIGHT / 2 + 20, 400, 40 };
            SDL_RenderCopy(renderer, winTexture, NULL, &winRect);
            SDL_RenderCopy(renderer, continueTexture, NULL, &continueRect);
            SDL_FreeSurface(winSurface);
            SDL_FreeSurface(continueSurface);
            SDL_DestroyTexture(winTexture);
            SDL_DestroyTexture(continueTexture);
        }
        else if (screenstate == AGENT) {
            dstRect = { offsetX, offsetY, scaledMazeWidth, scaledMazeHeight };
            SDL_RenderCopy(renderer, texture, NULL, &dstRect);
            int screenX = offsetX + startPos.first;
            int screenY = offsetY + startPos.second;
            renderPlayer(screenX, screenY);
        }

        SDL_RenderPresent(renderer);

        SDL_Delay(10);
    }

    return EXIT_SUCCESS;
}