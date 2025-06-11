import pygame
import os
from cell import Cell

class Maze:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.size = 4
        self.grid_cells = [Cell(col, row, self.size) for row in range(rows) for col in range(cols)]
    
    # Current: the current cell, next: the next cell
    def remove_walls(self, current, next):
        dx = current.x - next.x
        if dx == 1:
            current.walls['left'] = False
            next.walls['right'] = False
        elif dx == -1:
            current.walls['right'] = False
            next.walls['left'] = False
        dy = current.y - next.y
        if dy == 1:
            current.walls['top'] = False
            next.walls['bottom'] = False
        elif dy == -1:
            current.walls['bottom'] = False
            next.walls['top'] = False 

        # generate maze
    def generate_maze(self):
        current_cell = self.grid_cells[0]
        array = []
        break_count = 1
        while break_count != len(self.grid_cells):
            current_cell.visited = True
            next_cell = current_cell.check_neighbors(self.cols, self.rows, self.grid_cells)
            if next_cell:
                next_cell.visited = True
                break_count += 1
                array.append(current_cell)
                self.remove_walls(current_cell, next_cell)
                current_cell = next_cell
            elif array:
                current_cell = array.pop()
        return self.grid_cells
    
    def generateBitMap(self):
        # Create a 2D array representing the maze
        maze_bitmap = [[1 for _ in range(self.cols * 2 + 1)] for _ in range(self.rows * 2 + 1)]
        
        # Fill the array based on the walls in each cell
        for cell in self.grid_cells:
            x = cell.x * 2 + 1
            y = cell.y * 2 + 1
            maze_bitmap[y][x] = 0  # Mark the cell as open space
            
            # Check the walls and update the bitmap
            if not cell.walls['top']:
                maze_bitmap[y - 1][x] = 0  # Open top
            if not cell.walls['bottom']:
                maze_bitmap[y + 1][x] = 0  # Open bottom
            if not cell.walls['left']:
                maze_bitmap[y][x - 1] = 0  # Open left
            if not cell.walls['right']:
                maze_bitmap[y][x + 1] = 0  # Open right

        # Write the bitmap to a file
        with open("maze.txt", "w") as file:
            for row in maze_bitmap:
                file.write("".join(map(str, row)) + "\n")
