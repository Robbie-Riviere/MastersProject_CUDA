import pygame
from random import choice

class Cell:
    def __init__(self, x, y, thickness):
        # Position of the cell
        self.x = x
        self.y = y
        # Thickness for drawing the walls
        self.thickness = thickness
        # Position Tracking
        self.walls = {'top': True, 'right': True, 'bottom': True, 'left': True}
        self.visited = False
    
    # Draw the cell walls: self - the cell to draw, screen - the pygame screen, size - the size of the cell
    def draw(self, screen, size):
        # Find the starting Cell position
        x, y = self.x * size, self.y * size

        # Draw the walls
        if self.walls['top']:
            pygame.draw.line(screen, pygame.Color('Green'), (x, y), (x + size, y), self.thickness)
        if self.walls['right']:
            pygame.draw.line(screen, pygame.Color('Orange'), (x + size, y), (x + size, y + size), self.thickness)
        if self.walls['bottom']:
            pygame.draw.line(screen, pygame.Color('Blue'), (x + size, y + size), (x, y + size), self.thickness)
        if self.walls['left']:
            pygame.draw.line(screen, pygame.Color('Purple'), (x, y + size), (x, y), self.thickness)
        
    # Check if the cell is real
    def check_cell(self, x, y, r, c, grid):
        # convert the x, y coordinates to the index of the cell in the grid (2D to 1D)
        find_index = lambda x, y: x + y * c
        if x < 0 or y < 0  or x > c - 1 or y > r - 1:
            return False
        return grid[find_index(x, y)]
    
    # Check the neighbors of the cell
    def check_neighbors(self, r, c, grid):
        neighbors = []
        top = self.check_cell(self.x, self.y - 1, r, c, grid)
        right = self.check_cell(self.x + 1, self.y, r, c, grid)
        bottom = self.check_cell(self.x, self.y + 1, r, c, grid)
        left = self.check_cell(self.x - 1, self.y, r, c, grid)

        if top and not top.visited:
            neighbors.append(top)
        if right and not right.visited:
            neighbors.append(right)
        if bottom and not bottom.visited:
            neighbors.append(bottom)
        if left and not left.visited:
            neighbors.append(left)
        
        if neighbors:
            return choice(neighbors)
        return None
    
