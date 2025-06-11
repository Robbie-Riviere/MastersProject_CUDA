from PIL import Image

def generateBitmap(txtfile, bmp_file):
    
    with open(txtfile, 'r') as file:
        maze_text = file.read().strip()

    # Convert to a list of lists
    maze = [list(row) for row in maze_text.strip().split("\n")]
    height = len(maze)
    width = len(maze[0])

    # Create an image in mode "P" (palettized, 8-bit)
    img = Image.new("P", (width, height))

    # Define a custom palette (768 values: 256 colors x 3 components)
    # Index 0: Black (wall)
    # Index 1: White (floor)
    # Index 2: Green (start)
    # Index 3: Blue (exit)
    palette = []
    palette.extend([255, 255, 255]) # index 0: White
    palette.extend([0, 0, 0])       # index 1: Black
    palette.extend([0, 255, 0])     # index 2: Green
    palette.extend([0, 0, 255])     # index 3: Blue

    # Fill the rest of the 256*3 entries with zeros.
    palette.extend([0] * (768 - len(palette)))
    img.putpalette(palette)

    # Fill the image with the maze data
    for y in range(height):
        for x in range(width):
            pixel_value = int(maze[y][x])
            img.putpixel((x, y),pixel_value)  # 0 = black (wall), 1 = white (path) 2 = green (start) 3 = blue (exit)

    # Save as BMP
    img.save(bmp_file)


if __name__ == "__main__":
    generateBitmap("C:/Users/Robbie/RIT/Masters/MastersProj/Cstuff/MastersNew/GeneratedMazes/genmaze.txt", "C:/Users/Robbie/RIT/Masters/MastersProj/Cstuff/MastersNew/GeneratedMazes/genmaze.bmp")



