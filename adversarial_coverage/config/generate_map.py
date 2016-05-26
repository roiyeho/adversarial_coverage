#!/usr/bin/python

import yaml
import png
import os

pixels_per_cell = 0

def fill_grid_with_object(o, grid, t):
    global pixels_per_cell

    start_r = int(o["start_row"]) * pixels_per_cell
    start_c = int(o["start_col"]) * pixels_per_cell

    if t == "obstacle":
        v = 1
    elif t == "threat":
        v = 1 + o["level"]

    for i in range(int(o["num_rows"]) * pixels_per_cell):
        for j in range(int(o["num_cols"]) * pixels_per_cell):
            grid[start_r + i][start_c + j] = v

def main():
    PIXEL_SIZE = 0.05
    global pixels_per_cell

    input_file_path = os.path.dirname(__file__)

    # Read input file
    try:
        with open(os.path.join(input_file_path, "input.yaml"), "r") as input_file:
            try:
                input_yaml = yaml.load(input_file)
            except yaml.YAMLError as exc:
                print exc
    except IOError:
        print "Can't file input.yaml"

    cell_size       = float(input_yaml["cell_size"])
    pixels_per_cell = int(cell_size / PIXEL_SIZE)

    grid = [[0 for _ in range(int(input_yaml["map_size"]["cols"]) * pixels_per_cell)] 
               for _ in range(int(input_yaml["map_size"]["rows"]) * pixels_per_cell)]

    for obstacle in input_yaml["obstacles"].itervalues():
        fill_grid_with_object(obstacle, grid, "obstacle")

    max_threat_level = int(input_yaml["threat_levels"])
    for threat in input_yaml["threats"].itervalues():
        fill_grid_with_object(threat, grid, "threat")

    for i in range(int(input_yaml["map_size"]["rows"])):
        for j in range(int(input_yaml["map_size"]["cols"])):
            print grid[i * pixels_per_cell][j * pixels_per_cell], " ",
        print

    palette = [
                (0xff, 0xff, 0xff),  # Empty cell
                (0x0, 0x0, 0x0)  # Obstacle
              ]

    # Add palette values for different threat levels
    for i in range(max_threat_level):
        palette.append(((i + 1) * int(0xff/(max_threat_level + 2)), 0x0, 0x0))

    map_png = png.Writer(len(grid[0]), len(grid), palette=palette, bitdepth=8)

    try:
        output_png = open(os.path.join(input_file_path, ("../maps/%s.png" % (input_yaml["id"]))), "wb")
        map_png.write(output_png, grid)
    except:
        print "Can't write png file"
    else:
        output_png.close()

    try:
        output_yaml = open(os.path.join(input_file_path, ("../maps/%s.yaml" % (input_yaml["id"]))), "w")
        output_yaml.write('image: %s.yaml\n' % (input_yaml["id"]))
	output_yaml.write('resolution: %0.3f\n' % (PIXEL_SIZE))
	output_yaml.write('origin: [0.0, 0.0, 0.0]\n')
	output_yaml.write('occupied_thresh: 0.65\n')
	output_yaml.write('free_thresh: 0.196\n')
	output_yaml.write('negate: 0\n')
    except:
        print "Can't write yaml file"
    else:
        output_yaml.close()

if __name__ == "__main__":
    main()
