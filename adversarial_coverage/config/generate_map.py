#!/usr/bin/python

import yaml
import png
import os

def fill_grid_with_object(o, grid, t,pixelToCm):
    start_r = int(o["start_row"])*pixelToCm
    start_c = int(o["start_col"])*pixelToCm

    if t == "obstacle":
        v = 1
    elif t == "threat":
        v = 1 + o["level"]

    for i in range(int(o["num_rows"])*pixelToCm):
        for j in range(int(o["num_cols"])*pixelToCm):
            grid[start_r + i][start_c + j] = v

def main():
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

    pixelToCm = int(input_yaml["pixel_size"]/0.05)

    grid = [[0 for _ in range(int(input_yaml["map_size"]["cols"])*pixelToCm)] for _ in range(int(input_yaml["map_size"]["rows"])*pixelToCm)]

    for obstacle in input_yaml["obstacles"].itervalues():
        fill_grid_with_object(obstacle, grid, "obstacle",pixelToCm)

    max_threat_level = int(input_yaml["threat_levels"])
    for threat in input_yaml["threats"].itervalues():
        fill_grid_with_object(threat, grid, "threat",pixelToCm)

    for i in range(int(input_yaml["map_size"]["rows"])*pixelToCm):
        for j in range(int(input_yaml["map_size"]["cols"])*pixelToCm):
            print grid[i][j], " ",
        print

    palette = [
                (0xff, 0xff, 0xff),  # Empty cell
                (0x0, 0x0, 0x0)  # Obstacle
              ]
    # Add palette values for different threat levels
    for i in range(max_threat_level):
        palette.append((i * int(0xff/6), 0x0, 0x0))

    map_png = png.Writer(len(grid[0]), len(grid), palette=palette, bitdepth=8)

    try:
	grid = map(lambda x: map(int, x), grid)
        f = open('%s.png'%(input_yaml["id"]), 'wb')
	w = png.Writer(len(grid[0]), len(grid), palette=palette, bitdepth=8)	
	w.write(f, grid)
	f.close()
    except:
        print "Can't write png file"



	

if __name__ == "__main__":
    main()
