#!/usr/bin/python

import yaml
import png
import os

def fill_grid_with_object(o, grid, t,cmToPixel):
    start_r = int(o["start_row"])*cmToPixel
    start_c = int(o["start_col"])*cmToPixel

    if t == "obstacle":
        v = 1
    elif t == "threat":
        v = 1 + o["level"]

    for i in range(int(o["num_rows"])*cmToPixel):
        for j in range(int(o["num_cols"])*cmToPixel):
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

    cmToPixel = int(input_yaml["cell_size"]/0.05)

    grid = [[0 for _ in range(int(input_yaml["map_size"]["cols"])*cmToPixel)] for _ in range(int(input_yaml["map_size"]["rows"])*cmToPixel)]

    for obstacle in input_yaml["obstacles"].itervalues():
        fill_grid_with_object(obstacle, grid, "obstacle",cmToPixel)

    max_threat_level = int(input_yaml["threat_levels"])
    for threat in input_yaml["threats"].itervalues():
        fill_grid_with_object(threat, grid, "threat",cmToPixel)

    for i in range(int(input_yaml["map_size"]["rows"])*cmToPixel):
        for j in range(int(input_yaml["map_size"]["cols"])*cmToPixel):
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

    #create the image yaml file:
    objects = {}

    #image:Path to the image file containing the occupancy data; can be absolute, or relative to the location of the YAML file
    objects["image"] = ('%s.yaml'%input_yaml["id"])

    #resolution : Resolution of the map, meters / pixel
    objects["resolution"] = cmToPixel

    #origin:The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise rotation.
    objects["origin"] = [0.0, 0.0, 0.0]

    #occupied_thresh : Pixels with occupancy probability greater than this threshold are considered completely occupied.
    objects["occupied_thresh"] =  0.65

    #free_thresh : Pixels with occupancy probability less than this threshold are considered completely free.
    objects["free_thresh"] =  0.65

    #negate : Whether the white/black free/occupied semantics should be reversed (interpretation of thresholds is unaffected) 
    objects["negate"] = 0

    with open('%s.yaml'%(input_yaml["id"]), "w") as map_yaml:
        map_yaml.write(yaml.dump(objects))


	

if __name__ == "__main__":
    main()
