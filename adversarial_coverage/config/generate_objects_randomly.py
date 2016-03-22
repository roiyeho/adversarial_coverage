#!/usr/bin/python

import yaml
import random

def main():
    # Read config file
    try:
        with open("config.yaml", "r") as config_input_file:
            try:
                config_params = yaml.load(config_input_file)
            except yaml.YAMLError as exc:
                print exc
    except IOError:
        print "Can't file config.yaml"

    random.seed(config_params["random_seed"])

    if config_params["obstacle_ratio"] + config_params["threat_ratio"] >= 1:
        print "Too much obstacles and threats"

        return

    # Generate grid
    grid = [[0 for _ in xrange(config_params["map_size"]["cols"])] for _ in xrange(config_params["map_size"]["rows"])]

    # Generate obastacles
    num_of_obstacle_cells = int(config_params["map_size"]["rows"] * \
                                config_params["map_size"]["cols"] * \
                                config_params["obstacle_ratio"])

    # Each obstacle (i, j, r, c)
    #   i = row index
    #   j = column index
    #   r = numbers of rows
    #   c = number of columns
    if config_params["obstacles_areas"] == 0:
        obstacles = random.sample([(i, j, 1, 1) for j in range(config_params["map_size"]["cols"])
                                          for i in range(config_params["map_size"]["rows"])], num_of_obstacle_cells)

        for o in obstacles:
            grid[o[0]][o[1]] = -1

    # Generate threats
    num_of_threat_cells = int(config_params["map_size"]["rows"] * \
                              config_params["map_size"]["cols"] * \
                              config_params["threat_ratio"])

    # Each threat (i, j, l)
    #   i = row index
    #   j = column index
    #   l = threat level
    if config_params["threats_areas"] == 0:
        threats = random.sample([(i, j, random.randint(1, config_params["threat_levels"]))
                                        for j in range(config_params["map_size"]["cols"])
                                        for i in range(config_params["map_size"]["rows"]) if grid[i][j] == 0],
                                        num_of_threat_cells)

        for t in threats:
            grid[t[0]][t[1]] = t[2]

    for i in range(len(grid)):
        print "".join(["%3s" % (str(t)) for t in grid[i]])

    objects = { }
    objects["map_size"] = dict(
        rows = config_params["map_size"]["rows"],
        cols = config_params["map_size"]["cols"]
    )

    objects["obstacles"] = { }
    for i, o in enumerate(obstacles):
        objects["obstacles"][i] = dict(
            start_row   = o[0],
            start_col   = o[1],
            num_rows    = o[2],
            num_cols    = o[3]
        )

    objects["threats"] = { }
    for i, t in enumerate(threats):
        objects["threats"][i] = dict(
            start_row   = t[0],
            start_col   = t[1],
            level       = t[2],
        )

    objects["max_threat_prob"]  = config_params["max_threat_prob"]
    objects["risk_factor"]      = config_params["risk_factor"]
    objects["robots_num"]       = config_params["robots_num"]

    with open("input.yaml", "w") as config_output_file:
        config_output_file.write(yaml.dump(objects, default_flow_style=False))

if __name__ == "__main__":
    main()
