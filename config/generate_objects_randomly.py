#!/usr/bin/python

import yaml
import random
import hashlib
import os

config_params = None
grid = None
obstacles = None
threats = None

def generate_obstacles():
    global config_params
    global grid
    global obstacles

    # Generate obastacles
    num_of_obstacle_cells = int(config_params["map_size"]["rows"] *
                                config_params["map_size"]["cols"] *
                                config_params["obstacle_ratio"])

    # Each obstacle (i, j, r, c)
    #   i = row index
    #   j = column index
    #   r = numbers of rows
    #   c = number of columns
    config_params["obstacles_areas"] = int(config_params["obstacles_areas"])
    if config_params["obstacles_areas"] == 0:
        obstacles = random.sample([(i, j, 1, 1) for j in range(config_params["map_size"]["cols"])
                                   for i in range(config_params["map_size"]["rows"])],
                                  num_of_obstacle_cells)

        for o in obstacles:
            grid[o[0]][o[1]] = -1
    else:
        obstacles = []
        remaining_obstacles = num_of_obstacle_cells // int(config_params["obstacles_areas"])

        while remaining_obstacles != 0:
            r = random.randint(1, config_params["obstacles_areas"])
            c = config_params["obstacles_areas"] // r
            q = config_params["obstacles_areas"] - (r * c)

            rr = 0
            cc = 0
            s = 0
            t = 0

            tries = 5

            while True:
                tries -= 1

                if tries == 0:
                    break

                i = random.randint(0, config_params["map_size"]["rows"] - r)
                j = random.randint(0, config_params["map_size"]["cols"] - c)

                if not all([grid[i + k][j + l] == 0 for k in range(r) for l in range(c)]):
                    continue

                if q == 0:
                    break

                if r > c:
                    s = random.randint(2, 3)
                elif r < c:
                    s = random.randint(0, 1)
                else:
                    s = random.randint(0, 3)

                if s == 0 or s == 1:
                    t = random.randint(0, c - q)
                    if s == 0:
                        rr = i - 1
                    elif s == 1:
                        rr = i + r
                    if rr < 0 or rr >= config_params["map_size"]["rows"]:
                        continue

                    if not all([grid[rr][j + t + l] == 0 for l in range(q)]):
                        continue
                elif s == 2 or s == 3:
                    t = random.randint(0, r - q)
                    if s == 2:
                        cc = j - 1
                    elif s == 3:
                        cc = j + c
                    if cc < 0 or cc >= config_params["map_size"]["cols"]:
                        continue

                    if not all([grid[i + t + l][cc] == 0 for l in range(q)]):
                        continue

                break

            if tries == 0:
                continue

            obstacles.append((i, j, r, c))
            for k in range(r):
                for l in range(c):
                    grid[i + k][j + l] = -1

            if q != 0:
                if s == 0 or s == 1:
                    obstacles.append((rr, j + t, 1, q))
                    for k in range(q):
                        grid[rr][j + t + k] = -1
                elif s == 2 or s == 3:
                    obstacles.append((i + t, cc, q, 1))
                    for k in range(q):
                        grid[i + t + k][cc] = -1

            remaining_obstacles -= 1

            if remaining_obstacles == 1: # Last obstacle
                config_params["obstacles_areas"] += num_of_obstacle_cells % int(config_params["obstacles_areas"])

def generate_threats():
    global config_params
    global grid
    global threats

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
    else:
        threats = []
        remaining_threats = num_of_threat_cells // int(config_params["threats_areas"])

        while remaining_threats != 0:
	    level = random.randint(1, config_params["threat_levels"])
            r = random.randint(1, config_params["threats_areas"])
            c = config_params["threats_areas"] // r
            q = config_params["threats_areas"] - (r * c)

            rr = 0
            cc = 0
            s = 0
            t = 0

            tries = 5

            while True:
                tries -= 1

                if tries == 0:
                    break

                i = random.randint(0, config_params["map_size"]["rows"] - r)
                j = random.randint(0, config_params["map_size"]["cols"] - c)

                if not all([grid[i + k][j + l] == 0 for k in range(r) for l in range(c)]):
                    continue

                if q == 0:
                    break

                if r > c:
                    s = random.randint(2, 3)
                elif r < c:
                    s = random.randint(0, 1)
                else:
                    s = random.randint(0, 3)

                if s == 0 or s == 1:
                    t = random.randint(0, c - q)
                    if s == 0:
                        rr = i - 1
                    elif s == 1:
                        rr = i + r
                    if rr < 0 or rr >= config_params["map_size"]["rows"]:
                        continue

                    if not all([grid[rr][j + t + l] == 0 for l in range(q)]):
                        continue
                elif s == 2 or s == 3:
                    t = random.randint(0, r - q)
                    if s == 2:
                        cc = j - 1
                    elif s == 3:
                        cc = j + c
                    if cc < 0 or cc >= config_params["map_size"]["cols"]:
                        continue

                    if not all([grid[i + t + l][cc] == 0 for l in range(q)]):
                        continue

                break

            if tries == 0:
                continue

            threats.append((i, j, level, r, c))
            for k in range(r):
                for l in range(c):
                    grid[i + k][j + l] = level

            if q != 0:
                if s == 0 or s == 1:
                    threats.append((rr, j + t, level, 1, q))
                    for k in range(q):
                        grid[rr][j + t + k] = level
                elif s == 2 or s == 3:
                    threats.append((i + t, cc, level, q, 1))
                    for k in range(q):
                        grid[i + t + k][cc] = level

            remaining_threats -= 1

            if remaining_threats == 1: # Last obstacle
                config_params["threats_areas"] += num_of_threat_cells % int(config_params["threats_areas"])

def isConnected():
    def adjacnet_cells(i, j):
        ret = []

        if i > 0:
            ret.append((i - 1, j))
        if i < config_params["map_size"]["rows"] - 1:
            ret.append((i + 1, j))

        if j > 0:
            ret.append((i, j - 1))
        if j < config_params["map_size"]["cols"] - 1:
            ret.append((i, j + 1))

        return ret

    global config_params
    global grid

    # Find empty cell
    empty_cells = [(i, j) for i in range(config_params["map_size"]["rows"]) for j in range(config_params["map_size"]["cols"]) if grid[i][j] == 0]

    i, j = empty_cells[0]
    reachable_cells = []
    reachable_cells_n = [(i, j)]

    while set(reachable_cells) != set(reachable_cells_n):
	reachable_cells = list(set(reachable_cells_n))
        reachable_cells_n = []

        for i, j in reachable_cells:
            adj_cells = adjacnet_cells(i, j)

            for adj_cell in adj_cells:
                if grid[adj_cell[0]][adj_cell[1]] == 0:
                    reachable_cells_n.append(adj_cell)

        reachable_cells_n.extend(reachable_cells)

    return set(empty_cells) == set(reachable_cells)

def main():
    # Read config file

    config_file_path = os.path.dirname(__file__)
    config_file_name = "config.yaml"

    global config_params
    global grid
    global obstacles
    global threats

    try:
        with open(os.path.join(config_file_path, config_file_name), "r") as config_input_file:
            try:
                config_params = yaml.load(config_input_file)
            except yaml.YAMLError as exc:
                print exc
    except IOError:
        print "Can't file config.yaml"

    random.seed(config_params["random_seed"])

    assert config_params["obstacle_ratio"] + config_params["threat_ratio"] < 1, "Too much obstacles and threats"

    connected = False
    while not connected:
        # Generate grid
        grid = [[0 for _ in xrange(config_params["map_size"]["cols"])] for _ in xrange(config_params["map_size"]["rows"])]

        generate_obstacles()

        connected = isConnected()

    generate_threats()

    # Print generated grid
    for i in range(len(grid)):
        print "".join(["%3s" % (str(t)) for t in grid[i]])

    objects = {}
    objects["map_size"] = dict(
        rows=config_params["map_size"]["rows"],
        cols=config_params["map_size"]["cols"]
    )

    objects["obstacles"] = {}
    for i, o in enumerate(obstacles):
        objects["obstacles"][i] = dict(
            start_row=o[0],
            start_col=o[1],
            num_rows=o[2],
            num_cols=o[3]
        )

    objects["threats"] = {}
    for i, t in enumerate(threats):
        objects["threats"][i] = dict(
            start_row=t[0],
            start_col=t[1],
            level=t[2],
            num_rows=t[3],
            num_cols=t[4]
        )

    hasher = hashlib.md5()
    with open(os.path.join(config_file_path, config_file_name), "rb") as config_input_file:
        for chunk in iter(lambda: config_input_file.read(4096), b""):
            hasher.update(chunk)

    objects["id"] = hasher.hexdigest()
    objects["threat_levels"] = config_params["threat_levels"]
    objects["max_threat_prob"] = config_params["max_threat_prob"]
    objects["risk_factor"] = config_params["risk_factor"]
    objects["robots_num"] = config_params["robots_num"]
    objects["cell_size"] = config_params["cell_size"]

    with open(os.path.join(config_file_path, "input.yaml"), "w") as config_output_file:
        config_output_file.write(yaml.dump(objects, default_flow_style=False))
    print "input.yaml was generated successfully!"


if __name__ == "__main__":
    main()
