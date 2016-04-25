#!/usr/bin/python

import yaml
import xml.etree.ElementTree as ET
import os

def gen_obstacle_xacro_param(i, o):
    ret = ET.Element("param")

    ret.attrib["name"] = "obstacle_%s" % i
    ret.attrib["command"] = "$(find xacro)/xacro.py $(find adversarial_coverage)/objects/obstacle.urdf.xacro row:=%s col:=%s width:=%s height:=%s" % \
                               (str(float(o["start_row"]) - 10 + 0.5 * float(o["num_rows"])), str(float(o["start_col"]) - 10 + 0.5 * float(o["num_cols"])), \
                               o["num_rows"], o["num_cols"])

    return ret

def gen_obstacle_spawn_node(i):
    ret = ET.Element("node")

    ret.attrib["name"]    = "spawn_obstacle_%s" % i
    ret.attrib["pkg"]     = "gazebo_ros"
    ret.attrib["type"]    = "spawn_model"
    ret.attrib["args"]    = "-urdf -param obstacle_%s -model obstacle_%s" % (i, i)
    ret.attrib["respawn"] = "false"
    ret.attrib["output"]  = "screen"

    return ret

def gen_threat_xacro_param(i, o):
    ret = ET.Element("param")

    ret.attrib["name"] = "threat_%s" % i
    ret.attrib["command"] = "$(find xacro)/xacro.py $(find adversarial_coverage)/objects/threat.urdf.xacro row:=%s col:=%s width:=%s height:=%s level:=%s" % \
                               (str(float(o["start_row"]) - 10 + 0.5 * float(o["num_rows"])), str(float(o["start_col"]) - 10 + 0.5 * float(o["num_cols"])), \
                               o["num_rows"], o["num_cols"], o["level"])

    return ret

def gen_threat_spawn_node(i):
    ret = ET.Element("node")

    ret.attrib["name"]    = "spawn_threat_%s" % i
    ret.attrib["pkg"]     = "gazebo_ros"
    ret.attrib["type"]    = "spawn_model"
    ret.attrib["args"]    = "-urdf -param threat_%s -model threat_%s" % (i, i)
    ret.attrib["respawn"] = "false"
    ret.attrib["output"]  = "screen"

    return ret

def main():
    input_file_path = os.path.dirname(__file__)

    tree = ET.parse(os.path.join(input_file_path, "../launch/empty_world.launch"))
    launch = tree.getroot()

    # Read input file
    try:
        with open(os.path.join(input_file_path, "input.yaml"), "r") as input_file:
            try:
                input_yaml = yaml.load(input_file)
            except yaml.YAMLError as exc:
                print exc
    except IOError:
        print "Can't file input.yaml"

    for i, obstacle in input_yaml["obstacles"].iteritems():
        p = gen_obstacle_xacro_param(i, obstacle)
        launch.append(p)
        l = gen_obstacle_spawn_node(i)
        launch.append(l)

    for i, threat in input_yaml["threats"].iteritems():
        p = gen_threat_xacro_param(i, threat)
        launch.append(p)
        l = gen_threat_spawn_node(i)
        launch.append(l)


    #print etree.tostring(tree, pretty_print=True)
    tree.write(os.path.join(input_file_path, ("../launch/%s.launch" % (input_yaml["id"]))))

if __name__ == "__main__":
    main()
