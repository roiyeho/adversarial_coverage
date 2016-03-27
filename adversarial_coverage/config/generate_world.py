#!/usr/bin/python

import yaml
import xml.etree.ElementTree as ET
import os

def gen_obstacle_element(i, o):
    ret = ET.Element("include")

    uri = ET.SubElement(ret, "uri")
    uri.text = "model://obstacle"

    name = ET.SubElement(ret, "name")
    name.text = "obstacle_%s" % (i)

    pose = ET.SubElement(ret, "pose")
    pose.text = "%s %s %s %s %s %s" % (str(float(o["start_row"]) - 9.5), str(float(o["start_col"] - 9.5)), 0, 0, 0, 0)

    return ret

def main():
    input_file_path = os.path.dirname(__file__)

    tree = ET.parse(os.path.join(input_file_path, "../worlds/empty.world"))
    sdf = tree.getroot()

    world = sdf[0]

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
        o = gen_obstacle_element(i, obstacle)
        world.append(o)

    #print etree.tostring(tree, pretty_print=True)
    tree.write(os.path.join(input_file_path, ("../worlds/%s.world" % (input_yaml["id"]))))

if __name__ == "__main__":
    main()
