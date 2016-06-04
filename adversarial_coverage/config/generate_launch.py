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

def gen_walls(r, c):
    v = [(str(0 - 10 + 0.5 * r), str(0 - 10), str(0 - 10 + 1.5 * r), str(0 - 10)), \
         (str(0 - 10), str(0 - 10 + 0.5 * c), str(0 - 10), str(0 - 10 + 1.5 * c)), \
         (str(0 - 10 + 0.5 * r), str(0 - 10 + c), str(0 - 10 + 1.5 * r), str(0 - 10 + c)), \
         (str(0 - 10 + r), str(0 - 10 + 0.5 * c), str(0 - 10 + r), str(0 - 10 + 1.5 * c))
        ]

    ret = []

    for i in range(len(v)):
        t = ET.Element("param")

        t.attrib["name"] = "wall_%s" % str(i)
        t.attrib["command"] = "$(find xacro)/xacro.py $(find adversarial_coverage)/objects/wall.urdf.xacro start_x:=%s start_y:=%s end_x:=%s end_y:=%s" % v[i]
        ret.append(t)

        t = ET.Element("node")

        t.attrib["name"]    = "spawn_wall_%s" % str(i)
        t.attrib["pkg"]     = "gazebo_ros"
        t.attrib["type"]    = "spawn_model"
        t.attrib["args"]    = "-urdf -param wall_%s -model wall_%s" % (str(i), str(i))
        t.attrib["respawn"] = "false"
        t.attrib["output"]  = "screen"
        ret.append(t)

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

    for w in gen_walls(float(input_yaml["map_size"]["rows"]), float(input_yaml["map_size"]["cols"])):
        launch.append(w)

    #print etree.tostring(tree, pretty_print=True)
    xml_file_name = os.path.abspath(os.path.join(input_file_path, ("../launch/%s.launch" % (input_yaml["id"]))))
    tree.write(xml_file_name)

    # Indent the file (Using xmllint)
    try:
        os.system("xmllint --format %s > %s.formatted" % (xml_file_name, xml_file_name))
        os.system("mv %s.formatted %s" % (xml_file_name, xml_file_name))
        print "%s was generated successfully!" % xml_file_name	
        #call(["xmllint", "--format", "%s.raw" % (xml_file_name), ">", xml_file_name])
    except:
        print "Error generating launch file"

if __name__ == "__main__":
    main()
