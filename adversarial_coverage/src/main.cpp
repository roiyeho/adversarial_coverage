/*
 * Main.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: roiyeho
 */

#include <ros/ros.h>
#include <string>
#include "Coverage.h"
#include "Map.h"

using namespace std;

void generateThreatsFile(char **argv);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "adversarial_coverage_node");

	if (argc == 5) {
		generateThreatsFile(argv);
		return 0;
	}

	Coverage coverage;
	coverage.start();
	return 0;
}

void generateThreatsFile(char **argv) {
	int threatsNum = atoi(argv[1]);
	int dangerousAreasNum = atoi(argv[2]);
	float threatProbability = atof(argv[3]);
	char *fileName = argv[4];

	Map map;
	Robot robot(map);
	Cell startingCell = robot.getCurrentCell();

	// Initialize random seed
	srand (time(NULL));

	map.generateRandomThreatsFile(startingCell, threatsNum, dangerousAreasNum, threatProbability, fileName);
	ROS_INFO("Finished creating threats file");
}



