/*
 * Coverage.h
 *
 *  Created on: Aug 13, 2015
 *      Author: roiyeho
 */

#ifndef COVERAGE_H_
#define COVERAGE_H_

#include <ros/ros.h>
#include "GeneralDefinitions.h"
#include "Robot.h"
#include "Map.h"

class Coverage {
private:
	ros::NodeHandle nh;
	Map map;
	Robot robot;
	Cell initialRobotCell;
	Direction initialRobotDirection;
	Path coveragePath;
	int coverageTime;

	string statsFilePath;

	void printStatistics() const;

public:
	Coverage();
	void start();
	virtual ~Coverage();
};

#endif /* COVERAGE_H_ */
