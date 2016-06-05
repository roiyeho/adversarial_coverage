/*
 * Coverage.cpp
 *
 *  Created on: Aug 13, 2015
 *      Author: roiyeho
 */

#include "Coverage.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <time.h>
#include <fstream>
#include "GreedyAdversarialCoverage.h"
#include "Map.h"
#include "PathUtils.h"
#include "GeneralUtils.h"
#include "Logger.h"

Coverage::Coverage() : map(), robot(map), coverageTime(0) {
	nh.getParam("stat_file", statsFilePath);

	initialRobotCell = robot.getCurrentCell();
	initialRobotDirection = robot.getCurrentDirection();

	GreedyAdversarialCoverage gac(nh, map, initialRobotCell, initialRobotDirection);
	gac.buildCoveragePath(coveragePath);
	Logger::getInstance().write("Coverage path:");

	//TODO: Debug, Delete!
	if(coveragePath.size() == 0)
		Logger::getInstance().write("Well, youre screwd.\n");
	Logger::getInstance().write("Size is not Zero.\n");
	Logger::getInstance().printPath(coveragePath);

	printStatistics();
}

void Coverage::start() {
	time_t start, end;
	time (&start);
	robot.startCoverage(coveragePath);
	time (&end);
	coverageTime = end - start;

	stringstream logMessage;
	logMessage << "Coverage time was " << coverageTime << " seconds.";
	Logger::getInstance().write(logMessage.str());

	printStatistics();
}

void Coverage::printStatistics() const {
	string fileName = statsFilePath + "_" + GeneralUtils::getDateString() + ".txt";
	ofstream file(fileName.c_str());

	int accessibleCellsNum, accessibleDangerousCellsNum;
	map.getNumberOfAccessibleCells(initialRobotCell, accessibleCellsNum, accessibleDangerousCellsNum);
	double riskFactor;
	nh.getParam("risk_factor", riskFactor);

	int numOfTurns, numOf90DegreesTurns, numOf180DegreesTurns;
	PathUtils::getNumberOfTurns(initialRobotCell, initialRobotDirection, coveragePath,
			numOfTurns, numOf90DegreesTurns, numOf180DegreesTurns);

	file << fixed << setprecision(3);

	file << "Number of free cells: " << map.getNumberOfFreeCells() << endl;
	file << "Number of accessible free cells: " << accessibleCellsNum << endl;
	file << "Number of accessible dangerous cells: " << accessibleDangerousCellsNum << endl;
	file << "Risk factor: " << riskFactor << endl;
	file << "Coverage path length: " << (coveragePath.size() + 1) << endl;
	file << "Number of threats visits: " << PathUtils::getNumberOfThreatsVisits(coveragePath, map) << endl;
	file << "Survivability probability: " << PathUtils::getSurvivabilityProbability(coveragePath, map) * 100 << "%" << endl;
	file << "Number of turns: " << numOfTurns << endl;
	file << "Number of 90 degrees turns: " << numOf90DegreesTurns << endl;
	file << "Number of 180 degrees turns: " << numOf180DegreesTurns << endl;
	file << "Average turning time: " << robot.getAverageTurningTime() << " seconds" << endl;
	file << "Average 90 degrees turning time: " << robot.getAverage90DegreesTurningTime() << " seconds" << endl;
	file << "Average 180 degrees turning time: " << robot.getAverage180DegreesTurningTime() << " seconds" << endl;
	file << "Average moving forward to cell time: " << robot.getAverageMovingForwardToCellTime() << " seconds" << endl;
	file << "Average moving forward to position time: " << robot.getAverageMovingForwardToPositionTime() << " seconds" << endl;
	file << "Coverage time: " << coverageTime << " seconds" << endl;

	file.close();
}

Coverage::~Coverage() {
}

