/*
 * Robot.h
 *
 *  Created on: Aug 23, 2015
 *      Author: roiyeho
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "GeneralDefinitions.h"
#include "Map.h"

class Robot {
private:
	ros::NodeHandle nh;

	double cellSize;
	double highLinearSpeed;
	double lowLinearSpeed;
	double angularSpeed;
	double linearTolerance;
	double angularTolerance;
	double minDistanceFromObstacles;

	const Map &map;
	int rows, cols;

	ros::Publisher cmdVelPublisher;
	tf::TransformListener listener;

	//Position initialPosition;
	Position currentPosition;
	double currentAngle;
	Cell currentCell;
	Direction currentDirection;
	const char* directionNames[5];

	// Statistics
	float total90DegreesTurningTime;
	int numOf90DegreesTurns;
	float total180DegreesTurningTime;
	int numOf180DegreesTurns;
	float totalTurningTime;
	int numOfTurns;

	float totalMovingForwardToCellTime;
	int numOfStepsForwardToCell;
	float totalMovingForwardToPositionTime;
	int numOfStepsForwardToPosition;

	// Private methods
	double computeRotationTolerance();
	void getCurrentPose();
	void convertCurrentPositionToCell();
	void printCurrentPose();
	void printCurrentDirection();
	void moveToCell(Cell targetCell, Cell nextCellAfterTarget, const Path &coveragePath);
	Direction findDirection(Cell sourceCell, Cell targetCell);
	void rotateRobotToNewDirection(Direction newDirection);
	void adjustTargetPositionNearWalls(Cell targetCell, Position &targetPosition, Direction direction);
	void moveForwardToNextCell(Cell targetCell);
	void moveForwardToPreciseLocation(Position targetPosition);
	Position convertCellToPosition(Cell cell);
	Direction findCurrentDirection() const;

public:
	Robot(const Map &map);
	Cell getCurrentCell() const;
	Direction getCurrentDirection() const;
	void startCoverage(const Path &coveragePath);
	float getAverageTurningTime() const;
	float getAverage90DegreesTurningTime() const;
	float getAverage180DegreesTurningTime() const;
	float getAverageMovingForwardToCellTime() const;
	float getAverageMovingForwardToPositionTime() const;
	virtual ~Robot();
};

#endif /* ROBOT_H_ */
