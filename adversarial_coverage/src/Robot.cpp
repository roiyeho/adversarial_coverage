/*
 * Robot.cpp
 *
 *  Created on: Aug 23, 2015
 *      Author: roiyeho
 */

#include "Robot.h"
#include <geometry_msgs/Twist.h>
#include <cmath>
#include "PathUtils.h"
#include "Logger.h"
#include "GeneralUtils.h"

Robot::Robot(const Map &map): map(map), totalTurningTime(0), numOfTurns(0),
	total90DegreesTurningTime(0), numOf90DegreesTurns(0), total180DegreesTurningTime(0), numOf180DegreesTurns(0),
	totalMovingForwardToCellTime(0), numOfStepsForwardToCell(0), totalMovingForwardToPositionTime(0),
	numOfStepsForwardToPosition(0) {
	double robotWidth, robotLength;
	nh.getParam("robot_width", robotWidth);
	nh.getParam("robot_length", robotLength);
	cellSize = max(robotWidth, robotLength);

	nh.getParam("robot_name", robotName);
	nh.getParam("high_linear_speed", highLinearSpeed);
	nh.getParam("low_linear_speed", lowLinearSpeed);
	nh.getParam("angular_speed", angularSpeed);
	nh.getParam("linear_tolerance", linearTolerance);
	nh.getParam("angular_tolerance", angularTolerance);

	cmdVelPublisher = nh.advertise<geometry_msgs::Twist>(robotName+"/"CMD_VEL, 10);

	const Grid &grid = map.getGrid();
	rows = grid.size();
	cols = grid[0].size();
	minDistanceFromObstacles = map.getMinimumDistanceFromObstacles();

	directionNames[0] = "RIGHT";
	directionNames[1] = "UP";
	directionNames[2] = "LEFT";
	directionNames[3] = "DOWN";
	directionNames[4] = "INIT";

	// Need to wait between creating a new TF listener and calling lookupTransform()
	sleep(10.0);

	getCurrentPose();
	printCurrentPose();
	currentDirection = findCurrentDirection();
	//printCurrentDirection();
}

void Robot::startCoverage(const Path &coveragePath) {
	Logger::getInstance().write("Starting coverage...");

	for (vector<Cell>::const_iterator it = coveragePath.begin(); it != coveragePath.end(); it++) {
		Cell cell = *it;
		Cell nextCell = *(it + 1);
		moveToCell(cell, nextCell, coveragePath);
	}

	Logger::getInstance().write("Finished");
}

void Robot::getCurrentPose() {
    tf::StampedTransform transform;
    try {
    	listener.waitForTransform("/map", "/" + robotName + "/base_link", ros::Time(0), ros::Duration(60.0));
        listener.lookupTransform("/map", "/" + robotName + "/base_link", ros::Time(0), transform);
        currentPosition.first = transform.getOrigin().x();
        currentPosition.second = transform.getOrigin().y();
        currentAngle = tf::getYaw(transform.getRotation());

        convertCurrentPositionToCell();
    }
    catch (tf::TransformException & ex) {
        ROS_ERROR("%s", ex.what());
    }
}

Direction Robot::findCurrentDirection() const {
	double directionAngles[] = { 0, M_PI / 2, M_PI, -M_PI/2 };

	for (int i = 0; i < 4; i++) {
		if (abs(directionAngles[i] - currentAngle) < 0.01 || (i == 2 && abs(directionAngles[i] - (-M_PI) < 0.01))) {
			return (Direction)i;
		}
	}
	return OTHER;
}

// Converts the robot's current position to a grid cell
void Robot::convertCurrentPositionToCell() {
	currentCell = map.convertPositionToCell(currentPosition);
}

/*Position Robot::convertCellToPosition(Cell cell) {
	Position pos;
	//pos.first = (cell.second - cols / 2.0) * robotSize + robotSize / 2.0;
	//pos.second = (rows / 2.0 - cell.first) * robotSize - robotSize / 2.0;
	pos.first = cell.second * cellSize - map.getWidth() / 2.0 + cellSize / 2.0;
	pos.second = map.getHeight() / 2.0 - cell.first * cellSize - cellSize / 2.0;
	return pos;
}*/

// Shows the current position of the robot
void Robot::printCurrentPose() {
    std::stringstream logMessage;
	logMessage << "Current pose: (" << currentPosition.first << ", " << currentPosition.second << ", " << currentAngle << "), Cell: (" <<
		currentCell.first << ", " << currentCell.second << ")";
	Logger::getInstance().write(logMessage.str());
}

Cell Robot::getCurrentCell() const {
	return currentCell;
}

Direction Robot::getCurrentDirection() const {
	return currentDirection;
}

void Robot::printCurrentDirection() {
	std::stringstream logMessage;
	logMessage << "Current direction: " << directionNames[currentDirection];
	Logger::getInstance().write(logMessage.str());
}

void Robot::adjustTargetPositionNearWalls(Cell targetCell, Position &targetPosition, Direction direction) {
	// Make sure the robot doesn't get too close to the walls
	if (direction == UP && targetCell.first == 0)
		targetPosition.second -= minDistanceFromObstacles;
	else if (direction == DOWN && targetCell.first == rows - 1)
		targetPosition.second += minDistanceFromObstacles;
	else if (direction == LEFT && targetCell.second == 0)
		targetPosition.first += minDistanceFromObstacles;
	else if (direction == RIGHT && targetCell.second == cols - 1)
		targetPosition.first -= minDistanceFromObstacles;
}

void Robot::moveToCell(Cell targetCell, Cell nextCellAfterTarget, const Path &coveragePath) {
	// Change orientation of the robot if it needs to move to a new direction or if it has just started moving
	Direction newDirection = PathUtils::findDirection(currentCell, targetCell);

	// The target position is the center of the target cell
	Position targetPosition = map.convertCellToPosition(targetCell);
	adjustTargetPositionNearWalls(targetCell, targetPosition, newDirection);

	std::stringstream logMessage;
	logMessage << "Target cell: (" << targetCell.first << ", " << targetCell.second << "), position: (" << targetPosition.first << ", " << targetPosition.second << ")";
	Logger::getInstance().write(logMessage.str());

	if (newDirection != currentDirection || targetCell == *(coveragePath.begin()))
		rotateRobotToNewDirection(newDirection);
	currentDirection = newDirection;

	// If this is the last cell or the robot needs to turn after reaching the target cell, then
	// it needs to slow down at the end, so its position can more precisely match the target position
	if (targetCell == *(coveragePath.end() - 1)) {
		moveForwardToPreciseLocation(targetPosition);
	}
	else {
		Direction nextDirection = PathUtils::findDirection(targetCell, nextCellAfterTarget);
		if (nextDirection != newDirection) {
			moveForwardToPreciseLocation(targetPosition);
		}
		else {
			moveForwardToNextCell(targetCell);
		}
	}

	std::stringstream logMessage2;
	logMessage2 << "Reached cell: (" << targetCell.first << ", " << targetCell.second << ")";
	Logger::getInstance().write(logMessage2.str());

	// Stop the robot (in case the last command is still active)
	geometry_msgs::Twist stopCommand;
	cmdVelPublisher.publish(stopCommand);

	getCurrentPose();
	printCurrentPose();
}

void Robot::rotateRobotToNewDirection(Direction newDirection) {
	std::stringstream logMessage;
	logMessage << "Changing direction from " << directionNames[currentDirection] << " to " << directionNames[newDirection];
	Logger::getInstance().write(logMessage.str());

	double directionAngles[] = { 0, M_PI / 2, M_PI, -M_PI/2 };
	double targetAngle = directionAngles[newDirection];

	// Decide to which side to rotate - left or right by choosing the small angle
	bool turnLeft;
	if (targetAngle - currentAngle > 0 && targetAngle - currentAngle < M_PI)
		turnLeft = true;
	else if (targetAngle - currentAngle < -M_PI)
		turnLeft = true;
	else
		turnLeft = false;

	std::stringstream logMessage2;
	logMessage2 << "Target angle: " << targetAngle << ", " << (turnLeft ? "rotating left" : "rotating right");
	Logger::getInstance().write(logMessage2.str());

	long long startTime = GeneralUtils::getMSOfDay();

	geometry_msgs::Twist rotateCommand;
	rotateCommand.angular.z = turnLeft ? angularSpeed : -angularSpeed;

	// How fast will we update the robot's movement
	ros::Rate rate(20);

	// Rotate until the robot reaches the target angle
	while (ros::ok() && abs(currentAngle - targetAngle) > angularTolerance * 200) {

		// The robot can reach the LEFT direction from negative PI or positive PI
		if (newDirection == LEFT && (abs(currentAngle - (-M_PI)) <= angularTolerance * 200))
		    break;

		cmdVelPublisher.publish(rotateCommand);
		rate.sleep();
		getCurrentPose();
		printCurrentPose();
	}

	// Slow the speed near the target
	rotateCommand.angular.z = turnLeft ? 0.1 * angularSpeed : -0.1 * angularSpeed;
	while (ros::ok() && abs(currentAngle - targetAngle) > angularTolerance * 15) {
		// The robot can reach the LEFT direction from negative PI or positive PI
		if (newDirection == LEFT && (abs(currentAngle - (-M_PI)) <= angularTolerance * 15))
			break;

		cmdVelPublisher.publish(rotateCommand);
		rate.sleep();
		getCurrentPose();
		printCurrentPose();
	}

	// Further refine the angle
	rotateCommand.angular.z = turnLeft ? 0.005 * angularSpeed : -0.005 * angularSpeed;
	while (ros::ok() && abs(currentAngle - targetAngle) > angularTolerance) {
		// The robot can reach the LEFT direction from negative PI or positive PI
		if (newDirection == LEFT && (abs(currentAngle - (-M_PI)) <= angularTolerance))
			break;

		cmdVelPublisher.publish(rotateCommand);
		rate.sleep();
		getCurrentPose();
		printCurrentPose();
	}

	// The robot initially rotates in steps of 0.05 radians (however sometimes only a step of 0.1 can be detected), until it is within 0.12 from the target angle
	// Then it rotates in steps of 0.005 radians, until it is within 0.012 from the target angle
	// Finally, it rotates in steps of 0.0005 radians, until it is within 0.0012 from the target angle
	long long endTime = GeneralUtils::getMSOfDay();
	totalTurningTime += (endTime - startTime);
	numOfTurns++;

	if (currentDirection != OTHER) {
		if (abs(newDirection - currentDirection) == 2) {
			total180DegreesTurningTime += (endTime - startTime);
			numOf180DegreesTurns++;
		}
		else {
			total90DegreesTurningTime += (endTime - startTime);
			numOf90DegreesTurns++;
		}
	}
}

void Robot::moveForwardToNextCell(Cell targetCell) {
	stringstream logMessage;
	logMessage << "Going forward to target cell: (" << targetCell.first << ", " << targetCell.second << ")...";
	Logger::getInstance().write(logMessage.str());

	long long startTime = GeneralUtils::getMSOfDay();
	ros::Rate rate(20);

	geometry_msgs::Twist moveCommand;
	moveCommand.linear.x = highLinearSpeed;

	while (ros::ok() && (currentCell.first != targetCell.first || currentCell.second != targetCell.second)) {
		ROS_INFO("Sending move forward command.\n");

		cmdVelPublisher.publish(moveCommand);
		rate.sleep();
		getCurrentPose();
		printCurrentPose();
	}

	long long endTime = GeneralUtils::getMSOfDay();
	totalMovingForwardToCellTime += (endTime - startTime);
	numOfStepsForwardToCell++;
}

void Robot::moveForwardToPreciseLocation(Position targetPosition) {
	std::stringstream logMessage;
	logMessage << "Target position: (" << targetPosition.first << ", " << targetPosition.second << ")";
	Logger::getInstance().write(logMessage.str());

	bool moveHorizontally;
	if (currentDirection == LEFT || currentDirection == RIGHT)
		moveHorizontally = true;
	else
		moveHorizontally = false;

	double targetValue = moveHorizontally ? targetPosition.first : targetPosition.second;
	double currentValue = moveHorizontally ? currentPosition.first : currentPosition.second;

	stringstream logMessage2;
	logMessage2 << "Going forward to target position: (" << targetPosition.first << ", " << targetPosition.second << ")...";
	Logger::getInstance().write(logMessage2.str());

	long long startTime = GeneralUtils::getMSOfDay();
	ros::Rate rate(20);

	geometry_msgs::Twist moveCommand;
	moveCommand.linear.x = lowLinearSpeed;

	while (ros::ok() && abs(currentValue - targetValue) > 10 * linearTolerance) {
		cmdVelPublisher.publish(moveCommand);
		rate.sleep();
		getCurrentPose();
		printCurrentPose();
		currentValue = moveHorizontally ? currentPosition.first : currentPosition.second;
	}

	// Slow the speed near the target
	// The robot initially advances in steps of 0.05 meters, until it is within 0.1 from the target position
	// Then it advances in steps of 0.005 meters, until it is within 0.01 from the target position
	moveCommand.linear.x = 0.1 * lowLinearSpeed;

	while (ros::ok() && abs(currentValue - targetValue) > linearTolerance) {
		cmdVelPublisher.publish(moveCommand);
		rate.sleep();
		getCurrentPose();
		printCurrentPose();
		currentValue = moveHorizontally ? currentPosition.first : currentPosition.second;
	}

	long long endTime = GeneralUtils::getMSOfDay();
	totalMovingForwardToPositionTime += (endTime - startTime);
	numOfStepsForwardToPosition++;
}

float Robot::getAverageTurningTime() const {
	return (totalTurningTime / numOfTurns) / 1000;
}

float Robot::getAverage90DegreesTurningTime() const {
	return (total90DegreesTurningTime / numOf90DegreesTurns) / 1000;
}

float Robot::getAverage180DegreesTurningTime() const {
	return (total180DegreesTurningTime / numOf180DegreesTurns) / 1000;
}

float Robot::getAverageMovingForwardToCellTime() const {
	return (totalMovingForwardToCellTime / numOfStepsForwardToCell) / 1000;
}

float Robot::getAverageMovingForwardToPositionTime() const {
	return (totalMovingForwardToPositionTime / numOfStepsForwardToPosition) / 1000;
}

Robot::~Robot() {
}

