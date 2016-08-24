/*
 * Map.h
 *
 *  Created on: Aug 23, 2015
 *      Author: roiyeho
 */

#ifndef MAP_H_
#define MAP_H_

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <string>
#include "GeneralDefinitions.h"

class Map {
private:
	// Data members
	ros::NodeHandle nh;

	float mapResolution;
	float mapHeight;
	float mapWidth;
	int occupancyGridRows;
	int occupancyGridCols;

	double cellSize; // in meters
	float cellSizeRatio; // how many small cells are contained in one large cell
	double minDistanceFromObstacles;
	int obstacleInflation;

	Grid occupancyGrid;
	Grid grid; // the grid adjusted to the robot size
	int gridRows, gridCols;

	int freeCellsNum;
	int dangerousCellsNum;

	string threatsFilePath;
	float minThreatProbability;

	// Private methods
	bool getMap();
	void readMap(const nav_msgs::OccupancyGrid& msg);
	void adjustGridToRobotSize();
	double computeMinimumDistanceFromObstacles() const;
	double computeRotationTolerance() const;
	bool checkIfCellIsBlocked(int row, int col) const;
	float getThreatProbabilityInCell(int row, int col) const;
	bool readThreatsLocations();
	void addThreatToMap(float x, float y, float width, float length, float threatProbability);
	void getAccessibleCells(Cell startingCell, vector<Cell> &accessibleCells) const;
	bool checkIfThreatCanBePlacedAtCell(Cell cell, int horizontalSizeInCells, int verticalSizeInCells) const;
	void placeThreatAtCell(Cell cell, int horizontalSizeInCells, int verticalSizeInCells, float threatProbability);
	Position findDangerousAreaCenterPos(Cell topLeftCell, int dangerousAreaSizeInCells) const;
	void clearThreats();
	void createDangerousArea(Cell cell, int horizontalSizeInCells, int verticalSizeInCells, float threatProbability,
		vector<Cell> &accessibleCells, ofstream &threatsFile);

	void printGrid(const Grid &grid) const;
	void printGridToFile(const Grid &grid) const;
public:
	Map();
	const Grid &getGrid() const;
	double getMinimumDistanceFromObstacles() const;
	float getMinimumThreatProbability() const;
	int getNumberOfBlockedCells() const;
	int getNumberOfFreeCells() const;
	int getNumberOfDangerousCells() const;
	void getNumberOfAccessibleCells(Cell startingCell, int &accessibleCellsNum, int &dangerousAccessibleCellsNum) const;
	float getWidth() const;
	float getHeight() const;
	void getFreeCells(vector<Cell> &freeCells) const;
	void generateRandomThreatsFile(Cell startingCell, int threatsNum, int dangerousAreasNum, float threatProbability, string fileName);

	Cell convertPositionToCell(Position pos) const;
	Position convertCellToPosition(Cell cell) const;
	virtual ~Map();
};

#endif /* MAP_H_ */
