/*
 * Map.cpp
 *
 *  Created on: Aug 23, 2015
 *      Author: roiyeho
 */

#include "Map.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <limits>
#include <queue>
#include <cmath>
#include <ros/package.h>
#include "GridGraph.h"
#include "Logger.h"
#include "GeneralUtils.h"

Map::Map() : freeCellsNum(0), dangerousCellsNum(0) {
	nh.getParam("threats_file", threatsFilePath);
	nh.getParam("simulation_mode", simulationMode);

	// Read the map
	if (!getMap())
		exit(1);

	readThreatsLocations();
	//Logger::getInstance().printGrid(occupancyGrid);

	adjustGridToRobotSize();
	Logger::getInstance().write("Adjusted grid: ");
	Logger::getInstance().printGrid(grid, true);
}

// Requests the map from the static map_service and initializes the occupancy grid
bool Map::getMap() {
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;

    while (!ros::service::waitForService("static_map", ros::Duration(3.0))) {
         ROS_INFO("Waiting for service static_map to become available");
    }

    ROS_INFO("Requesting the map...");
    ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");

    if (mapClient.call(req, res)) {
        readMap(res.map);
        return true;
    }
    else {
        ROS_ERROR("Failed to call map service");
        return false;
    }
}

// Initializes the occupancy grid. The value of each cell in the occupancy grid represents the probability of
// a threat existing in that cell (between 0-100).
void Map::readMap(const nav_msgs::OccupancyGrid& map) {
	stringstream logMessage;
	logMessage << "Received a " << map.info.width << " X " << map.info.height << " map, resolution: "
			<< map.info.resolution << " m/px";
	Logger::getInstance().write(logMessage.str());

    int rows = map.info.height;
    int cols = map.info.width;
    mapResolution = map.info.resolution;
    mapHeight = rows * mapResolution;
    mapWidth = cols * mapResolution;

    //ROS_INFO("Creating occupancy grid of %d X %d\n", rows, cols);

    // Dynamically resize the occupancy grid
    occupancyGrid.resize(rows);
    for (int i = 0; i < rows; i++) {
        occupancyGrid[i].resize(cols);
    }

    // Read the map into the occupancy grid (change the order of the rows from top to bottom)
    int currCell = 0;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
        	if (map.data[currCell] == 100 || map.data[currCell] == -1) // occupied or unknown cell
        		occupancyGrid[rows - 1 - i][j] = OBSTACLE;
			else
				//occupancyGrid[rows - 1 - i][j] = map.data[currCell];
				occupancyGrid[rows - 1 - i][j] = FREE;
			currCell++;
        }
    }
}

void Map::adjustGridToRobotSize() {
	double robotWidth, robotLength;
	nh.getParam("robot_width", robotWidth);
	nh.getParam("robot_length", robotLength);

	cellSize = max(robotWidth, robotLength);
	//nh.getParam("robot_size", robotSize);

	int origGridRows = occupancyGrid.size();
	int origGridCols = occupancyGrid[0].size();

	// Compute how many small cells are contained in one large cell
	cellSizeRatio = cellSize / mapResolution;

	minDistanceFromObstacles = computeMinimumDistanceFromObstacles();
	obstacleInflation = ceil(minDistanceFromObstacles / mapResolution);

	gridRows = (int)(mapHeight / cellSize);
	gridCols = (int)(mapWidth / cellSize);

    // Dynamically resize the coarse grid
	grid.resize(gridRows);
    for (int i = 0; i < gridRows; i++) {
    	grid[i].resize(gridCols);
    }

    // For each cell in the adjusted grid, check if one of its subcells in the occupancy grid is blocked
    // (taking into account obstacle inflation). If the cell is free, its threat probability is the maximum
    // of the threat probabilities in its subcells.
    for (int i = 0; i < gridRows; i++) {
		for (int j = 0; j < gridCols; j++) {
    		if (checkIfCellIsBlocked(i, j)) {
    			grid[i][j] = OBSTACLE;
    		}
    		else {
    			freeCellsNum++;
    			grid[i][j] = getThreatProbabilityInCell(i, j);
    			if (grid[i][j] > 0)
    				dangerousCellsNum++;
    		}
    	}
    }
}

double Map::computeMinimumDistanceFromObstacles() const {
	return computeRotationTolerance() + 0.03; // TODO: Find appropriate constant
	// This constant needs to take into account the inability of the robot to keep a precise rotation angle for long periods of movement
}

double Map::computeRotationTolerance() const {
	// cx, cy - center of square coordinates
	// x, y - coordinates of a corner point of the square
	// theta is the angle of rotation
	double cx = cellSize / 2;
	double cy = cellSize / 2;

	// Take the coordinates of the upper-right corner
	double x = cellSize;
	double y = cellSize;

	// Translate point to origin
	double tempX = x - cx;
	double tempY = y - cy;

	double theta = M_PI / 4; // a rotation of 45 degrees reaches the highest point

	// now apply rotation
	double rotatedX = tempX * cos(theta) - tempY * sin(theta);
	double rotatedY = tempX * sin(theta) + tempY * cos(theta);

	// translate back
	x = rotatedX + cx;
	y = rotatedY + cy;

	return y - cellSize;
}

bool Map::checkIfCellIsBlocked(int row, int col) const {
	int origGridRows = occupancyGrid.size();
	int origGridCols = occupancyGrid[0].size();

	int startRow = max((int)(row * cellSizeRatio - obstacleInflation), 0);
	int endRow = min((int)(ceil((row + 1) * cellSizeRatio + obstacleInflation)), origGridRows);
	int startCol = max((int)(col * cellSizeRatio - obstacleInflation), 0);
	int endCol = min((int)(ceil((col + 1) * cellSizeRatio + obstacleInflation)), origGridCols);

	for (int i = startRow; i < endRow; i++) {
		for (int j = startCol; j < endCol; j++) {
			if (occupancyGrid[i][j] == OBSTACLE)
				return true;
		}
	}
	return false;
}

float Map::getThreatProbabilityInCell(int row, int col) const {
	float maxThreatProb = 0;
	int occupancyGridRow = row * (cellSizeRatio + float_correction);
	int occupancyGridCol = col * (cellSizeRatio + float_correction);

	for (int k = 0; k < cellSizeRatio; k++) {
		for (int m = 0; m < cellSizeRatio; m++) {
			float threatProb = occupancyGrid[occupancyGridRow + k][occupancyGridCol + m];
			if (threatProb > maxThreatProb)
				maxThreatProb = threatProb;
		}
	}

	return maxThreatProb;
}

bool Map::readThreatsLocations() {
	ifstream threatsFile(threatsFilePath.c_str());
	string line;
	minThreatProbability = numeric_limits<float>::infinity();

	while (getline(threatsFile, line))
	{
	    std::istringstream iss(line);
	    string threatLabel, poseLabel, bracket, sizeLabel, probLabel;
	    float x, y, z, theta, width, length, height, threatProbability;
	    if (!(iss >> threatLabel >> poseLabel >> bracket >> x >> y >> z >> theta
	    		>> bracket >> sizeLabel >> bracket >> width >> length >> height
	    		>> bracket >> probLabel >> threatProbability))  {
	    	ROS_ERROR("Error in threats file");
			return false;
	    }
	    addThreatToMap(x, y, width, length, threatProbability);
	    if (threatProbability < minThreatProbability)
	    	minThreatProbability = threatProbability;
	}
	return true;
}

void Map::addThreatToMap(float x, float y, float width, float length, float threatProbability) {
	int rows = occupancyGrid.size();
	int cols = occupancyGrid[0].size();

	// Handle imprecision of floating numbers
	int startRow = (int)(rows / 2 - (y + length / 2) / mapResolution - float_correction);
	int endRow = (int)ceil(rows / 2 - ((y + float_correction) - length / 2) / mapResolution);
	int startCol = (int)(cols / 2 + ((x + float_correction) - width / 2) / mapResolution);
	int endCol = (int)ceil(cols / 2 + ((x - float_correction) + width / 2) / mapResolution);

	for (int i = startRow; i < endRow; i++) {
		for (int j = startCol; j < endCol; j++) {
			// Make sure there is no obstacle first
			if (occupancyGrid[i][j] != OBSTACLE)
				occupancyGrid[i][j] = threatProbability;
		}
	}
}

// Converts the robot's current position to a grid cell
Cell Map::convertPositionToCell(Position pos) const {
	Cell cell;

	// In the simulator, the origin (0,0) is in the center of the map, while in the robot the origin (0,0) is in the lower-left corner
	if (simulationMode) {
		cell.first = (mapHeight / 2.0 - pos.second) / cellSize;
		cell.second = (mapWidth / 2.0 + pos.first) / cellSize;
	}
	else {
		cell.first = (mapHeight - pos.second) / cellSize;
		cell.second = pos.first / cellSize;
	}
	return cell;
}

// Return the center point of the cell
Position Map::convertCellToPosition(Cell cell) const {

	Position pos;
	// In the simulator, the origin (0,0) is in the center of the map, while in the robot the origin (0,0) is in the lower-left corner
	if (simulationMode) {
		pos.first = cell.second * cellSize - mapWidth / 2.0 + cellSize / 2.0;
		pos.second = mapHeight / 2.0 - cell.first * cellSize - cellSize / 2.0;
        } 
	else {
		pos.first = cell.second * cellSize + cellSize / 2.0;
		pos.second = mapHeight - cell.first * cellSize - cellSize / 2.0;
 	}
	return pos;
}

void Map::getFreeCells(vector<Cell> &freeCells) const {
	for (int i = 0; i < gridRows; i++) {
		for (int j = 0; j < gridCols; j++) {
			if (grid[i][j] != OBSTACLE)
				freeCells.push_back(Cell(i, j));
		}
	}
}

Position Map::findDangerousAreaCenterPos(Cell topLeftCell, int dangerousAreaSizeInCells) const {
	Position pos = convertCellToPosition(topLeftCell);
	pos.first += 0.5 * (dangerousAreaSizeInCells - 1) * cellSize;
	pos.second -= 0.5 * (dangerousAreaSizeInCells - 1) * cellSize;
	return pos;
}

void Map::clearThreats() {
	for (int i = 0; i < gridRows; i++) {
		for (int j = 0; j < gridCols; j++) {
			if (grid[i][j] != OBSTACLE)
				grid[i][j] = FREE;
		}
	}
}

/*void Map::createDangerousAreas(int dangerousAreasNum, int dangerousAreaSizeInCells, float threatProbability,
		vector<Cell> &accessibleCells, ofstream &threatsFile) {
	for (int i = 0; i < dangerousAreasNum; i++) {
		int cellNum = GeneralUtils::randInRange(0, accessibleCells.size() - 1);
		Cell cell = accessibleCells[cellNum];

		if (checkIfThreatCanBePlacedAtCell(cell, dangerousAreaSizeInCells)) {
			accessibleCells.erase(accessibleCells.begin() + cellNum - 1);

			placeThreatAtCell(cell, dangerousAreaSizeInCells, threatProbability);

			// A grid cell can occupy a non-integer number of small occupancy grid cells
			// In such case one small dangerous cell can belong to two different cells
			// Thus, in order to precisely control the number of dangerous big cells, we want our
			// randomly chosen cell to include only an integer number of small cells
			int leftSubcell = (ceil)(cell.second * cellSizeRatio);
			int rightSubcell = (int)((cell.second + dangerousAreaSizeInCells) * cellSizeRatio);
			float left = leftSubcell * mapResolution - mapWidth / 2.0;
			float right = rightSubcell * mapResolution - mapWidth / 2.0;
			float horizontalSize = right - left;
			float x = (left + right) / 2;

			int topSubcell = (ceil)(cell.first * cellSizeRatio);
			int bottomSubcell = (int)((cell.first + dangerousAreaSizeInCells) * cellSizeRatio);
			float top = mapHeight / 2 - topSubcell * mapResolution;
			float bottom = mapHeight / 2 - bottomSubcell * mapResolution;
			float verticalSize = top - bottom;
			float y = (top + bottom) / 2;

			float dangerousAreaSize = min(horizontalSize, verticalSize) - 0.001;

			// Write the threat info to the file
			// Format: threat( pose [ -7.65 7.65 0 0 ] size [ 0.7 0.7 0 ] prob 0.01 color "red")
			threatsFile << "threat( pose [ " << x << " " << y
					<< " 0 0 ] size [ " << dangerousAreaSize << " " << dangerousAreaSize
					<< " 0 ] prob " << threatProbability
					<< " color " << "\"red\")" << endl;
		}
		else { // Remove cells that cannot be used
			accessibleCells.erase(accessibleCells.begin() + cellNum - 1);
			i--;
		}
	}
}*/

void Map::createDangerousArea(Cell cell, int horizontalSizeInCells, int verticalSizeInCells, float threatProbability,
		vector<Cell> &accessibleCells, ofstream &threatsFile) {

	// Save the threat in the grid
	placeThreatAtCell(cell, horizontalSizeInCells, verticalSizeInCells, threatProbability);

	// A grid cell can occupy a non-integer number of small occupancy grid cells
	// In such case one small dangerous cell can belong to two different cells
	// Thus, in order to precisely control the number of dangerous big cells, we want our
	// randomly chosen cell to include only an integer number of small cells
	int leftSubcell = (ceil)(cell.second * cellSizeRatio);
	int rightSubcell = (int)((cell.second + horizontalSizeInCells) * cellSizeRatio);
	float left = leftSubcell * mapResolution - mapWidth / 2.0;
	float right = rightSubcell * mapResolution - mapWidth / 2.0;
	float horizontalSize = right - left - 0.001;
	float x = (left + right) / 2;

	int topSubcell = (ceil)(cell.first * cellSizeRatio);
	int bottomSubcell = (int)((cell.first + verticalSizeInCells) * cellSizeRatio);
	float top = mapHeight / 2 - topSubcell * mapResolution;
	float bottom = mapHeight / 2 - bottomSubcell * mapResolution;
	float verticalSize = top - bottom - 0.001;
	float y = (top + bottom) / 2;

	// Write the threat info to the file
	// Format: threat( pose [ -7.65 7.65 0 0 ] size [ 0.7 0.7 0 ] prob 0.01 color "red")
	threatsFile << "threat( pose [ " << x << " " << y
			<< " 0 0 ] size [ " << horizontalSize << " " << verticalSize
			<< " 0 ] prob " << threatProbability
			<< " color " << "\"red\")" << endl;
}

void Map::generateRandomThreatsFile(Cell startingCell, int threatsNum, int dangerousAreasNum, float threatProbability, string fileName) {
	clearThreats();

	// Create a new threats file
	string path = ros::package::getPath("adversarial_coverage") + "/worlds/" + fileName + ".inc";
	ofstream threatsFile(path.c_str());

	threatsFile << fixed << setprecision(3);

	vector<Cell> accessibleCells;
	getAccessibleCells(startingCell, accessibleCells);
	int areaSize = sqrt(threatsNum / dangerousAreasNum);

	int threatsPlaced = 0;

	while (threatsPlaced < threatsNum) {
		int cellNum = GeneralUtils::randInRange(0, accessibleCells.size() - 1);
		Cell cell = accessibleCells[cellNum];
		accessibleCells.erase(accessibleCells.begin() + cellNum - 1);

		int threatsLeft = threatsNum - threatsPlaced;

		int horizontalSizeInCells = GeneralUtils::randInRange(1, min(areaSize * areaSize, threatsLeft));
		int verticalSizeInCells = GeneralUtils::randInRange(1, min((areaSize * areaSize) / horizontalSizeInCells * 2, threatsLeft / horizontalSizeInCells));

		if (checkIfThreatCanBePlacedAtCell(cell, horizontalSizeInCells, verticalSizeInCells)) {
			createDangerousArea(cell, horizontalSizeInCells, verticalSizeInCells, threatProbability, accessibleCells, threatsFile);
			threatsPlaced += horizontalSizeInCells * verticalSizeInCells;
		}
	}

	// Place the leftovers as single cell threats
	/*int leftOvers = threatsNum - threatsPlaced;
	for (int i = 0; i < leftOvers; i++) {
		int cellNum = GeneralUtils::randInRange(0, accessibleCells.size() - 1);
		Cell cell = accessibleCells[cellNum];
		accessibleCells.erase(accessibleCells.begin() + cellNum - 1);

		if (checkIfThreatCanBePlacedAtCell(cell, 1, 1))
			createDangerousArea(cell, 1, 1, threatProbability, accessibleCells, threatsFile);
		else
			i--;
	}*/

	threatsFile.close();
}

bool Map::checkIfThreatCanBePlacedAtCell(Cell cell, int horizontalSizeInCells, int verticalSizeInCells) const {
	// Assuming that the threat area will start from the upper-left corner of the chosen cell
	int startRow = cell.first;
	int endRow = cell.first + verticalSizeInCells - 1;
	if (endRow > gridRows - 1)
		return false;
	int startCol = cell.second;
	int endCol = cell.second + horizontalSizeInCells - 1;
	if (endCol > gridCols - 1)
		return false;

	for (int i = startRow; i <= endRow; i++) {
		for (int j = startCol; j <= endCol; j++) {
			// Make sure there is no obstacle or an already placed threat in that area
			if (grid[i][j] != FREE)
				return false;
		}
	}
	return true;
}

void Map::placeThreatAtCell(Cell cell, int horizontalSizeInCells, int verticalSizeInCells, float threatProbability) {
	int startRow = cell.first;
	int endRow = cell.first + verticalSizeInCells - 1;
	int startCol = cell.second;
	int endCol = cell.second + horizontalSizeInCells - 1;

	for (int i = startRow; i <= endRow; i++) {
		for (int j = startCol; j <= endCol; j++) {
			grid[i][j] = threatProbability;
		}
	}
}

const Grid &Map::getGrid() const {
	return grid;
}

double Map::getMinimumDistanceFromObstacles() const {
	return minDistanceFromObstacles;
}

float Map::getMinimumThreatProbability() const {
	return minThreatProbability;
}

int Map::getNumberOfFreeCells() const {
	return freeCellsNum;
}

int Map::getNumberOfBlockedCells() const {
	return (gridRows * gridCols) - freeCellsNum;
}

int Map::getNumberOfDangerousCells() const {
	return dangerousCellsNum;
}

void Map::getNumberOfAccessibleCells(Cell startingCell, int &accessibleCellsNum, int &dangerousAccessibleCellsNum) const {
	accessibleCellsNum = 0;
	dangerousAccessibleCellsNum = 0;

	vector<Cell> accessibleCells;
	getAccessibleCells(startingCell, accessibleCells);

	for (vector<Cell>::const_iterator it = accessibleCells.begin(); it != accessibleCells.end(); it++) {
		Cell currCell = *it;

		accessibleCellsNum++;
		if (grid[currCell.first][currCell.second] > 0)
			dangerousAccessibleCellsNum++;
	}
}

void Map::getAccessibleCells(Cell startingCell, vector<Cell> &accessibleCells) const {
	// Create a graph from the grid
	GridGraph graph(grid);

	// Initialize a queue of reachable nodes
	queue<Node *> openQueue;
	Node *startingNode = graph.getNode(startingCell.first, startingCell.second);
	openQueue.push(startingNode);
	startingNode->visited = true;

	// Run BFS to find all reachable cells
	while (!openQueue.empty()) {
		Node *currNode = openQueue.front();
		openQueue.pop();

		accessibleCells.push_back(currNode->cell);
		for (vector<Node *>::iterator it = currNode->neighbors.begin(); it != currNode->neighbors.end(); it++) {
			Node *neighbor = *it;

			if (!neighbor->visited) {
				openQueue.push(neighbor);
				neighbor->visited = true;
			}
		}
	}
}

float Map::getHeight() const {
	return mapHeight;
}

float Map::getWidth() const {
	return mapWidth;
}

Map::~Map() {
}

