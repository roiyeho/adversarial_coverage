/*
 * PathUtils.cpp
 *
 *  Created on: Aug 30, 2015
 *      Author: roiyeho
 */

#include "PathUtils.h"

PathUtils::PathUtils() {
}

double PathUtils::getSurvivabilityProbability(const Path &path, const Map &map) {
	double prob = 1.0;

	const Grid &grid = map.getGrid();

	for (vector<Cell>::const_iterator it = path.begin(); it != path.end(); it++) {
		Cell cell = *it;

		float threatProb = grid[cell.first][cell.second];
		prob *= (1 - threatProb);
	}
	return prob;
}

/*int PathUtils::getNumberOfThreatsRevisits(const Path &path, const Map &map) {
	const Grid &grid = map.getGrid();

	int cellsWithThreats = 0;
	for (int i = 0; i < grid.size(); i++) {
		for (int j = 0; j < grid[0].size(); j++) {
			if (grid[i][j] > 0)
				cellsWithThreats++;
		}
	}

	int threatsVisits = 0;
	for (vector<Cell>::const_iterator it = path.begin(); it != path.end(); it++) {
		Cell cell = *it;

		if (grid[cell.first][cell.second] > 0)
			threatsVisits++;
	}
	return threatsVisits - cellsWithThreats;
}*/

int PathUtils::getNumberOfThreatsVisits(const Path &path, const Map &map) {
	const Grid &grid = map.getGrid();

	int threatsVisits = 0;
	for (vector<Cell>::const_iterator it = path.begin(); it != path.end(); it++) {
		Cell cell = *it;

		if (grid[cell.first][cell.second] > 0)
			threatsVisits++;
	}

	return threatsVisits;
}

/*int PathUtils::getNumberOfTurns(Cell currCell, Direction currDirection, const Path &path) {
	//Direction currDirection = RIGHT;
	int numOfTurns = 0;

	for (vector<Cell>::const_iterator it = path.begin(); it != path.end(); it++) {
		Cell nextCell = *it;

		Direction newDirection = findDirection(currCell, nextCell);
		if (newDirection != currDirection)
			numOfTurns++;
		currDirection = newDirection;
		currCell = nextCell;
	}
	return numOfTurns;
}*/

void PathUtils::getNumberOfTurns(Cell currCell, Direction currDirection, const Path &path,
		int &numOfTurns, int &numOf90DegreesTurns, int &numOf180DegreesTurns) {
	numOfTurns = 0; numOf90DegreesTurns = 0; numOf180DegreesTurns = 0;

	for (vector<Cell>::const_iterator it = path.begin(); it != path.end(); it++) {
		Cell nextCell = *it;

		Direction newDirection = findDirection(currCell, nextCell);
		if (newDirection != currDirection) {
			numOfTurns++;
			if (abs(newDirection - currDirection) == 2)
				numOf180DegreesTurns++;
			else
				numOf90DegreesTurns++;
		}
		currDirection = newDirection;
		currCell = nextCell;
	}
}

Direction PathUtils::findDirection(Cell sourceCell, Cell targetCell) {
	Direction direction;
	if (targetCell.first == sourceCell.first - 1) {
		direction = UP;
	}
	else if (targetCell.first == sourceCell.first + 1) {
		direction = DOWN;
	}
	else if (targetCell.second == sourceCell.second - 1) {
		direction = LEFT;
	}
	else if (targetCell.second == sourceCell.second + 1) {
		direction = RIGHT;
	}
	return direction;
}

