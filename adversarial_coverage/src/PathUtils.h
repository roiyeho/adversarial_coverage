/*
 * PathUtils.h
 *
 *  Created on: Aug 30, 2015
 *      Author: roiyeho
 */

#ifndef PATHUTILS_H_
#define PATHUTILS_H_

#include "GeneralDefinitions.h"
#include "Map.h"

class PathUtils {
private:
	PathUtils();
public:
	static double getSurvivabilityProbability(const Path &coveragePath, const Map &map);
	static int getNumberOfThreatsVisits(const Path &coveragePath, const Map &map);
	static int getNumberOfTurns(Cell currCell, Direction currDirection, const Path &path);
	static void getNumberOfTurns(Cell currCell, Direction currDirection, const Path &path,
			int &numOfTurns, int &numOf90DegreesTurns, int &numOf180DegreesTurns);
	static Direction findDirection(Cell sourceCell, Cell targetCell);
};

#endif /* PATHUTILS_H_ */
