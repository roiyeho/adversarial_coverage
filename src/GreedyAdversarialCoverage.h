/*
 * GreedyAdversarialCoverage.h
 *
 *  Created on: Aug 17, 2015
 *      Author: roiyeho
 */

#ifndef GREEDYADVERSARIALCOVERAGE_H_
#define GREEDYADVERSARIALCOVERAGE_H_

#include <ros/ros.h>
#include "GeneralDefinitions.h"
#include "Map.h"
#include "GridGraph.h"
#include "Dijkstra.h"

class GreedyAdversarialCoverage {
private:
	// Data members
	const Map &map;
	Cell startingCell;
	Direction startingDirection;
	double riskFactor;

	// Private methods
	Node *findUnvisitedNodeWithMinimumDistance(const Graph &graph);
	Node *findUnvisitedNodeWithMinimumDistanceAndTurns(const Graph &graph,
			const Dijkstra &dijkstra, Node *currNode, Direction currDirection);
	void addPathToCoveragePath(Path &coveragePath, const Path &path);

public:
	GreedyAdversarialCoverage(ros::NodeHandle &nh, const Map &map, Cell startCell, Direction startingDirection);
	void buildCoveragePath(Path &coveragePath);

	virtual ~GreedyAdversarialCoverage();
};

#endif /* GREEDYADVERSARIALCOVERAGE_H_ */
