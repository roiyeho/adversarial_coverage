/*
 * GreedyAdversarialCoverage.cpp
 *
 *  Created on: Aug 17, 2015
 *      Author: roiyeho
 */

#include <limits>
#include "GreedyAdversarialCoverage.h"
#include "GACTransitionCostFunction.h"
#include "PathUtils.h"

GreedyAdversarialCoverage::GreedyAdversarialCoverage(ros::NodeHandle &nh, const Map &map, Cell startingCell, Direction startingDirection) :
	map(map), startingCell(startingCell), startingDirection(startingDirection) {
	nh.getParam("risk_factor", riskFactor);
}

void GreedyAdversarialCoverage::buildCoveragePath(Path &coveragePath) {
	// Create a graph from the given grid
	GridGraph graph(map.getGrid()); // TODO: Create graph only from accessible nodes

	GACTransitionCostFunction costFunction(riskFactor, map.getMinimumThreatProbability());
	Node *currNode = graph.getNode(startingCell.first, startingCell.second);

	Direction currDirection = startingDirection;
	Dijkstra dijkstra(&graph, currNode, currDirection, &costFunction);

	// Continue until there are no more accessible nodes in the graph
	while (currNode) {
		currNode->visited = true;

		// Run Dijkstra from current node
		dijkstra.changeSource(currNode, currDirection);
		dijkstra.run();

		// The greedy step - find an unvisited node with minimum distance and turns from current node
		Node *target = findUnvisitedNodeWithMinimumDistance(graph);
		//Node *target = findUnvisitedNodeWithMinimumDistanceAndTurns(graph, dijkstra, currNode, currDirection);

		if (target) {
			Path path = dijkstra.getShortestPath(target);

			// Add current path to the coverage path
			addPathToCoveragePath(coveragePath, path);

			if (path.size() == 1)
				currDirection = PathUtils::findDirection(currNode->cell, target->cell);
			else
				currDirection = PathUtils::findDirection(*(path.end() - 2), target->cell);
		}
		currNode = target;
	}
}

Node *GreedyAdversarialCoverage::findUnvisitedNodeWithMinimumDistance(const Graph &graph) {
	float minDistance = numeric_limits<float>::infinity();
	Node *minNode = NULL;

	for (vector<Node *>::const_iterator it = graph.nodesList.begin(); it != graph.nodesList.end(); it++) {
		Node *node = *it;
		if (!node->visited && node->distance < minDistance)	{
			minDistance = node->distance;
			minNode = node;
		}
	}
	return minNode;
}

/*Node *GreedyAdversarialCoverage::findUnvisitedNodeWithMinimumDistanceAndTurns(const Graph &graph,
		const Dijkstra &dijkstra, Node *currNode, Direction currDirection) {
	float minDistance = numeric_limits<float>::infinity();
	Node *minNode = NULL;

	for (vector<Node *>::const_iterator it = graph.nodesList.begin(); it != graph.nodesList.end(); it++) {
		Node *node = *it;
		if (!node->visited) {
			if (node->distance < minDistance) {
				minDistance = node->distance;
				Path path = dijkstra.getShortestPath(node);
				node->numOfTurns = PathUtils::getNumberOfTurns(currNode->cell, currDirection, path);
				minNode = node;
			}
			// If the distance is equal, check the number of turns
			else if (node->distance != numeric_limits<float>::infinity() && node->distance == minDistance) {
				Path path = dijkstra.getShortestPath(node);

				node->numOfTurns = PathUtils::getNumberOfTurns(currNode->cell, currDirection, path);
				if (node->numOfTurns < minNode->numOfTurns) {
					minNode = node;
				}
			}
		}
	}
	return minNode;
}*/

void GreedyAdversarialCoverage::addPathToCoveragePath(Path &coveragePath, const Path &path) {
	for (vector<Cell>::const_iterator it = path.begin(); it != path.end(); it++) {
		Cell cell = *it;
		// TODO: Count number of visits in each cell
		coveragePath.push_back(cell);
	}
}

GreedyAdversarialCoverage::~GreedyAdversarialCoverage() {

}

