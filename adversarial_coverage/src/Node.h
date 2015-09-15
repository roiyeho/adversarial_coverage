/*
 * Node.h
 *
 *  Created on: Aug 16, 2015
 *      Author: roiyeho
 */

#ifndef NODE_H_
#define NODE_H_

#include "GeneralDefinitions.h"
#include <limits>

class Node {

public:
	Cell cell;
	float threatProb;
	bool visited;
	float distance; // an upper bound on the shortest path from the source node to this node
	float distancesByDirection[DIRECTIONS_NUM];  // distance from the source node using each of this node's 4 neighbors
	                                 	 	 	 // to reach this node
	Node *previous;
	int numOfTurns;

	vector<Node *> neighbors;

	Node(const Cell &cell, float threatProb) : cell(cell), threatProb(threatProb),
			visited(false), distance(numeric_limits<float>::infinity()), previous(NULL), numOfTurns(0) {
	}
};

#endif /* NODE_H_ */
