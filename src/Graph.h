/*
 * Graph.h
 *
 *  Created on: Aug 16, 2015
 *      Author: roiyeho
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include "GeneralDefinitions.h"
#include "Node.h"

class Graph {
public:
	vector<Node *> nodesList;

	Graph() { }
	virtual ~Graph() { }
};

#endif /* GRAPH_H_ */
