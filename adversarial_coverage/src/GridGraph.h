/*
 * GridGraph.h
 *
 *  Created on: Aug 16, 2015
 *      Author: roiyeho
 */

#ifndef GRIDGRAPH_H_
#define GRIDGRAPH_H_

#include "Graph.h"

class GridGraph: public Graph {
private:
	// Data members
	vector<vector<Node *> > nodesMatrix;
	int rows, cols;

	// Private methods
	void createNodes(const Grid &grid);
	void createNeighbors();

public:
	GridGraph(const Grid& grid);
	Node* getNode(int i, int j);
	virtual ~GridGraph();
};

#endif /* GRIDGRAPH_H_ */
