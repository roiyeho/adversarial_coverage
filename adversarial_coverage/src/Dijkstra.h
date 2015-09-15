/*
 * Dijkstra.h
 *
 *  Created on: Aug 17, 2015
 *      Author: roiyeho
 */

#ifndef DIJKSTRA_H_
#define DIJKSTRA_H_

#include "Graph.h"
#include "TransitionCostFunction.h"
#include <set>

typedef set<pair<float, Node *> > NodesQueue;

class Dijkstra {
private:
	Graph *graph;
	Node *source;
	Direction initialDirection;
	TransitionCostFunction *transitionCostFunction;

	// Private methods
	void initNodes();
	void computeDistanceThroughNode(Node *node, Node *neighbor, NodesQueue &pqueue);

public:
	Dijkstra(Graph *graph, Node *source, Direction initialDirection, TransitionCostFunction *transitionCostFunction);
	void changeSource(Node *source, Direction currDirection);
	void run();
	Path getShortestPath(Node *target) const;
};

#endif /* DIJKSTRA_H_ */
