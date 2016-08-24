/*
 * Dijkstra.cpp
 *
 *  Created on: Aug 17, 2015
 *      Author: roiyeho
 */

#include "Dijkstra.h"
#include <limits>
#include "PathUtils.h"

Dijkstra::Dijkstra(Graph *graph, Node *source, Direction initialDirection, TransitionCostFunction *transitionCostFunction) : graph(graph), source(source),
	initialDirection(initialDirection), transitionCostFunction(transitionCostFunction) {

}

void Dijkstra::changeSource(Node *source, Direction initialDirection) {
	this->source = source;
	this->initialDirection = initialDirection;
}

/*void Dijkstra::run() {
	initNodes();

	set<pair<float, Node *> > pqueue; // priority queue in C++ doesn't support a decreaseKey function
	// Thus, we use a self-balancing binary search tree (std::set), which should bound time complexity by O(E log V).

	source->distance = 0;
	pqueue.insert(make_pair(source->distance, source));

	while (!pqueue.empty()) {
		Node *node = pqueue.begin()->second;
		pqueue.erase(pqueue.begin()); // Because set containers keep their elements sorted at all times, begin points to the first element following the container's sorting criterion

		// Visit each edge exiting current node
		for (vector<Node *>::iterator it = node->neighbors.begin(); it != node->neighbors.end(); it++) {
			Node *neighbor = *it;

			float cost = (*transitionCostFunction)(node, neighbor);
			float distanceThroughNode = node->distance + cost;

			// Relax the edge to this neighbor
			if (distanceThroughNode < neighbor->distance) {
				pqueue.erase(make_pair(neighbor->distance, neighbor));
				neighbor->distance = distanceThroughNode;
				neighbor->previous = node;
				pqueue.insert(make_pair(neighbor->distance, neighbor));
			}
		}
	}
}*/

void Dijkstra::run() {
	initNodes();

	NodesQueue pqueue; // priority queue in C++ doesn't support a decreaseKey function
	// Thus, we use a self-balancing binary search tree (std::set), which should bound time complexity by O(E log V).

	source->distance = 0;
	for (int i = 0; i < DIRECTIONS_NUM; i++) {
		source->distancesByDirection[i] = 0;
	}
	pqueue.insert(make_pair(source->distance, source));

	while (!pqueue.empty()) {
		Node *node = pqueue.begin()->second;
		pqueue.erase(pqueue.begin()); // Because set containers keep their elements sorted at all times, begin points to the first element following the container's sorting criterion

		// Visit each edge exiting current node
		for (vector<Node *>::iterator it = node->neighbors.begin(); it != node->neighbors.end(); it++) {
			Node *neighbor = *it;

			/* float cost = (*transitionCostFunction)(node, neighbor);
			float distanceThroughNode = node->distance + cost;
			// Relax the edge to this neighbor
			if (distanceThroughNode < neighbor->distance) {
				pqueue.erase(make_pair(neighbor->distance, neighbor));
				neighbor->distance = distanceThroughNode;
				neighbor->previous = node;
				pqueue.insert(make_pair(neighbor->distance, neighbor));
			}*/

			computeDistanceThroughNode(node, neighbor, pqueue);
		}
	}
}

void Dijkstra::computeDistanceThroughNode(Node *node, Node *neighbor, NodesQueue &pqueue) {
	Direction direction = PathUtils::findDirection(node->cell, neighbor->cell);
	float distancesThroughNode[DIRECTIONS_NUM];
	const float penaltyForTurning90Degrees = 17.4;
	const float penaltyForTurning180Degrees = 20.9;

	if (node == source) {
		for (int i = 0; i < DIRECTIONS_NUM; i++) {
			if (abs(initialDirection - i) == 2)
				distancesThroughNode[i] = (*transitionCostFunction)(node, neighbor) + penaltyForTurning180Degrees;
			else
				distancesThroughNode[i] = (*transitionCostFunction)(node, neighbor) + penaltyForTurning90Degrees;
		}
		if (direction == initialDirection) {
			distancesThroughNode[direction] = (*transitionCostFunction)(node, neighbor);
		}
	}
	else {
		for (int i = 0; i < DIRECTIONS_NUM; i++) {
			if (i == direction) {
				distancesThroughNode[i] = node->distancesByDirection[direction] +
					(*transitionCostFunction)(node, neighbor);
			}
			else { // Add a penalty for making a turn
				if (abs(direction - i) == 2)
					distancesThroughNode[i] = node->distancesByDirection[i] +
						(*transitionCostFunction)(node, neighbor) + penaltyForTurning180Degrees;
				else
					distancesThroughNode[i] = node->distancesByDirection[i] +
						(*transitionCostFunction)(node, neighbor) + penaltyForTurning90Degrees;
			}
		}
	}

	// Take the minimum of all the distances
	float minDistance = numeric_limits<float>::infinity();
	for (int i = 0; i < DIRECTIONS_NUM; i++) {
		if (distancesThroughNode[i] < minDistance)
			minDistance = distancesThroughNode[i];
	}

	// Check if we need to update the distance from that direction
	if (minDistance < neighbor->distancesByDirection[direction]) {
		neighbor->distancesByDirection[direction] = minDistance;

		// Return this neighbor to the queue if needed
		bool isNeighborInQueue = (pqueue.find(make_pair(neighbor->distance, neighbor)) != pqueue.end());
		if (!isNeighborInQueue)
			pqueue.insert(make_pair(neighbor->distance, neighbor));

		// Now check if we need to update the general distance
		for (int i = 0; i < DIRECTIONS_NUM; i++) {
			if (neighbor->distancesByDirection[i] < minDistance) {
				minDistance = neighbor->distancesByDirection[i];
			}
		}

		if (minDistance < neighbor->distance) {
			// Update the path to this neighbor
			pqueue.erase(make_pair(neighbor->distance, neighbor));
			neighbor->distance = minDistance;
			neighbor->previous = node;
			pqueue.insert(make_pair(neighbor->distance, neighbor));
		}
	}
}

void Dijkstra::initNodes() {
	for (vector<Node *>::iterator it = graph->nodesList.begin(); it != graph->nodesList.end(); it++) {
		Node *node = *it;

		node->distance = numeric_limits<float>::infinity();
		for (int i = 0; i < DIRECTIONS_NUM; i++) {
			node->distancesByDirection[i] = numeric_limits<float>::infinity();
		}
		node->previous = NULL;
	}
}

/*void Dijkstra::initNodes() {
	for (vector<Node *>::iterator it = graph->nodesList.begin(); it != graph->nodesList.end(); it++) {
		Node *node = *it;

		node->distance = numeric_limits<float>::infinity();
		node->previous = NULL;
	}
}*/

Path Dijkstra::getShortestPath(Node *target) const {
	Path path;
	Node *currNode = target;

	while (currNode->previous != NULL) {
		path.insert(path.begin(), currNode->cell);
		currNode = currNode->previous;
	}
	return path;
}

