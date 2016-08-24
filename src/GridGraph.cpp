/*
 * GridGraph.cpp
 *
 *  Created on: Aug 16, 2015
 *      Author: roiyeho
 */

#include "GridGraph.h"

GridGraph::GridGraph(const Grid &grid) {
	rows = grid.size();
	cols = (grid[0]).size();
	nodesMatrix.resize(rows);
	for (int i = 0; i < rows; i++)
		nodesMatrix[i].resize(cols);

	createNodes(grid);
	createNeighbors();
}

void GridGraph::createNodes(const Grid &grid) {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			if (grid[i][j] != OBSTACLE) {
				Node *node = new Node(Cell(i, j), grid[i][j]);
				nodesList.push_back(node);
				nodesMatrix[i][j] = node;
			}
		}
	}
}

void GridGraph::createNeighbors() {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			Node *node = nodesMatrix[i][j];

			// Check that this is not an obstacle
			if (node) {
				if (i > 0) {
					Node *neighbor = nodesMatrix[i - 1][j];
					if (neighbor)
						node->neighbors.push_back(neighbor);
				}
				if (j > 0) {
					Node *neighbor = nodesMatrix[i][j - 1];
					if (neighbor)
						node->neighbors.push_back(neighbor);
				}
				if (i < rows - 1) {
					Node *neighbor = nodesMatrix[i + 1][j];
					if (neighbor)
						node->neighbors.push_back(neighbor);
				}
				if (j < cols - 1) {
					Node *neighbor = nodesMatrix[i][j + 1];
					if (neighbor)
						node->neighbors.push_back(neighbor);
				}
			}
		}
	}
}

Node* GridGraph::getNode(int i, int j) {
	return nodesMatrix[i][j];
}

GridGraph::~GridGraph() {
	for (vector<Node *>::iterator it = nodesList.begin(); it != nodesList.end(); it++)
		delete *it;
}

