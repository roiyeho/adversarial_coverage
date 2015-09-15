/*
 * NodesComparer.h
 *
 *  Created on: Aug 17, 2015
 *      Author: roiyeho
 */

#ifndef NODESCOMPARER_H_
#define NODESCOMPARER_H_

#include "Node.h"

class NodesComparer {
public:

	bool operator()(Node& n1, Node& n2)
	{
		if (n1.distance < n2.distance)
			return true;

		return false;
	}
};

#endif /* NODESCOMPARER_H_ */
