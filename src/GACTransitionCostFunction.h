/*
 * GACTransitionCostFunction.h
 *
 *  Created on: Aug 18, 2015
 *      Author: roiyeho
 */

#ifndef GACTRANSITIONCOSTFUNCTION_H_
#define GACTRANSITIONCOSTFUNCTION_H_

#include "TransitionCostFunction.h"
#include <cmath>

class GACTransitionCostFunction: public TransitionCostFunction {
private:
	double riskFactor;
	double minThreatProbability;
	double riskPenalty;

	//const static double costForMovingForward = 1;
	//const static double costForRotating90Degrees = 18.1;
	//const static double costForRotating180Degrees = 21.6;

public:
	GACTransitionCostFunction(double riskFactor, double minThreatProbability) :
		riskFactor(riskFactor), minThreatProbability(minThreatProbability)  {

		riskPenalty = -riskFactor / log(1 - minThreatProbability);
	}

	/*float operator()(const Node* from, const Node* to) {
		// Assign a cost to the destination node according to its number of neighbors
		float movementCost;

		int numOfNeighbors = to->neighbors.size();

		if (to->neighbors.size() == 1)
			movementCost = costForRotating180Degrees;
		else if (to->neighbors.size() == 2)
			movementCost = costForRotating90Degrees;
		else
			movementCost = costForMovingForward;

		if (to->threatProb > 0) {
			float cost = -riskPenalty * log(1 - to->threatProb) + movementCost;
			return cost;
		}
		return movementCost;
	}*/

	float operator()(const Node* from, const Node* to) {
		if (to->threatProb > 0) {
			float cost = -riskPenalty * log(1 - to->threatProb) + 1;
			return cost;
		}

		return 1.0; // take into account the path length
	}
};

#endif /* GACTRANSITIONCOSTFUNCTION_H_ */
