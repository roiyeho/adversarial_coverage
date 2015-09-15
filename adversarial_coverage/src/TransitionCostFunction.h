/*
 * TransitionCostFunction.h
 *
 *  Created on: Aug 17, 2015
 *      Author: roiyeho
 */

#ifndef TRANSITIONCOSTFUNCTION_H_
#define TRANSITIONCOSTFUNCTION_H_

class TransitionCostFunction {
public:
	virtual float operator()(const Node* from, const Node* to) = 0;
	virtual ~TransitionCostFunction() { }
};

#endif /* TRANSITIONCOSTFUNCTION_H_ */
