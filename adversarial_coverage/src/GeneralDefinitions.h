/*
 * General.h
 *
 *  Created on: Aug 16, 2015
 *      Author: roiyeho
 */

#ifndef GENERAL_H_
#define GENERAL_H_

#include <vector>
#include <stdlib.h>

enum Direction { RIGHT, UP, LEFT, DOWN, OTHER };

using namespace std;

typedef pair<double, double> Position;
typedef pair<double, double> Size;
typedef pair<int, int> Cell;
typedef vector<vector<float> > Grid;
typedef vector<Cell> Path;

#define FREE 0
#define OBSTACLE -1
#define float_correction 0.00001
#define DIRECTIONS_NUM 4

#endif /* GENERAL_H_ */
