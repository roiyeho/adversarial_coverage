/*
 * GeneralUtils.h
 *
 *  Created on: Aug 31, 2015
 *      Author: roiyeho
 */

#ifndef GENERALUTILS_H_
#define GENERALUTILS_H_

#include <string>
using namespace std;

class GeneralUtils {
private:
	GeneralUtils() { }
public:
	static string getDateString();
	static int randInRange(int min, int max);
	static long long getMSOfDay();
};

#endif /* GENERALUTILS_H_ */
