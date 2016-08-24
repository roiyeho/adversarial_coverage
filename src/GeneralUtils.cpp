/*
 * GeneralUtils.cpp
 *
 *  Created on: Aug 31, 2015
 *      Author: roiyeho
 */

#include "GeneralUtils.h"
#include <ctime>
#include <sstream>
#include <stdlib.h>
#include <sys/time.h>

string GeneralUtils::getDateString() {
   time_t t = time(0); // get time now
   struct tm* now = localtime(&t);

   stringstream s;
   s   << (now->tm_year + 1900) << '-'
	   << (now->tm_mon + 1) << '-'
	   << now->tm_mday << '-'
	   << now->tm_hour << '-'
	   << now->tm_min << '-'
	   << now->tm_sec;

   return s.str();
}

int GeneralUtils::randInRange(int min, int max) {
	return min + (rand() % (max - min + 1));
}

long long GeneralUtils::getMSOfDay()
{
   struct timeval tv;
   gettimeofday(&tv, NULL);
   return (long)tv.tv_sec*1000 + tv.tv_usec/1000;
}
