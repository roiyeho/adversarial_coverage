/*
 * Logger.h
 *
 *  Created on: Sep 6, 2015
 *      Author: roiyeho
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "GeneralDefinitions.h"
#include <string>
#include <fstream>

class Logger {
private:
	// Make it a singleton
	Logger();
	Logger(const Logger&);
	void operator=(const Logger&);
	virtual ~Logger();

	bool writeToLogFile;
	string logFilePath;
	ofstream logFile;

public:
	static Logger& getInstance();
	static void test();
	void write(const string &msg);
	void printGrid(const Grid &grid, bool printToConsole = false);
	void printPath(const Path &path);
};

#endif /* LOGGER_H_ */

