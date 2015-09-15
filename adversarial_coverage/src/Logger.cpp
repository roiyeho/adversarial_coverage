/*
 * Logger.cpp
 *
 *  Created on: Sep 6, 2015
 *      Author: roiyeho
 */

#include "Logger.h"
#include <ros/ros.h>

Logger& Logger::getInstance() {
	static Logger instance; // Instantiated on first use, guaranteed to be destroyed.
	return instance;
}

Logger::Logger() {
	ros::NodeHandle nh;
	nh.getParam("write_log", writeToLogFile);

	if (writeToLogFile) {
		nh.getParam("log_file", logFilePath);
		logFile.open(logFilePath.c_str(), fstream::out);
	}
}

void Logger::write(const string &msg) {
	ROS_INFO("%s", msg.c_str());
	if (writeToLogFile)
		logFile << msg << endl;
}

void Logger::printGrid(const Grid &grid, bool writeToConsole) {
	int rows = grid.size();
	int cols = grid[0].size();

    for (int i = 0; i < rows; i++)
    {
    	logFile << "row " << i << ": ";
    	if (writeToConsole) cout << "row " << i << ": ";
        for (int j = 0; j < cols; j++)
        {
        	logFile << grid[i][j] << " ";
        	if (writeToConsole) cout << grid[i][j] << " ";
        }
        logFile << endl;
        if (writeToConsole) cout << endl;
    }
}

void Logger::printPath(const Path& path) {
	for (vector<Cell>::const_iterator it = path.begin(); it != path.end(); it++) {
		Cell cell = *it;

		stringstream msg;
		msg << "(" << cell.first << "," << cell.second << ") ";
		logFile << msg.str();
		cout << msg.str();
	}
	logFile << endl;
	cout << endl;
}

Logger::~Logger() {
	if (writeToLogFile)
		logFile.close();
}
