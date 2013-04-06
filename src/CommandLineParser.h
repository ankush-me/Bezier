#ifndef __COMMAND_LINE_PARSER__
#define __COMMAND_LINE_PARSER__

#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <string>

#include "Eigen/Dense"

#include "BezierPatch.h"
#include "FileParser.h"

using namespace std;

vector< BezierPatch, Eigen::aligned_allocator<BezierPatch> >
	intializePatchesFromCommandLine(int argCount, char * argVals[], bool & adaptive) {

	string fname = EXPAND (PROJECT_DATA_DIR) "/teapot.bez";
	if (argCount > 1) fname = string(argVals[1]);

	float value = 0.1;
	if (argCount > 2) value = ::atof(argVals[2]);

	adaptive = false; string option("");
	if (argCount > 3) option = string (argVals[3]);
	if (option == "-a") {adaptive = true;}

	return readPatches (fname, value);
}
#endif
