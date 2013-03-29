#include "Parser.h"

#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)

int main (int argc, char **argv) {

	string fname = EXPAND (PROJECT_DATA_DIR) "/test.bez";
	cout << EXPAND(PROJECT_DATA_DIR) "/test.bez" <<endl;

	vector<BezierPatch> x = readPatches (fname);

	return 0;
}
