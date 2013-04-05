#include "Parser.h"
#include "Eigen/StdVector"

#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)

int main (int argc, char **argv) {

	string fname = EXPAND (PROJECT_DATA_DIR) "/test.bez";
	cout << EXPAND(PROJECT_DATA_DIR) "/test.bez" <<endl;

	/************************************************************
	 * Please make sure that whenever a vector of a class       *
	 * which has an object of eigen inside it is defined        *
	 * the allocator is also specified (as below).              *
	 *                                                          *
	 * Also include #include <Eigen/StdVector>   				*
	 * 															*
	 ************************************************************/
	vector<BezierPatch, Eigen::aligned_allocator<BezierPatch> > x = readPatches (fname);

	for (int i = 0; i < x.size(); ++i) {
		x[0].adaptiveSample();
		cout<<"Bezier patch "<<i+1<<": "<<endl;
		for (int j = 0; j < x[i].adaptiveTriangles.size(); ++j) {
			cout<<"    Triangle "<<j+1<<": "<<endl;
			vector<unsigned int> inds = x[i].adaptiveTriangles[j].indices;
			cout<<"        "<<x[i].adaptiveSamples[inds[0]].pos<<endl;
			cout<<"        "<<x[i].adaptiveSamples[inds[1]].pos<<endl;
			cout<<"        "<<x[i].adaptiveSamples[inds[2]].pos<<endl;
		}
	}

	return 0;
}
