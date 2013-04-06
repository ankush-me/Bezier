#ifndef __PARSER__
#define __PARSER__

#include "BezierPatch.h"
#include "Eigen/StdVector"

#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)

/************************************************************
 * Please make sure that whenever a vector of a class       *
 * which has an object of eigen inside it is defined        *
 * the allocator is also specified (as below).              *
 *                                                          *
 * Also include #include <Eigen/StdVector>   				*
 * 															*
 ************************************************************/

/** Reads a .bez file. */
vector<BezierPatch, Eigen::aligned_allocator<BezierPatch> >
readPatches(std::string fname, float step_or_tol) {
	vector <BezierPatch, Eigen::aligned_allocator<BezierPatch> > patches;
	int num_patches   = -1;
	bool readNum      = false;

	ifstream inpfile(fname.c_str());
	if(!inpfile.is_open()) {
		cout << "Unable to open file : " << fname << endl;
	} else {
		MatrixXf *mat = new MatrixXf(4,12);
		int row = 0;

		while(!inpfile.eof()) {
			string line;
			getline(inpfile,line);
			vector<string> splitline;
			string buf;
			stringstream ss(line);

			while (ss >> buf)
				splitline.push_back(buf);

			if (splitline.size()==0) // skip the blank line
				continue;
			if(splitline[0][0] == '#')
				continue;

			if (!readNum) {
				num_patches = atof(splitline[0].c_str());
				readNum     = true;
				cout << "Number of patches: "<< num_patches<<endl;
				continue;
			}

			assert (("Expecting 12 values per row. Not Found!", splitline.size()==12));
			for (int i=0; i < 12; i +=1)
				(*mat)(row, i) = atof(splitline[i].c_str());
			row += 1;

			if (row == 4) { // start a new patch
				row = 0;
				BezierPatch patch(*mat, step_or_tol, step_or_tol);
				patches.push_back(patch);
				mat = new MatrixXf(4,12);
			}
		}
	}
	assert(("Incorrect number of patches read.", num_patches==patches.size()));
	return patches;
}

#endif
