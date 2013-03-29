#include <vector>
#include <string.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


/** define the basis function. Maps control-points --> parametric point.
 *  [a_x(u), a_y(u), a_z(u)]  = [1, u, u^2, u^3] * BEZIERBASIS * [p0_x, p0_y, p0_z,
 *                                                                p1_x, p1_y, p1_z,
 *                                                                p2_x, p2_y, p2_z,
 *                                                                p3_x, p3_y, p3_z ]
 *  where a(u) is the point parameterized by u and p_i are the control points.*/
const Matrix4f BezierBasis = (Matrix4f() << 1.f,  0.f,  0.f,  0.f,
		-3.f,  3.f,  0.f,  0.f,
		3.f, -6.f,  3.f,  0.f,
		-1.f,  3.f,  -3.f, 1.f). finished();

const Matrix4f BezierBasisTranspose = BezierBasis.transpose();

Vector4f powers(float u) { // powers of parameter used for calculating the points
	return Vector4f(1.0, u, u*u, u*u*u);
}

Vector4f derivativePowers(float u) {// powers used to calculate the derivative/ tangent
	return Vector4f(0.0, 1.0, 2*u, 3*u*u);
}

/** Evaluates a point of a Bezier CURVE.
 *  CTRLPTS : is a 4x3 matrix holding the control points : [c0_x, c0_y, c0_z,
 *                                                          c1_x, c1_y, c1_z,
 *                                                          c2_x, c2_y, c2_z,
 *                                                          c3_x, c3_y, c3_z ]
 *  u       : is the interpolation parameter (0 <= u <= 1).*/
Vector3f bezierEval(const MatrixXf &mat, float u) {
	assert(("Unknown shape of control-point matrix.", mat.rows()==4 && mat.cols()==3));
	return Vector3f((powers(u).transpose()*BezierBasis)*mat);
}

/* Evaluates the tangent at a point pt(u) of a Bezier CURVE. */
Vector3f bezierEvalTangent(const MatrixXf &mat, float u) {
	assert(("Unknown shape of control-point matrix.", mat.rows()==4 && mat.cols()==3));
	return Vector3f((derivativePowers(u).transpose()*BezierBasis)*mat);
}


/** Evaluates ONE-DIMENSION (i.e. x, y or z) of a point on a Bezier SURFACE patch.
 *  MAT : is a 4x4 matrix holding the values of that dimension for the 16 control points.
 *  u,v : are the interpolation parameters (0 <= u,v <= 1).
 *
 *  Assumes, that u is down the column, v is across the row of MAT.*/
float bezierEval2D(const Matrix4f &mat, float u, float v) {
	return (powers(u).transpose()*BezierBasis)*(mat*(BezierBasisTranspose*powers(v)));
}

/** Evaluates a point on a Bezier SURFACE patch.
 *  MAT_X : the values of x-coordinates of the control-points.
 *  MAT_Y : the values of y-coordinates of the control-points.
 *  MAT_Z : the values of z-coordinates of the control-points.
 *  u,v : are the interpolation parameters (0 <= u,v <= 1).
 *
 *  Assumes, that u is down the column, v is across the row of MAT.*/
Vector3f bezierEval2D(const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
		float u, float v) {
	return Vector3f(bezierEval2D(matX, u,v), bezierEval2D(matY, u,v), bezierEval2D(matZ, u,v));
}

/** Evaluates 1-coordinate of the tangent along the 'u' parameter of a 2D surface.
 *  The evaluated coordinates corresponds to the matrix MAT. */
float bezierEval2DTangentU(const Matrix4f &mat, float u, float v) {
	return (derivativePowers(u).transpose()*BezierBasis)*(mat*(BezierBasisTranspose*powers(v)));
}
/** Evaluates 1-coordinate of the tangent along the 'v' parameter of a 2D surface.
 *  The evaluated coordinates corresponds to the matrix MAT. */
float bezierEval2DTangentV(const Matrix4f &mat, float u, float v) {
	return (powers(u).transpose()*BezierBasis)*(mat*(BezierBasisTranspose*derivativePowers(v)));
}

/** Evaluates the tangent along the 'u' parameter of a 2D surface.*/
Vector3f bezierEval2DTangentU(const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
		float u, float v) {
	return Vector3f(bezierEval2DTangentU(matX, u,v),
			bezierEval2DTangentU(matY, u,v),
			bezierEval2DTangentU(matZ, u,v));
}

/** Evaluates the tangent along the 'v' parameter of a 2D surface.*/
Vector3f bezierEval2DTangentV(const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
		float u, float v) {
	return Vector3f(bezierEval2DTangentV(matX, u,v),
			bezierEval2DTangentV(matY, u,v),
			bezierEval2DTangentV(matZ, u,v));
}

/* Evaluates the tangent at a point pt(u,v) of a Bezier CURVE. */
Vector3f bezierEval2DNormal(const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
		float u, float v) {
	Vector3f tangentU =  bezierEval2DTangentU(matX, matY, matZ, u, v);
	Vector3f tangentV =  bezierEval2DTangentV	(matX, matY, matZ, u, v);
	return (tangentU.cross(tangentV)).normalized();
}


// vertex which stores position and normal
struct VertexNormal {
	const Vector3f pos;
	const Vector3f normal;
	VertexNormal (const Vector3f &_pos, const Vector3f & _normal) : pos(_pos), normal(_normal) {}
};


/* Class to represent a Bezier Patch: Defined by 16 control points.*/
class BezierPatch {

	const float step;              // step-size for uniform sampling
	const float tolerance;         // tolerance for adaptive sampling

	MatrixXf data;
	Matrix4f matX, matY, matZ;      // store the coordinates of the control points

	vector<VertexNormal> uniformSamples;  // vertices found using uniform samples
	vector<VertexNormal> adaptiveSamples; // vertices found using adaptive sampling


public:
	BezierPatch (const MatrixXf & _data, float _tolerance = 0.001, float _step=0.01) :
		data(_data), step(_step), tolerance(_tolerance) {
		assert(("BezierPatch did not get correct patch data. Expecting 4x12 matrix.",
				 data.rows()==4 && data.cols()==12));

		for (int i = 0; i < 4; i+=1) {  // get the coordinates of the control points.
			matX.col(i) = data.col(3*i);
			matY.col(i) = data.col(3*i +1);
			matZ.col(i) = data.col(3*i + 2);
		}
	}

	/* Draws the patch in openGL.
	 * if DRAWUNIFORM is true, uniformly sampled patch is drawn,
	 * else adaptively-sampled patch is drawn.*/
	void drawPatch(bool drawUniform);

	/** Sample the bezier patch uniformly with STEP.*/
	void sampleUniformly() {
		uniformSamples.clear();
		float u = 0.0;
		while (u < 1.0) {
			float v = 0.0;
			while (v < 0.0) {
				VertexNormal vn (bezierEval2D(matX, matY, matZ, u,v),
							     bezierEval2DNormal(matX, matY, matZ, u,v));
				uniformSamples.push_back(vn);
				v += step;
			}
			v = 1.f;
			VertexNormal vn (bezierEval2D(matX, matY, matZ, u,v),
									     bezierEval2DNormal(matX, matY, matZ, u,v));
			uniformSamples.push_back(vn);
			u += step;
		}

		u = 1.0f;
		float v = 0.0;
		while (v < 0.0) {
			VertexNormal vn (bezierEval2D(matX, matY, matZ, u,v),
					         bezierEval2DNormal(matX, matY, matZ, u,v));
			uniformSamples.push_back(vn);
			v += step;
		}
		v = 1.f;
		VertexNormal vn (bezierEval2D(matX, matY, matZ, u,v),
				         bezierEval2DNormal(matX, matY, matZ, u,v));
		uniformSamples.push_back(vn);
	}


	void adaptiveSample() {}
};

/** Reads a .bez file. */
vector<BezierPatch> readPatches(std::string fname) {
	vector <BezierPatch> patches;
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
				BezierPatch patch(*mat);
				patches.push_back(patch);
				mat = new MatrixXf(4,12);
			}
		}
	}
	assert(("Incorrect number of patches read.", num_patches==patches.size()));
	return patches;
}
