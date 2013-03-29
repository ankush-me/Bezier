#ifndef __SUBDIVIDE__
#define __SUBDIVIDE__

#include <vector>
#include <string.h>
#include <iostream>
#include <fstream>
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

// vertex which stores position and normal
struct VertexNormal {
	Vector3f pos;
	Vector3f normal;
	VertexNormal (const Vector3f &_pos, const Vector3f & _normal) : pos(_pos), normal(_normal) {}
};

/* Class to represent a Bezier Patch: Defined by 16 control points.*/
class BezierPatch {

	static const Matrix4f BezierBasis;
	static const Matrix4f BezierBasisTranspose;

	float step;              // step-size for uniform sampling
	float tolerance;         // tolerance for adaptive sampling

	MatrixXf data;
	Matrix4f matX, matY, matZ;      // store the coordinates of the control points

	// Triangle whose vertices are specified by the indices
	struct Triangle {
		vector<unsigned int> indices;
		Triangle (const int i0, const int i1, const int i2) : indices (3) {
			indices[0] = i0; indices[1] = i1; indices[2] = i2;
		}
		Triangle (const vector<unsigned int> inds) : indices (3) {
			indices[0] = inds[0]; indices[1] = inds[1]; indices[2] = inds[2];
		}
	};

	vector<VertexNormal> uniformSamples;  // vertices found using uniform samples
	vector<Triangle> uniformTriangles;    // triangles found using uniform samples
	vector<VertexNormal> adaptiveSamples; // vertices found using adaptive sampling
	vector<Triangle> adaptiveTriangles;   // triangles found using adaptive sampling

	// powers of parameter used for calculating the points
	static Vector4f powers(float u) { return Vector4f(1.0, u, u*u, u*u*u);}
	// powers used to calculate the derivative/ tangent
	static Vector4f derivativePowers(float u) {return Vector4f(0.0, 1.0, 2*u, 3*u*u);}

	/** Evaluates a point of a Bezier CURVE.
	 *  CTRLPTS : is a 4x3 matrix holding the control points : [c0_x, c0_y, c0_z,
	 *                                                          c1_x, c1_y, c1_z,
	 *                                                          c2_x, c2_y, c2_z,
	 *                                                          c3_x, c3_y, c3_z ]
	 *  u       : is the interpolation parameter (0 <= u <= 1).*/
	static Vector3f curveEval(const MatrixXf &mat, float u);

	/* Evaluates the tangent at a point pt(u) of a Bezier CURVE. */
	static Vector3f curveEvalTangent(const MatrixXf &mat, float u);

	/** Evaluates ONE-DIMENSION (i.e. x, y or z) of a point on a Bezier SURFACE patch.
	 *  MAT : is a 4x4 matrix holding the values of that dimension for the 16 control points.
	 *  u,v : are the interpolation parameters (0 <= u,v <= 1).
	 *
	 *  Assumes, that u is down the column, v is across the row of MAT.*/
	static float surfaceEval2D(const Matrix4f &mat, float u, float v);

	/** Evaluates a point on a Bezier SURFACE patch.
	 *  MAT_X : the values of x-coordinates of the control-points.
	 *  MAT_Y : the values of y-coordinates of the control-points.
	 *  MAT_Z : the values of z-coordinates of the control-points.
	 *  u,v : are the interpolation parameters (0 <= u,v <= 1).
	 *
	 *  Assumes, that u is down the column, v is across the row of MAT.*/
	static Vector3f surfaceEval2D(const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
			float u, float v);

	/** Evaluates 1-coordinate of the tangent along the 'u' parameter of a 2D surface.
	 *  The evaluated coordinates corresponds to the matrix MAT. */
	static float surfaceEval2DTangentU(const Matrix4f &mat, float u, float v);

	/** Evaluates 1-coordinate of the tangent along the 'v' parameter of a 2D surface.
	 *  The evaluated coordinates corresponds to the matrix MAT. */
	static float surfaceEval2DTangentV(const Matrix4f &mat, float u, float v);

	/** Evaluates the tangent along the 'u' parameter of a 2D surface.*/
	static Vector3f surfaceEval2DTangentU (const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
											   float u, float v);

	/** Evaluates the tangent along the 'v' parameter of a 2D surface.*/
	static Vector3f surfaceEval2DTangentV(const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
			float u, float v);

	/* Evaluates the normal at a point pt(u,v) of a Bezier SURFACE. */
	static Vector3f surfaceEval2DNormal(const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
			float u, float v);

public:
	BezierPatch (const MatrixXf & _data, float _tolerance = 0.001, float _step=0.01);
	Vector3f evalPoint (float u, float v);
	Vector3f evalNormal (float u, float v);

	/* Draws the patch in openGL.
	 * if DRAWUNIFORM is true, uniformly sampled patch is drawn,
	 * else adaptively-sampled patch is drawn.*/
	void drawPatch(bool drawUniform);

	/** Sample the bezier patch uniformly with STEP.*/
	void sampleUniformly();


	void adaptiveSample();
	void splitTriangle (vector<float> us, vector<float> vs, vector<unsigned int> inds);
};


#endif
