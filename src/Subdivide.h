#include <string.h>
#include <iostream>
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



/* Class to represent a Bezier Patch: Defined by 16 control points.*/
class BezierPatch {
	MatrixXf data;

public:
	BezierPatch (const MatrixXf & _data, float _step=0.01) : data(_data) {
		assert(("Bezier Patch did not get 16 3-dimensional points.", data.rows()==16 && data.cols()==3));
	}
};
