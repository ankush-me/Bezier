#include "BezierPatch.h"


/** define the basis function. Maps control-points --> parametric point.
 *  [a_x(u), a_y(u), a_z(u)]  = [1, u, u^2, u^3] * BEZIERBASIS * [p0_x, p0_y, p0_z,
 *                                                                p1_x, p1_y, p1_z,
 *                                                                p2_x, p2_y, p2_z,
 *                                                                p3_x, p3_y, p3_z ]
 *  where a(u) is the point parameterized by u and p_i are the control points.*/
const Matrix4f BezierPatch::BezierBasis = (Matrix4f() << 1.f,  0.f,  0.f,  0.f,
														 -3.f,  3.f,  0.f,  0.f,
														  3.f, -6.f,  3.f,  0.f,
														  -1.f,  3.f,  -3.f, 1.f). finished();

const Matrix4f BezierPatch::BezierBasisTranspose = BezierPatch::BezierBasis.transpose();


Vector3f BezierPatch::curveEval(const MatrixXf &mat, float u) {
	assert(("Unknown shape of control-point matrix.", mat.rows()==4 && mat.cols()==3));
	return Vector3f((powers(u).transpose()*BezierBasis)*mat);
}

/* Evaluates the tangent at a point pt(u) of a Bezier CURVE. */
Vector3f BezierPatch::curveEvalTangent(const MatrixXf &mat, float u) {
	assert(("Unknown shape of control-point matrix.", mat.rows()==4 && mat.cols()==3));
	return Vector3f((derivativePowers(u).transpose()*BezierBasis)*mat);
}

/** Evaluates ONE-DIMENSION (i.e. x, y or z) of a point on a Bezier SURFACE patch.
 *  MAT : is a 4x4 matrix holding the values of that dimension for the 16 control points.
 *  u,v : are the interpolation parameters (0 <= u,v <= 1).
 *
 *  Assumes, that u is down the column, v is across the row of MAT.*/
float BezierPatch::surfaceEval2D(const Matrix4f &mat, float u, float v) {
	return (powers(u).transpose()*BezierBasis)*(mat*(BezierBasisTranspose*powers(v)));
}

/** Evaluates a point on a Bezier SURFACE patch.
 *  MAT_X : the values of x-coordinates of the control-points.
 *  MAT_Y : the values of y-coordinates of the control-points.
 *  MAT_Z : the values of z-coordinates of the control-points.
 *  u,v : are the interpolation parameters (0 <= u,v <= 1).
 *
 *  Assumes, that u is down the column, v is across the row of MAT.*/
Vector3f BezierPatch::surfaceEval2D(const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
		float u, float v) {
	return Vector3f(surfaceEval2D(matX, u,v), surfaceEval2D(matY, u,v), surfaceEval2D(matZ, u,v));
}

/** Evaluates 1-coordinate of the tangent along the 'u' parameter of a 2D surface.
 *  The evaluated coordinates corresponds to the matrix MAT. */
float BezierPatch::surfaceEval2DTangentU(const Matrix4f &mat, float u, float v) {
	return (derivativePowers(u).transpose()*BezierBasis)*(mat*(BezierBasisTranspose*powers(v)));
}

/** Evaluates 1-coordinate of the tangent along the 'v' parameter of a 2D surface.
 *  The evaluated coordinates corresponds to the matrix MAT. */
float BezierPatch::surfaceEval2DTangentV(const Matrix4f &mat, float u, float v) {
	return (powers(u).transpose()*BezierBasis)*(mat*(BezierBasisTranspose*derivativePowers(v)));
}

/** Evaluates the tangent along the 'u' parameter of a 2D surface.*/
Vector3f BezierPatch::surfaceEval2DTangentU (const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
		float u, float v) {
	return Vector3f(surfaceEval2DTangentU(matX, u,v),
			surfaceEval2DTangentU(matY, u,v),
			surfaceEval2DTangentU(matZ, u,v));
}

/** Evaluates the tangent along the 'v' parameter of a 2D surface.*/
Vector3f BezierPatch::surfaceEval2DTangentV(const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
		float u, float v) {
	return Vector3f(surfaceEval2DTangentV(matX, u,v),
			surfaceEval2DTangentV(matY, u,v),
			surfaceEval2DTangentV(matZ, u,v));
}

/* Evaluates the normal at a point pt(u,v) of a Bezier SURFACE. */
Vector3f BezierPatch::surfaceEval2DNormal(const Matrix4f &matX, const Matrix4f &matY, const Matrix4f &matZ,
		float u, float v) {
	Vector3f tangentU =  surfaceEval2DTangentU(matX, matY, matZ, u, v);
	Vector3f tangentV =  surfaceEval2DTangentV	(matX, matY, matZ, u, v);
	return (tangentU.cross(tangentV)).normalized();
}



// Constructor
BezierPatch::BezierPatch (const MatrixXf & _data, float _tolerance, float _step):
										data(_data), step(_step), tolerance(_tolerance) {

	assert(("BezierPatch did not get correct patch data. Expecting 4x12 matrix.",
			data.rows()==4 && data.cols()==12));

	for (int i = 0; i < 4; i+=1) {  // get the coordinates of the control points.
		matX.col(i) = data.col(3*i);
		matY.col(i) = data.col(3*i +1);
		matZ.col(i) = data.col(3*i + 2);
	}
}

Vector3f BezierPatch::evalPoint(float u, float v) {
	return surfaceEval2D (matX, matY, matZ, u, v);
}

Vector3f BezierPatch::evalNormal (float u, float v) {
	return surfaceEval2DNormal (matX, matY, matZ, u, v);
}

void getXY(int i, const int N, int &x, int &y) {
	x = (int) i/N;
	y = (int) i % N;
}

int serializeIndex(const int i, const int j, const int N) {
	return i*N+j;
}

/** Sample the bezier patch uniformly with STEP.*/
void BezierPatch::sampleUniformly(){
	uniformSamples.clear();

	const int M = (int) ceil(1.0/step) - 1;
	vector<float> step_points;
	int i = 0;
    while (i <= M) {
        step_points.push_back(i*step);
        i += 1;
    }
    step_points.push_back(1.0);
    const int num_points = step_points.size();

    // sample the bezier patch
    for (int ui = 0; ui < num_points; ui +=1) {
    	for (int vi = 0; vi < num_points; vi +=1) {
    		VertexNormal vn (surfaceEval2D(matX, matY, matZ, step_points[ui], step_points[vi]),
    				         surfaceEval2DNormal(matX, matY, matZ, step_points[ui], step_points[vi]));
    		uniformSamples.push_back(vn);
    	}
    }

	// now form triangles out of the sampled points.
    uniformTriangles.clear();
    for (int ui = 0; ui < num_points -1; ui += 1) {
    	for (int vi = 0; vi < num_points -1; vi +=1) {
    		const int v1 = serializeIndex(ui,   vi,   num_points);
    		const int v2 = serializeIndex(ui+1, vi,   num_points);
    		const int v3 = serializeIndex(ui+1, vi+1, num_points);
    		const int v4 = serializeIndex(ui,   vi+1, num_points);

    		uniformTriangles.push_back(Triangle(v1, v2, v3));
    		uniformTriangles.push_back(Triangle(v1, v3, v4));
    	}
    }
}

/** Adaptive tessellation -
 * 	Recursively split into triangles until error is less that tolerance.
 * 	Error - distance between midpoints of triangles and points evaluated
 * 	at u,v of midpoint on curve. */
void BezierPatch::adaptiveSample () {

	vector<float> us1(3), vs1(3);
	vector<float> us2(3), vs2(3);
	vector<unsigned int> inds1(3), inds2(3);
	us1[0] = 0.0; us1[1] = 1.0; us1[2] = 1.0;
	vs1[0] = 0.0; vs1[1] = 1.0; vs1[2] = 0.0;
	inds1[0] = 0; inds1[1] = 2; inds1[2] = 3;
	us2[0] = 0.0; us2[1] = 0.0; us2[2] = 1.0;
	vs2[0] = 0.0; vs2[1] = 1.0; vs2[2] = 1.0;
	inds2[0] = 0; inds2[1] = 1; inds2[2] = 2;

	adaptiveSamples.push_back(VertexNormal(evalPoint(0.0, 0.0), evalNormal(0.0, 0.0)));
	adaptiveSamples.push_back(VertexNormal(evalPoint(0.0, 1.0), evalNormal(0.0, 1.0)));
	adaptiveSamples.push_back(VertexNormal(evalPoint(1.0, 1.0), evalNormal(1.0, 1.0)));
	adaptiveSamples.push_back(VertexNormal(evalPoint(1.0, 0.0), evalNormal(1.0, 0.0)));

	splitTriangle(us1, vs1, inds1);
	splitTriangle(us2, vs2, inds2);
}

/**
 * Splits triangle described by us and vs, only if error is greater than threshold.
 * TODO: Might want to change order of inserting into us, vs so as to make triangulation
 * look good.
 */
void BezierPatch::splitTriangle (vector<float> us, vector<float> vs, vector<unsigned int> inds) {

	Vector3f trianglePt0 = adaptiveSamples[inds[0]].pos;
	Vector3f trianglePt1 = adaptiveSamples[inds[1]].pos;
	Vector3f trianglePt2 = adaptiveSamples[inds[2]].pos;

	// Check distance of midpoints
	if (((trianglePt0 + trianglePt1)/2 - evalPoint((us[0]+us[1])/2, (vs[0]+vs[1])/2)).norm() > tolerance) {

		adaptiveSamples.push_back(VertexNormal(evalPoint((us[0]+us[1])/2, (vs[0]+vs[1])/2), evalNormal((us[0]+us[1])/2, (vs[0]+vs[1])/2)));
		unsigned int newInd = adaptiveSamples.size()-1;

		vector<float> us1(3), vs1(3);
		vector<float> us2(3), vs2(3);
		vector<unsigned int> inds1(3), inds2(3);
		us1[0] = us[0]; us1[1] = (us[0]+us[1])/2; us1[2] = us[2];
		vs1[0] = vs[0]; vs1[1] = (vs[0]+vs[1])/2; vs1[2] = vs[2];
		inds1[0] = inds[0]; inds1[1] = newInd; inds1[2] = inds[2];
		us2[0] = (us[0]+us[1])/2; us2[1] = us[1]; us2[2] = us[2];
		vs2[0] = (vs[0]+vs[1])/2; vs2[1] = vs[1]; vs2[2] = vs[2];
		inds2[0] = newInd; inds2[1] = inds[1]; inds2[2] = inds[2];

		splitTriangle(us1, vs1, inds1);
		splitTriangle(us2, vs2, inds2);

	} else if (((trianglePt1 + trianglePt2)/2 - evalPoint((us[1]+us[2])/2, (vs[1]+vs[2])/2)).norm() > tolerance) {

		adaptiveSamples.push_back(VertexNormal(evalPoint((us[1]+us[2])/2, (vs[1]+vs[2])/2), evalNormal((us[1]+us[2])/2, (vs[1]+vs[2])/2)));
		unsigned int newInd = adaptiveSamples.size()-1;

		vector<float> us1(3), vs1(3);
		vector<float> us2(3), vs2(3);
		vector<unsigned int> inds1(3), inds2(3);
		us1[0] = us[0]; us1[1] = us[1]; us1[2] = (us[1]+us[2])/2;
		vs1[0] = vs[0]; vs1[1] = vs[1]; vs1[2] = (vs[1]+vs[2])/2;
		inds1[0] = inds[0]; inds1[1] = inds[1]; inds1[2] = newInd;
		us2[0] = us[0]; us2[1] = (us[1]+us[2])/2; us2[2] = us[2];
		vs2[0] = vs[0]; vs2[1] = (vs[1]+vs[2])/2; vs2[2] = vs[2];
		inds2[0] = inds[0]; inds2[1] = newInd; inds2[2] = inds[2];

		splitTriangle(us1, vs1, inds1);
		splitTriangle(us2, vs2, inds2);

	} else if (((trianglePt0 + trianglePt2)/2 - evalPoint((us[0]+us[2])/2, (vs[0]+vs[2])/2)).norm() > tolerance) {

		adaptiveSamples.push_back(VertexNormal(evalPoint((us[0]+us[2])/2, (vs[0]+vs[2])/2), evalNormal((us[0]+us[2])/2, (vs[1]+vs[2])/2)));
		unsigned int newInd = adaptiveSamples.size()-1;

		vector<float> us1(3), vs1(3);
		vector<float> us2(3), vs2(3);
		vector<unsigned int> inds1(3), inds2(3);
		us1[0] = us[0]; us1[1] = us[1]; us1[2] = (us[0]+us[2])/2;
		vs1[0] = vs[0]; vs1[1] = vs[1]; vs1[2] = (vs[0]+vs[2])/2;
		inds1[0] = inds[0]; inds1[1] = inds[1]; inds1[2] = newInd;
		us2[0] = (us[0]+us[2])/2; us2[1] = us[1]; us2[2] = us[2];
		vs2[0] = (vs[0]+vs[2])/2; vs2[1] = vs[1]; vs2[2] = vs[2];
		inds2[0] = newInd; inds2[1] = inds[1]; inds2[2] = inds[2];

		splitTriangle(us1, vs1, inds1);
		splitTriangle(us2, vs2, inds2);

	} else {
		Triangle t = Triangle(inds);
		vector<unsigned int> indos = t.indices;
		adaptiveTriangles.push_back(t);
		cout<<"\n\nStuff: \n    Pos: "<<endl;
		cout<<"      "<<adaptiveSamples[indos[0]].pos.transpose()<<endl;
		cout<<"      "<<adaptiveSamples[indos[1]].pos.transpose()<<endl;
		cout<<"      "<<adaptiveSamples[indos[2]].pos.transpose()<<endl;
		cout<<"    Normals: "<<endl;
		cout<<"      "<<adaptiveSamples[indos[0]].normal.transpose()<<endl;
		cout<<"      "<<adaptiveSamples[indos[1]].normal.transpose()<<endl;
		cout<<"      "<<adaptiveSamples[indos[2]].normal.transpose()<<endl;
		cout<<"    U's and V's: "<<endl;
		cout<<"      "<<us[0]<<", "<<vs[0]<<endl;
		cout<<"      "<<us[1]<<", "<<vs[1]<<endl;
		cout<<"      "<<us[2]<<", "<<vs[2]<<endl;
		cout<<"Size of adaptive triangles: "<<adaptiveTriangles.size()<<endl;
	}
}
