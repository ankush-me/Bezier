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
	return (tangentV.cross(tangentU)).normalized();
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

void BezierPatch::findAABB (Vector3f & minPoint, Vector3f & maxPoint) {
	minPoint = Vector3f(matX.minCoeff(), matY.minCoeff(), matZ.minCoeff());
	maxPoint = Vector3f(matX.maxCoeff(), matY.maxCoeff(), matZ.maxCoeff());
}

int serializeIndex(const int i, const int j, const int N) {
	return i*N+j;
}

/** Sample the bezier patch uniformly with STEP.*/
void BezierPatch::sampleUniformly(){
	uniformSamplesGL.clear();

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
			Vector3f pos    = surfaceEval2D(matX, matY, matZ, step_points[ui], step_points[vi]);
			Vector3f normal = surfaceEval2DNormal(matX, matY, matZ, step_points[ui], step_points[vi]);

			VertexNormal vn(pos, normal);
			VertexNormalGL vnGL(vn);
			uniformSamplesGL.push_back(vnGL);
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

/** Adaptive tessellation
 * 	Recursively split into triangles until error is less that tolerance.
 * 	Error - distance between midpoints of triangles and points evaluated
 * 	at u,v of midpoint on curve. */
void BezierPatch::adaptiveSample () {
	adaptiveSamples.clear();
	adaptiveSamplesGL.clear();

	float t1us[] = {0, 1, 1}, t2us[] = {0, 1, 0};
	float t1vs[] = {0, 0, 1}, t2vs[] = {0, 1, 1};
	int t1Inds[] = {0, 1, 2}, t2Inds[] = {0, 2, 3};

	VertexNormal v1(evalPoint(0, 0), evalNormal(0, 0));
	VertexNormal v2(evalPoint(1, 0), evalNormal(1, 0));
	VertexNormal v3(evalPoint(1, 1), evalNormal(1, 1));
	VertexNormal v4(evalPoint(0, 1), evalNormal(0, 1));
	adaptiveSamples.push_back(v1);
	adaptiveSamples.push_back(v2);
	adaptiveSamples.push_back(v3);
	adaptiveSamples.push_back(v4);

	VertexNormalGL v1GL(v1);
	VertexNormalGL v2GL(v2);
	VertexNormalGL v3GL(v3);
	VertexNormalGL v4GL(v4);

	adaptiveSamplesGL.push_back(v1GL);
	adaptiveSamplesGL.push_back(v2GL);
	adaptiveSamplesGL.push_back(v3GL);
	adaptiveSamplesGL.push_back(v4GL);

	assert(adaptiveSamples.size() == adaptiveSamplesGL.size());

	checkAndSplit(t1us, t1vs, t1Inds,0);
	checkAndSplit(t2us, t2vs, t2Inds,0);
}


bool BezierPatch::shouldSplit(const Vector3f &p1,
		const Vector3f & p2, const Vector3f &pt, float frac) {
	return ((frac*p1 + (1-frac)*p2 - pt).norm() > tolerance);
}


void BezierPatch::checkAndSplit(const float us[3], const float vs[3],
		const int inds[3], int depth) {

	for (int i = 0 ; i < 3; i +=1 ) {
		assert(inds[i] >= 0 && inds[i] < adaptiveSamples.size());
	}

	Vector3f v1 =  adaptiveSamples[inds[0]].pos;
	Vector3f v2 =  adaptiveSamples[inds[1]].pos;
	Vector3f v3 =  adaptiveSamples[inds[2]].pos;

	Vector3f v12 =  evalPoint((us[0] + us[1])/2.0, (vs[0] + vs[1])/2.0);
	Vector3f v23 =  evalPoint((us[1] + us[2])/2.0, (vs[1] + vs[2])/2.0);
	Vector3f v31 =  evalPoint((us[2] + us[0])/2.0, (vs[2] + vs[0])/2.0);

	Vector3f n12 =  evalNormal((us[0] + us[1])/2.0, (vs[0] + vs[1])/2.0);
	Vector3f n23 =  evalNormal((us[1] + us[2])/2.0, (vs[1] + vs[2])/2.0);
	Vector3f n31 =  evalNormal((us[2] + us[0])/2.0, (vs[2] + vs[0])/2.0);

	bool split12 = shouldSplit(v1, v2, v12);
	bool split23 = shouldSplit(v2, v3, v23);
	bool split31 = shouldSplit(v3, v1, v31);

	int numSplit = (split12? 1:0) + (split23? 1:0) + (split31? 1:0);

	if (!(split12 || split23 || split31)) {// good triangle keep it
		Triangle T(inds[0], inds[1], inds[2]);
		adaptiveTriangles.push_back(T);
		return;
	} else if (numSplit == 1) { // split 1 edge:
		Vector3f w , n;
		unsigned int i1, i2, i3;
		if (split12)      {i1=0; i2=1; i3=2; w = v12; n = n12;}
		else if (split23) {i1 = 1; i2 = 2; i3 = 0; w = v23; n = n23;}
		else if (split31) {i1 = 2; i2 = 0; i3 = 1; w = v31; n = n31;}

		// add the split vertex
		VertexNormal vn(w, n);
		adaptiveSamples.push_back(vn);
		VertexNormalGL vnGL(vn);
		adaptiveSamplesGL.push_back(vnGL);

		// calculate the new u,v, and sample index
		float u4 = (us[i1] + us[i2])/2.0, v4 = (vs[i1] + vs[i2])/2.0;
		int v4Ind = adaptiveSamples.size()-1;

		// make recursive calls
		float t1u[3] = {us[i1], u4, us[i3]};
		float t1v[3] = {vs[i1], v4, vs[i3]};
		int t1inds[3] = {inds[i1], v4Ind, inds[i3]};
		checkAndSplit(t1u, t1v, t1inds,depth+1);

		float t2u[3] = {u4, us[i2], us[i3]};
		float t2v[3] = {v4, vs[i2], vs[i3]};
		int t2inds[3] = {v4Ind, inds[i2], inds[i3]};
		checkAndSplit(t2u, t2v, t2inds, depth+1);


	} else if (numSplit == 2) { // split 2 edges:

		Vector3f w1, n1, w2, n2;
		int i1, i2, i3;
		if       (split12 && split23  && !split31) {i1=0; i2=1; i3=2; w1=v12; n1=n12; w2=v23; n2=n23;}
		else if (split23 && split31 && !split12)  {i1=1;  i2=2; i3=0; w1=v23; n1=n23; w2=v31; n2=n31;}
		else if (split31 && split12 && !split23)  {i1 = 2;  i2 = 0; i3 = 1; w1 = v31; n1 = n31; w2 = v12; n2 = n12;}
		else  {return; cout << "****************** NOONNEEE *************"<<endl; }

		// add the split vertices
		VertexNormal vn1(w1, n1);
		adaptiveSamples.push_back(vn1);
		VertexNormalGL vnGL1(vn1);
		adaptiveSamplesGL.push_back(vnGL1);

		VertexNormal vn2(w2, n2);
		adaptiveSamples.push_back(vn2);
		VertexNormalGL vnGL2(vn2);
		adaptiveSamplesGL.push_back(vnGL2);

		const int numSamples = adaptiveSamples.size();

		// calculate the new u,v, and sample index
		float u4 = (0.5*us[i1] + 0.5*us[i2]), v4 = (0.5*vs[i1] + 0.5*vs[i2]);
		int v4Ind = numSamples-2;

		float u5= (0.5*us[i2] + 0.5*us[i3]), v5 = (0.5*vs[i2] + 0.5*vs[i3]);
		int v5Ind = numSamples-1;

		// make recursive calls
		float tt1u[3] =  {us[i1], u5, us[i3]};
		float tt1v[3] =  {vs[i1], v5, vs[i3]};
		int tt1inds[3] = {inds[i1], v5Ind, inds[i3]};

		float tt2u[3] =  {u5, u4, us[i2]};
		float tt2v[3] =  {v5, v4, vs[i2]};
		int tt2inds[3] = {v5Ind, v4Ind, inds[i2]};

		float tt3u[3]  =  {us[i1], u4, u5};
		float tt3v[3]  =  {vs[i1], v4, v5};
		int   tt3inds[3] = {inds[i1], v4Ind, v5Ind};


		checkAndSplit(tt1u, tt1v, tt1inds, depth+1);
		checkAndSplit(tt2u, tt2v, tt2inds, depth+1);
		checkAndSplit(tt3u, tt3v, tt3inds, depth+1);

	} else if (numSplit == 3) { // split all edges edges:

		// add the split vertices
		VertexNormal vn1(v12, n12);
		adaptiveSamples.push_back(vn1);
		VertexNormalGL vnGL1(vn1);
		adaptiveSamplesGL.push_back(vnGL1);

		VertexNormal vn2(v23, n23);
		adaptiveSamples.push_back(vn2);
		VertexNormalGL vnGL2(vn2);
		adaptiveSamplesGL.push_back(vnGL2);

		VertexNormal vn3(v31, n31);
		adaptiveSamples.push_back(vn3);
		VertexNormalGL vnGL3(vn3);
		adaptiveSamplesGL.push_back(vnGL3);

		// calculate the new u,v, and sample index
		float u4 = (us[0] + us[1])/2.0, v4 = (vs[0] + vs[1])/2.0;
		int v4Ind = adaptiveSamples.size()-3;

		float u5= (us[1] + us[2])/2.0, v5 = (vs[1] + vs[2])/2.0;
		int v5Ind = adaptiveSamples.size()-2;

		float u6= (us[2] + us[0])/2.0, v6 = (vs[2] + vs[0])/2.0;
		int v6Ind = adaptiveSamples.size()-1;

		// make recursive calls
		float t1u[3] = {us[0], u4, u6};
		float t1v[3] = {vs[0], v4, v6};
		int t1inds[3] = {inds[0], v4Ind, v6Ind};
		checkAndSplit(t1u, t1v, t1inds, depth+1);

		float t2u[3] = {u6, u4, u5};
		float t2v[3] = {v6, v4, v5};
		int t2inds[3] = {v6Ind, v4Ind, v5Ind};
		checkAndSplit(t2u, t2v, t2inds, depth+1);

		float t3u[3] = {u5, u4, us[1]};
		float t3v[3] = {v5, v4, vs[1]};
		int t3inds[3] = {v5Ind, v4Ind, inds[1]};
		checkAndSplit(t3u, t3v, t3inds, depth+1);

		float t4u[] = {us[2], u6, u5};
		float t4v[] = {vs[2], v6, v5};
		int t4inds[] = {inds[2], v6Ind, v5Ind};
		checkAndSplit(t4u, t4v, t4inds, depth+1);
	}
}


/** OpenGL code to draw the Bezier Patch */
void BezierPatch::drawPatch(bool drawUniform) {
	const vector<Triangle> *tris = drawUniform? &uniformTriangles : &adaptiveTriangles;
	const vector<VertexNormalGL> *verts = drawUniform? &uniformSamplesGL : &adaptiveSamplesGL;

	for (int t = 0; t < tris->size(); t +=1) {

		Triangle T = tris->at(t);
		VertexNormalGL v1 = verts->at(T.indices[0]);
		VertexNormalGL v2 = verts->at(T.indices[1]);
		VertexNormalGL v3 = verts->at(T.indices[2]);

		glBegin(GL_TRIANGLES);

		glNormal3f(v1.normal[0], v1.normal[1], v1.normal[2]);
		glVertex3f(v1.pos[0], v1.pos[1], v1.pos[2]);

		glNormal3f(v2.normal[0], v2.normal[1], v2.normal[2]);
		glVertex3f(v2.pos[0], v2.pos[1], v2.pos[2]);

		glNormal3f(v3.normal[0], v3.normal[1], v3.normal[2]);
		glVertex3f(v3.pos[0], v3.pos[1], v3.pos[2]);

		glEnd();
	}
}
