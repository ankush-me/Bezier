// Code based off glut_example.c
// Stanford University, CS248, Fall 2000


#include <GL/glut.h>

#include "CommandLineParser.h"
#include "BezierPatch.h"

using namespace std;

#define VIEWING_DISTANCE_MIN  2.0

static bool g_bLightingEnabled = true;
static bool g_bFillPolygons    = true;
static bool g_bSmoothShading   = true;
static bool g_bButton1Down     = false;
static GLfloat g_fTeapotAngle  = 0.0;
static GLfloat g_fInitViewDistance = VIEWING_DISTANCE_MIN;
static GLfloat g_fViewDistance = 1.2*VIEWING_DISTANCE_MIN;
static GLfloat g_nearPlane     = 1;
static GLfloat g_farPlane      = 1000;

// variables to rotate/ translate the object.
static GLfloat g_angleX        = 0;
static GLfloat g_angleY        = 0;
static GLfloat g_angleZ        = 0;
static GLfloat g_transX        = 0;
static GLfloat g_transY        = 0;

static int g_Width            = 600;                          // Initial window width
static int g_Height           = 600;                         // Initial window height
static int g_yClick            = 0;
static float g_lightPos[4] = { 10, 10, -100, 1 };  // Position of light

const float colorBronzeDiff[4] = { 0.8, 0.6, 0.0, 1.0 };
const float colorBronzeSpec[4] = { 1.0, 1.0, 0.4, 1.0 };
const float colorBlue[4]       = { 0.0, 0.2, 1.0, 1.0 };
const float colorNone[4]       = { 0.0, 0.0, 0.0, 0.0 };

vector < BezierPatch, Eigen::aligned_allocator<BezierPatch> > patches;
bool adaptive = false;

void RenderObjects(void) {
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	// Child object (teapot) ... relative transform, and render
	glTranslatef(-g_transX, g_transY, 0.0);
	glRotatef(g_angleX, 1, 0, 0);
	glRotatef(g_angleY, 0, 1, 0);

	glMaterialfv(GL_FRONT, GL_DIFFUSE, colorBronzeDiff);
	glMaterialfv(GL_FRONT, GL_SPECULAR, colorBronzeSpec);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
	glColor4fv(colorBronzeDiff);

	//glutSolidTeapot(0.3);
	for (int i = 0; i < patches.size(); i+= 1) patches[i].drawPatchSimple(!adaptive);

	glPopMatrix();
}

void display(void) {
	// Clear frame buffer and depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Set up viewing transformation, looking down -Z axis
	glLoadIdentity();
	gluLookAt(0, 0, -g_fViewDistance, 0, 0, -1, 0, 1, 0);
	// Set up the stationary light
	glLightfv(GL_LIGHT0, GL_POSITION, g_lightPos);
	//	 glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

	// Render the scene
	RenderObjects();
	// Make sure changes appear onscreen
	glutSwapBuffers();
}

void reshape(GLint width, GLint height) {
	g_Width = width;
	g_Height = height;
	glViewport(0, 0, g_Width, g_Height);
	glMatrixMode(GL_PROJECTION);    // edit the projection matrix
	glLoadIdentity();
	gluPerspective(65.0, (float)g_Width / g_Height, g_nearPlane, g_farPlane);
	glMatrixMode(GL_MODELVIEW);
}

void InitGraphics(void) {
	int width, height;
	int nComponents;
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
}

void MouseButton(int button, int state, int x, int y) {
	// Respond to mouse button presses.
	// If button1 pressed, mark this state so we know in motion function.
	if (button == GLUT_LEFT_BUTTON) {
		g_bButton1Down = (state == GLUT_DOWN) ? true : false;
		g_fInitViewDistance = g_fViewDistance;
		g_yClick = y;
	}
}

void MouseMotion(int x, int y) {
	// If button1 pressed, zoom in/out if mouse is moved up/down.
	if (g_bButton1Down) {
		g_fViewDistance = g_fInitViewDistance + (y - g_yClick) / 20.0;
		if (g_fViewDistance < VIEWING_DISTANCE_MIN)
			g_fViewDistance = VIEWING_DISTANCE_MIN;
		glutPostRedisplay();
	}
}

void AnimateScene(void) {
	glutPostRedisplay();
}

void KeyBoardArrows (int key, int x, int y) {
	switch(key) {
	case GLUT_KEY_LEFT:
		if (glutGetModifiers() == GLUT_ACTIVE_SHIFT) { // translate
			g_transX -= 0.1;
		} else {           // rotate
			g_angleY-= 5;
		}
		break;
	case GLUT_KEY_RIGHT:
		if (glutGetModifiers() == GLUT_ACTIVE_SHIFT) { // translate
			g_transX += 0.1;
		} else {           // rotate
			g_angleY += 5;
		}
		break;
	case GLUT_KEY_UP:
		if (glutGetModifiers() == GLUT_ACTIVE_SHIFT) { // translate
			g_transY += 0.1;
		} else {            // rotate
			g_angleX += 5;
		}
		break;
	case GLUT_KEY_DOWN:
		if (glutGetModifiers() == GLUT_ACTIVE_SHIFT) { // translate
			g_transY -= 0.1;
		} else {            // rotate
			g_angleX -= 5;
		}
		break;
	}
	glutPostRedisplay();
}


void Keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 27:  // ESCAPE
		exit (0);
		break;
	case 'l': // light/ no light
		g_bLightingEnabled = !g_bLightingEnabled;
		if (g_bLightingEnabled)
			glEnable(GL_LIGHTING);
		else
			glDisable(GL_LIGHTING);
		break;
	case 'w':  // filled/ wireframe
		g_bFillPolygons = !g_bFillPolygons;
		glPolygonMode (GL_FRONT_AND_BACK, g_bFillPolygons ? GL_FILL : GL_LINE);
		break;
	case 's':  // smooth/ flat shading
		g_bSmoothShading = !g_bSmoothShading;
		glShadeModel(g_bSmoothShading? GL_SMOOTH : GL_FLAT);
		break;
	case '+': // zoom in
		g_fViewDistance = g_fViewDistance - 0.05;
		if (g_fViewDistance < VIEWING_DISTANCE_MIN)
			g_fViewDistance = VIEWING_DISTANCE_MIN;
		break;
	case '-': // zoom in
		g_fViewDistance = g_fViewDistance + 0.05;
		break;
	case 'a': // sampling - toggle between adaptive and uniform
		adaptive = !adaptive;
		cout<<" Sampling mode: "<<(adaptive? "Adaptive" : "Uniform")<<endl;
		break;
	}

	glutPostRedisplay();
}


int main(int argc, char** argv) {

	// read and tesselate the bezier patches
	string fname = EXPAND (PROJECT_DATA_DIR) "/teapot.bez";
	patches = intializePatchesFromCommandLine (argc, argv, adaptive);

	for (int i = 0; i < patches.size(); i+=1 ) {
		patches[i].sampleUniformly();
		patches[i].adaptiveSample();
	}

	// GLUT Window Initialization:
	glutInit (&argc, argv);
	glutInitWindowSize (g_Width, g_Height);
	glutInitDisplayMode ( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow ("CS184  - Bezier Subdivision");

	// initialize OpenGL graphics state
	InitGraphics();

	// register callbacks:
	glutDisplayFunc(display);
	glutReshapeFunc  (reshape);
	glutKeyboardFunc(Keyboard);
	glutSpecialFunc(KeyBoardArrows);  // to handle arrow keys
	glutMouseFunc(MouseButton);
	glutMotionFunc(MouseMotion);
	glutIdleFunc(AnimateScene);

	// Turn the flow of control over to GLUT
	glutMainLoop();
	return 0;
}
