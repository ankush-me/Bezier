// glut_example.c
// Stanford University, CS248, Fall 2000
//
// Demonstrates basic use of GLUT toolkit for CS248 video game assignment.
// More GLUT details at http://reality.sgi.com/mjk_asd/spec3/spec3.html
// Here you'll find examples of initialization, basic viewing transformations,
// mouse and keyboard callbacks, menus, some rendering primitives, lighting,
// double buffering, Z buffering, and texturing.
//
// Matt Ginzton -- magi@cs.stanford.edu

#include <GL/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "BezierPatch.h"
//#include "texture.h"

using namespace std;

#define VIEWING_DISTANCE_MIN  8.9
#define TEXTURE_ID_CUBE 1
enum {
	MENU_LIGHTING = 1,
	MENU_POLYMODE,
	MENU_TEXTURING,
	MENU_EXIT
};

typedef int BOOL;
#define TRUE 1
#define FALSE 0
static BOOL g_bLightingEnabled = TRUE;
static BOOL g_bFillPolygons    = TRUE;
static BOOL g_bTexture         = FALSE;
static BOOL g_bButton1Down     = FALSE;
static GLfloat g_fTeapotAngle  = 0.0;
static GLfloat g_fTeapotAngle2 = 0.0;
static GLfloat g_fInitViewDistance = VIEWING_DISTANCE_MIN;
static GLfloat g_fViewDistance = 1.2*VIEWING_DISTANCE_MIN;
static GLfloat g_nearPlane     = 1;
static GLfloat g_farPlane      = 1000;
static int g_Width            = 600;                          // Initial window width
static int g_Height           = 600;                         // Initial window height
static int g_yClick            = 0;
static float g_lightPos[4] = { 10, 10, -100, 1 };  // Position of light

void RenderObjects(void)
{
	float colorBronzeDiff[4] = { 0.8, 0.6, 0.0, 1.0 };
	float colorBronzeSpec[4] = { 1.0, 1.0, 0.4, 1.0 };
	float colorBlue[4]       = { 0.0, 0.2, 1.0, 1.0 };
	float colorNone[4]       = { 0.0, 0.0, 0.0, 0.0 };
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	// Main object (cube) ... transform to its coordinates, and render
	glColor4fv(colorBlue);
	// glBindTexture(GL_TEXTURE_2D, TEXTURE_ID_CUBE);
	// Child object (teapot) ... relative transform, and render
	glTranslatef(0, 0, -7.5);
	glRotatef(g_fTeapotAngle2, 1, 1, 0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, colorBronzeDiff);
	glMaterialfv(GL_FRONT, GL_SPECULAR, colorBronzeSpec);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
	glColor4fv(colorBronzeDiff);
	// glBindTexture(GL_TEXTURE_2D, 0);
	glutSolidTeapot(0.3);
	glPopMatrix();
}

void display(void)
{
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

void reshape(GLint width, GLint height)
{
	g_Width = width;
	g_Height = height;
	glViewport(0, 0, g_Width, g_Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(65.0, (float)g_Width / g_Height, g_nearPlane, g_farPlane);
	glMatrixMode(GL_MODELVIEW);
}

void InitGraphics(void)
{
	int width, height;
	int nComponents;
	//void* pTextureImage;
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	// Create texture for cube; load marble texture from file and bind it
	// pTextureImage = read_texture("marble.rgb", &width, &height, &nComponents);
	// glBindTexture(GL_TEXTURE_2D, TEXTURE_ID_CUBE);
	//gluBuild2DMipmaps(GL_TEXTURE_2D,     // texture to specify
	//                 GL_RGBA,           // internal texture storage format
	//                 width,             // texture width
	//                 height,            // texture height
	//                GL_RGBA,           // pixel format
	//                 GL_UNSIGNED_BYTE,	// color component format
	//                 pTextureImage);    // pointer to texture image
	// glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
	//                 GL_LINEAR_MIPMAP_LINEAR);
	// glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
}

void MouseButton(int button, int state, int x, int y)
{
	// Respond to mouse button presses.
	// If button1 pressed, mark this state so we know in motion function.
	if (button == GLUT_LEFT_BUTTON)
	{
		g_bButton1Down = (state == GLUT_DOWN) ? TRUE : FALSE;
		g_fInitViewDistance = g_fViewDistance;
		g_yClick = y;
	}
}

void MouseMotion(int x, int y)
{
	// If button1 pressed, zoom in/out if mouse is moved up/down.
	if (g_bButton1Down)
	{
		g_fViewDistance = g_fInitViewDistance + (y - g_yClick) / 20.0;
		cout<< g_fViewDistance << endl;
		if (g_fViewDistance < VIEWING_DISTANCE_MIN)
			g_fViewDistance = VIEWING_DISTANCE_MIN;
		glutPostRedisplay();
	}
}

void AnimateScene(void)
{
	glutPostRedisplay();
}

void SelectFromMenu(int idCommand)
{
	switch (idCommand)
	{
	case MENU_LIGHTING:
		g_bLightingEnabled = !g_bLightingEnabled;
		if (g_bLightingEnabled)
			glEnable(GL_LIGHTING);
		else
			glDisable(GL_LIGHTING);
		break;
	case MENU_POLYMODE:
		g_bFillPolygons = !g_bFillPolygons;
		glPolygonMode (GL_FRONT_AND_BACK, g_bFillPolygons ? GL_FILL : GL_LINE);
		break;
		//case MENU_TEXTURING:
			//  g_bTexture = !g_bTexture;
		//  if (g_bTexture)
		//     glEnable(GL_TEXTURE_2D);
		//  else
		//     glDisable(GL_TEXTURE_2D);
		//  break;
	case MENU_EXIT:
		exit (0);
		break;
	}
	// Almost any menu selection requires a redraw
	glutPostRedisplay();
}

void Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:             // ESCAPE key
		exit (0);
		break;
	case 'l':
		SelectFromMenu(MENU_LIGHTING);
		break;
	case 'p':
		SelectFromMenu(MENU_POLYMODE);
		break;
	case 't':
		SelectFromMenu(MENU_TEXTURING);
		break;
	}
}

int BuildPopupMenu (void)
{
	int menu;
	menu = glutCreateMenu (SelectFromMenu);
	glutAddMenuEntry ("Toggle lighting\tl", MENU_LIGHTING);
	glutAddMenuEntry ("Toggle polygon fill\tp", MENU_POLYMODE);
	//glutAddMenuEntry ("Toggle texturing\tt", MENU_TEXTURING);
	glutAddMenuEntry ("Exit demo\tEsc", MENU_EXIT);
	return menu;
}

int main(int argc, char** argv)
{
	// GLUT Window Initialization:
	glutInit (&argc, argv);
	glutInitWindowSize (g_Width, g_Height);
	glutInitDisplayMode ( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow ("CS248 GLUT example");
	// Initialize OpenGL graphics state
	InitGraphics();
	// Register callbacks:
	glutDisplayFunc (display);
	glutReshapeFunc (reshape);
	glutKeyboardFunc (Keyboard);
	glutMouseFunc (MouseButton);
	glutMotionFunc (MouseMotion);
	glutIdleFunc (AnimateScene);
	// Create our popup menu
	BuildPopupMenu ();
	//glutAttachMenu (GLUT_RIGHT_BUTTON);
	// Turn the flow of control over to GLUT
	glutMainLoop ();
	return 0;
}
