#ifndef EIGEN_INITIALIZE_MATRICES_BY_ZERO
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO
#endif

#define Cos(th) cos(acos(-1.0)/180.0*(th))
#define Sin(th) sin(acos(-1.0)/180.0*(th))

#include <cstdarg>
#include <GL/glut.h>
#include <iostream>

#include "CASD_SurfaceReconstruction.hpp"
#define LEN 8192 //length of char array in C array

using namespace std;
// using namespace HashColon::CAGD;

struct XYZ { float x; float y; float z; };
struct RThetaPhi { float r; float theta; float phi; };
struct VisualizationOptions
{
	struct {
		// View mode type: Perspective / Orthogonal
		enum ViewModeType {Perspective = 0, Orthogonal = 1};
		size_t ViewMode = ViewModeType::Perspective;

		// View point: position of camera
		RThetaPhi ViewPoint{ 100.0 , 0.0, 0.0 };
		// Focus point: point where camera looks at
		XYZ FocusPoint{ 0.0 , 0.0, 0.0 };
		// Translattion
		XYZ Translation{ 0.0, 0.0, 0.0 };
		// Angle of view
		float FovAngle = 60; 
		float AspectRatio = 1.0;		
	} Camera;

	struct {		
		int width = 1920;
		int height = 1080 - 100;
		string windowName = "CASD Offset Table Surface Reconstruction";
	} Window;
	
	bool ShowAxes = true;	
	bool ShowScripts = true;

	struct {
		int x;
		int y;		
		int button;
	} Mouse;
		
} VisOpt;

void printv(va_list args, const char* format)
{
	char buf[LEN];
	char* ch = buf;

	vsnprintf(buf, LEN, format, args);

	while (*ch)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *ch++);
	}
}

void print(const char* format, ...)
{
	va_list args;
	va_start(args, format);
	printv(args, format);
	va_end(args);
}

void printAt(int x, int y, const char* format, ...)
{
	va_list args;

	int ix = (int)x; //Integer versions of x and y
	int iy = (int)y;

	glPushAttrib(GL_TRANSFORM_BIT | GL_VIEWPORT_BIT); //Save transform attributes (Matrix Mode and Viewport)

	glMatrixMode(GL_PROJECTION); //Save projection matrix and set identity
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW); //Save model view matrix and set to indentity
	glPushMatrix();
	glLoadIdentity();

	glViewport(ix - 1, iy - 1, 2, 2); //Set viewport to 2x2 pixels around (x,y)
	glRasterPos4f(x - ix, y - iy, 0, 1); //Finally set the raster position
	glPopMatrix(); //Reset model view matrix

	glMatrixMode(GL_PROJECTION); //Reset projection matrix
	glPopMatrix();
	glPopAttrib(); //Pop transform attributes (Matrix Mode and Viewport)

	va_start(args, format);
	printv(args, format);
	va_end(args);
}

void Project()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	auto& camera = VisOpt.Camera;
	auto& eye = VisOpt.Camera.ViewPoint;
	auto& trans = VisOpt.Camera.Translation;

	// Perspective mode camera settings
	if (camera.ViewMode == camera.Perspective)
	{
		gluPerspective(
			camera.FovAngle, camera.AspectRatio,
			eye.r / 4.0, 4.0 * eye.r); // perspective 		// up direction vector		

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glTranslatef(trans.x, trans.y, 0.0);
	}
	else
	{
		glOrtho(
			-eye.r * camera.AspectRatio - trans.x,
			+eye.r * camera.AspectRatio - trans.x,
			-eye.r - trans.y, +eye.r - trans.y, 
			-eye.r, +eye.r); // orthogonal projection

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}

	
}

void SetCamera()
{
	auto& camera = VisOpt.Camera;
	auto& eye = VisOpt.Camera.ViewPoint;
	auto& center = VisOpt.Camera.FocusPoint;
	
	// Perspective mode camera settings
	if (camera.ViewMode == camera.Perspective)
	{
		

		XYZ eyeXYZ
		{
			-2 * eye.r * Sin(eye.theta) * Cos(eye.phi),		// x
			+2 * eye.r * Sin(eye.phi),						// y
			+2 * eye.r * Cos(eye.theta) * Cos(eye.phi)		// z
		};
		gluLookAt(
			eyeXYZ.x, eyeXYZ.y, eyeXYZ.z,	// View point
			center.x, center.y, center.z,	// Focus point
			0, 1, 0);		
	}
	else
	{	
		glRotatef(eye.phi, 1, 0, 0);		// rotate along x-axis
		glRotatef(eye.theta, 0, 1, 0);		// rotate along y-axis		
	}
	
}

void DrawAxes()
{
	if (VisOpt.ShowAxes)
	{
		double len = 10.0; //  Length of axes 
		glColor3f(1.0, 1.0, 1.0);

		glBegin(GL_LINES);
		glVertex3d(0, 0, 0);
		glVertex3d(len, 0, 0);
		glVertex3d(0, 0, 0);
		glVertex3d(0, len, 0);
		glVertex3d(0, 0, 0);
		glVertex3d(0, 0, len);
		glEnd();

		//  Label axes 
		glRasterPos3d(len, 0, 0);
		print("X");

		glRasterPos3d(0, len, 0);
		print("Y");

		glRasterPos3d(0, 0, len);
		print("Z");
	}
}

void DrawScripts()
{
	auto& eye = VisOpt.Camera.ViewPoint;
	if (VisOpt.Camera.ViewMode)
	{
		//glDisable(GL_LIGHTING);
		glColor3f(0.8, 0.8, 0.8);
		printAt(5, 45, "Perspective mode =(%s)", VisOpt.Camera.ViewMode ? "Orthogonal" : "Perspective");
		printAt(5, 25, "View Angle (theta, phi) =(%f, %f)", eye.theta, eye.phi);
		printAt(5, 5, "Mouse: (%d, %d)", VisOpt.Mouse.x, VisOpt.Mouse.y);
	}
}

void Reshape(
	//AIM: GLUT calls this routine when the window is resized. 
	int width,
	int height
)
{
	auto& camera = VisOpt.Camera;
	auto& window = VisOpt.Window;
	camera.AspectRatio = (window.height > 0) ? (double)window.width / window.height : 1;

	glViewport(0, 0, window.width, window.height);
	Project();
}

void WindowSpecial(int key, int x, int y)
{

	if (key == GLUT_KEY_RIGHT) //  Right arrow key - increase azimuth by 5 degrees 
		VisOpt.Camera.ViewPoint.theta += 5;
	else if (key == GLUT_KEY_LEFT) //  Left arrow key - decrease azimuth by 5 degrees 
		VisOpt.Camera.ViewPoint.theta -= 5;
	else if (key == GLUT_KEY_UP) //  Up arrow key - increase elevation by 5 degrees 
		VisOpt.Camera.ViewPoint.phi += 5; 
	else if (key == GLUT_KEY_DOWN) //  Down arrow key - decrease elevation by 5 degrees 
		VisOpt.Camera.ViewPoint.phi -= 5;

	//  Keep angles to +/-360 degrees 
	VisOpt.Camera.ViewPoint.theta = (float)((int)VisOpt.Camera.ViewPoint.theta % 360);
	VisOpt.Camera.ViewPoint.phi = (float)((int)VisOpt.Camera.ViewPoint.phi % 360);
	

	Project();
	glutPostRedisplay();
}

void WindowKey(unsigned char key, int x, int y)
{
	if (key == 27) //  Exit on ESC 
		exit(0);
	else if (key == 'a' || key == 'A')
		VisOpt.ShowAxes = !VisOpt.ShowAxes;
	else if (key == 'v' || key == 'V')
		VisOpt.ShowScripts = !VisOpt.ShowScripts;
	else if (key == 'm' || key == 'M')
	{		
		VisOpt.Camera.ViewMode = 1 - VisOpt.Camera.ViewMode;	
	}
	else if (key == '-' && key > 1) //  Change field of view angle 
	{
		VisOpt.Camera.ViewPoint.r += 0.1;
		VisOpt.Camera.FovAngle += 1.0;		
	}	
	else if (key == '+' && key < 179)
	{
		VisOpt.Camera.ViewPoint.r -= 0.1;
		VisOpt.Camera.FovAngle -= 1.0;
	}
	else if (key == '8')
	{
		VisOpt.Camera.Translation.y += 0.1;		
	}
	else if (key == '2')
	{
		VisOpt.Camera.Translation.y -= 0.1;
	}
	else if (key == '4')
	{
		VisOpt.Camera.Translation.x -= 0.1;
	}
	else if (key == '6')
	{
		VisOpt.Camera.Translation.x += 0.1;
	}

	Project();
	glutPostRedisplay();
}

void WindowMouseEvent(int button, int state, int x, int y)
{
	VisOpt.Mouse.x = x;
	VisOpt.Mouse.y = y;

	if (state == GLUT_DOWN)
		VisOpt.Mouse.button = button;
	else
		VisOpt.Mouse.button = -1;
}

void WindowMouseMove(int x, int y)
{
	const int delta_x = x - VisOpt.Mouse.x;
	const int delta_y = y - VisOpt.Mouse.y;

	VisOpt.Mouse.x = x;
	VisOpt.Mouse.y = y;

	const float spinCoeff = 0.2;
	const float transCoeff = 0.005;

	switch (VisOpt.Mouse.button)
	{
	case GLUT_LEFT_BUTTON:
	{
		VisOpt.Camera.ViewPoint.theta += spinCoeff * delta_x;
		VisOpt.Camera.ViewPoint.phi += spinCoeff * delta_y;
	}break;
	case GLUT_MIDDLE_BUTTON:
	{
		VisOpt.Camera.Translation.x += transCoeff * delta_x;
		VisOpt.Camera.Translation.y -= transCoeff * delta_y;
	}break;
	}

	VisOpt.Camera.ViewPoint.theta = (float)((int)VisOpt.Camera.ViewPoint.theta % 360);
	VisOpt.Camera.ViewPoint.phi = (float)((int)VisOpt.Camera.ViewPoint.phi % 360);

	Project();
	glutPostRedisplay();
}



void WindowMenu(int value)
{
	WindowKey((unsigned char)value, 0, 0);
}

void RenderScene() 
{
	// clear scene
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// set camera
	SetCamera();

	// Draw things
	DrawAxes();
	DrawShip();
	DrawScripts();
	
	// flush & swap buffers
	glFlush();
	glutSwapBuffers();
}

void init_lighting_model()
{
	GLfloat mat_ambient[4] = { 0.03784, 0.30712, 0.03784, 1.0 };
	GLfloat mat_diffuse[4] = { 0.07568, 0.61424, 0.07568, 1.0 };
	GLfloat mat_specular[4] = { 0.633, 0.727811, 0.633, 1.0 };
	GLfloat mat_shininess = 3.0;


	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);


	GLfloat mat_ambient2[4] = { 0.33, 0.22, 0.03, 1.0 };
	GLfloat mat_diffuse2[4] = { 0.78, 0.57, 0.11, 1.0 };
	GLfloat mat_specular2[4] = { 0.99, 0.91, 0.81, 1.0 };
	GLfloat mat_shininess2 = 5.0;

	glMaterialfv(GL_BACK, GL_AMBIENT, mat_ambient2);
	glMaterialfv(GL_BACK, GL_DIFFUSE, mat_diffuse2);
	glMaterialfv(GL_BACK, GL_SPECULAR, mat_specular2);
	glMaterialf(GL_BACK, GL_SHININESS, mat_shininess2);

	GLfloat light_position[4] = { 0.0, 5.0, 5.0, 1.0 };
	GLfloat light_ambient_color[4] = { 0.23, 0.23, 0.23, 1.0 };
	GLfloat light_diffuse_color[4] = { 0.95, 0.95, 0.95, 1.0 };
	GLfloat light_specular_color[4] = { 0.95, 0.95, 0.95, 1.0 };

	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient_color);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse_color);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular_color);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	//draw both sides
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
}

void Visualization(int argc, char* argv[])
{
	// Initialization
	glutInit(&argc, argv);											// glut initialization
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);		// set display mode for window
	glutInitWindowSize(VisOpt.Window.width, VisOpt.Window.height);	// set initial window size
	glutCreateWindow(VisOpt.Window.windowName.c_str());				// open a new glut window

	// Define Rendering function
	glutDisplayFunc(RenderScene);

	// Other user defined rendering
	glutReshapeFunc(Reshape);			// callback when window size changed
	glutKeyboardFunc(WindowKey);		// callback when keyboard key input event
	glutSpecialFunc(WindowSpecial);		// callback when keyboard special key input event (such as arrow, etc...)
	glutMouseFunc(WindowMouseEvent);	// callback when mouse button event 
	glutMotionFunc(WindowMouseMove);	// callback when mouse move(while button down) event 	

	// Define menus bar items
	glutCreateMenu(WindowMenu);
	glutAddMenuEntry("Toggle Axes [a]", 'a');
	glutAddMenuEntry("Toggle Values [v]", 'v');
	glutAddMenuEntry("Toggle Mode [m]", 'm');
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	init_lighting_model();

	// Run GL. GL runs infinite loop until the program ends
	glutMainLoop();
}
