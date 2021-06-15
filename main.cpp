#ifndef EIGEN_INITIALIZE_MATRICES_BY_ZERO
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO
#endif

// #include <memory>
#include "CASD_SurfaceReconstruction.hpp"

#define Cos(th) cos(acos(-1.0)/180.0*(th))
#define Sin(th) sin(acos(-1.0)/180.0*(th))

#include <cstdarg>
#include <GL/glut.h>

#include <iostream>
using namespace std;
// using namespace HashColon::CAGD;

void ReadFile()
{

	/* read file here */	
}

void PrepareModel()
{
	/* build topology model & reconstruct surface */
}

// DrawShape is a callback function for glDisplayFunc()
// This function should be in form (void)func()
void DrawShip(/*do not add parameter here*/)
{
	/* draw ship hull here with 
	* glBegin(GL_Triangles),
	* glVertex3f, glVertex3d, 
	* glEnd()
	*/

	/* sample code !!*/
	glBegin(GL_TRIANGLES);
	glColor3f(1.0, 0.0, 0.75);

	glVertex3f(6.6f, 0.0f, 4.0f);
	glVertex3f(8.25f, 0.265f, 5.0f);
	glVertex3f(6.6f, 0.0f, 5.0f);

	glVertex3f(8.25f, 0.265f, 5.0f);
	glVertex3f(6.6f, 0.0f, 4.0f);
	glVertex3f(8.25f, 0.4665f, 4.0f);

	glVertex3f(6.6f, 0.0f, 5.0f);
	glVertex3f(8.25f, 0.265f, 5.0f);
	glVertex3f(8.25f, 0.0f, 6.0f);

	glVertex3d(-1, 1, 0);
	glVertex3d(1, 1, 0);
	glVertex3d(0, -1, 0);

	glEnd();
}

int main(int argc, char* argv[])
{
	// ReadFile();
	auto offsettabledata = ReadOffSetTable("offset_table_rev.txt");
	PrepareModel();
	Visualization(argc, argv);
}