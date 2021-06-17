#ifndef EIGEN_INITIALIZE_MATRICES_BY_ZERO
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO
#endif

// #include <memory>

#define Cos(th) cos(acos(-1.0)/180.0*(th))
#define Sin(th) sin(acos(-1.0)/180.0*(th))

#include <cstdarg>
// #include <GL/glut.h>
#include <iostream>
#include "casd_pa2.h"

using namespace std;
using namespace CASD_PA2;

void DrawShip();

int main(int argc, char* argv[])
{
	auto offsettabledata = ReadOffSetTable("offset_table_rev.txt");
	auto model = BuildTopology(offsettabledata);
	ReconstructSurface(model);
	GL_Object::ship_model = model;
	Visualization(argc, argv);
}

void DrawShip(/*do not add parameter here*/)
{
	DrawAllSurface(GL_Object::ship_model);
	// DrawVertexes(GL_Object::ship_model);
	// DrawCP(GL_Object::ship_model);
}