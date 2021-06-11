#ifndef CASD_SURFRECON_HPP
#define CASD_SURFRECON_HPP

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

vector<vector<Vector3f>> ReadOffSetTable(string filename);

class Vertex
{
public:
	shared_ptr<Vector3f> point = nullptr;
	shared_ptr<Vector3f> normal = nullptr;
	Vertex() {}
	~Vertex() {};
};

class Face
{
public:

};

void Visualization(int argc, char* argv[]);
void DrawShip();

#endif
