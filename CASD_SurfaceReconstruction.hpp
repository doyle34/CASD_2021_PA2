#ifndef CASD_SURFRECON_HPP
#define CASD_SURFRECON_HPP

#include <string>
#include <vector>
#include <array>
#include <memory>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
using namespace Geometry;

vector<vector<Vector3f>> ReadOffSetTable(string filename);


namespace Topology
{

	class Vertex
	{
	public:
		shared_ptr<Vector3f> point = nullptr;
		shared_ptr<Vector3f> normal = nullptr;
		Vertex() {};
		~Vertex() {};
	};

	class Edge
	{
	public:
		shared_ptr<Geometry::CubicCurve> curve = nullptr;
		array<shared_ptr<Vertex>, 2> vertex = { { nullptr, nullptr } };
		Edge() {};
		~Edge() {};
	};

	class Face
	{
	public:

	};

	class TriFace : Face
	{
	public:
		array<shared_ptr<Edge>, 3> edge = { { nullptr, nullptr, nullptr } };
		shared_ptr<Geometry::CubicSurface> surface = nullptr;

	};

	class QuadFace : Face
	{
	public:
		array<shared_ptr<Edge>, 4> edge = { { nullptr, nullptr, nullptr, nullptr } };
		shared_ptr<Geometry::CubicSurface> surface = nullptr;

	};

	class WingedEdge
	{
	public:
		shared_ptr<Edge> selfedge = nullptr;
		array<shared_ptr<Vertex>, 2> vertex = { { nullptr, nullptr } };
		enum { Start = 0, End = 1 };
		array<shared_ptr<WingedEdge>, 4> edge = { { nullptr, nullptr, nullptr, nullptr } };
		enum { LeftPrev = 0, LeftNext = 1, RightPrev = 2, RightNext = 3 };
		WingedEdge() {};
		~WingedEdge() {};
	};

	class TopologyModel
	{
	public:
		vector<shared_ptr<Vertex>> vertices;
		vector<shared_ptr<WingedEdge>> edges;
		vector<shared_ptr<Face>> faces;
		TopologyModel() {};
		~TopologyModel() {};
	};
}


namespace Geometry
{
	class CubicCurve
	{
	public:
		vector<shared_ptr<Vector3f>> control_points;

		shared_ptr<Vector3f> ComputePointOnCurve();
		// control points
		// compute point on curve
	};

	class CubicSurface
	{
		vector<vector<shared_ptr<Vector3f>>> control_points;

		shared_ptr<Vector3f> ComputePointOnSurface();
	};
}

void Visualization(int argc, char* argv[]);
void DrawShip();

#endif
