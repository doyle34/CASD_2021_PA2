#ifndef CASD_SURFRECON_HPP
#define CASD_SURFRECON_HPP

#include <string>
#include <vector>
#include <array>
// #include <memory>
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
		enum { x = 0, y = 1, z = 2 };
		Vertex() {};
		Vertex(Vector3f p)
		{
			this->point = make_shared<Vector3f>(p);
		}
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
		FaceType facetype = NULLFACE;
		vector<shared_ptr<WingedEdge>> edge;
		shared_ptr<Geometry::CubicSurface> surface = nullptr;
		Face() {};
		Face(FaceType ty)
		{
			if (ty == QUADFACE)
			{
				this->edge.resize(4);
				this->facetype = ty;
			}
			else if (ty == NULLFACE) this->facetype = ty;
			else
			{
				this->edge.resize(3);
				this->facetype = ty;
			}
		}
		~Face() {};
	};

	class WingedEdge
	{
	public:
		array<shared_ptr<Vertex>, 2> vertex = { { nullptr, nullptr } };
		array<shared_ptr<WingedEdge>, 4> edge = { { nullptr, nullptr, nullptr, nullptr } };
		array<shared_ptr<Face>, 2> face = { { nullptr, nullptr } };
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

	enum FaceType
	{
		QUADFACE,
		UPPER_LEFT_TRIFACE,
		LOWER_LEFT_TRIFACE,
		UPPER_RIGHT_TRIFACE,
		LOWER_RIGHT_TRIFACE,
		NULLFACE
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
