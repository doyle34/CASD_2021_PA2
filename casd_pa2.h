#pragma once
#define N_MESH_DIV 2

#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <sstream>
// #include <memory>
#include <Eigen/Dense>
#include <GL/glut.h>

using namespace Eigen;
using namespace std;

void DrawShip();

namespace CASD_PA2
{
	// Geometry Structures
	class CubicCurve;
	class CubicSurface;

	// Topology Structures
	class Vertex;
	enum FaceType;
	class Face;
	class WingedEdge;
	class TopologyModel
	{
	public:
		vector<shared_ptr<Vertex>> vertices;
		vector<shared_ptr<WingedEdge>> edges;
		vector<shared_ptr<Face>> faces;
		TopologyModel() {};
		~TopologyModel() {};
	};

	// Visualization Structures
	struct GL_Object
	{
		static inline TopologyModel ship_model;
	};
	
	TopologyModel BuildTopology(vector<vector<Vector3f>> offsettable);
	vector<vector<FaceType>> EvaluateFaceTypes(vector<vector<Vector3f>> datapoints);
	void InitializeTopology(const vector<vector<Vector3f>>& offsettable,
		const vector<vector<FaceType>>& facetype,
		vector<vector<shared_ptr<Vertex>>>& vertex_arr,
		vector<vector<array<shared_ptr<WingedEdge>, 4>>>& edge_arr,
		vector<vector<shared_ptr<Face>>>& face_arr);
	void BuildConnection(const vector<vector<FaceType>>& facetype,
		vector<vector<shared_ptr<Vertex>>>& vertex_arr,
		vector<vector<array<shared_ptr<WingedEdge>, 4>>>& edge_arr,
		vector<vector<shared_ptr<Face>>>& face_arr);
	void SetFaceEdges(TopologyModel& model);

	Vector3f DeCasteljau(CubicCurve c, float u);
	Vector3f DeCasteljau(CubicSurface s, float u, float v);

	void ReconstructSurface(TopologyModel& model);

	shared_ptr<Vector3f> ComputeANormal(
		const shared_ptr<WingedEdge>& edge,
		const shared_ptr<Vertex>& vertex);
	void ComputeAllNormal(TopologyModel& model);
	void ComputeTriSurfaceCP(shared_ptr<Face>& face);
	void ComputeQuadSurfaceCP(shared_ptr<Face>& face);
	void ComputeCurveCP(shared_ptr<WingedEdge>& edge);
	void ComputeAllFaceSurface(TopologyModel& model);

	vector<vector<Vector3f>> ReadOffSetTable(string filename);

	void DrawSurface(shared_ptr<Face>& face);

	void DrawAllSurface(const TopologyModel& model);

	void DrawVertexes(const TopologyModel& model);

	void DrawCP(const TopologyModel& model);

	// void DrawShip();

	void Visualization(int argc, char* argv[]);
}