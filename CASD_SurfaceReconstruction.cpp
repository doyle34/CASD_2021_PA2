#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include "CASD_SurfaceReconstruction.hpp"


using namespace Eigen;
using namespace std;
using namespace Geometry;

vector<vector<Vector3f>> ReadOffSetTable(string filename)
{
	ifstream in(filename);

	vector<vector<float>> ydata;
	vector<float> xdata;
	vector<float> zdata;
	int i = 0;
	for (string line; getline(in, line); i++)
	{
		stringstream ss(line);
		if (i == 0)
			while (!ss.eof())
			{
				float val;
				ss >> val;
				zdata.push_back(val);
			}
		else
		{
			ydata.push_back(vector<float>());
			float x;
			ss >> x;
			xdata.push_back(x);
			while (!ss.eof())
			{
				float val;
				ss >> val;
				ydata.back().push_back(val);
			}
		}

	}

	vector<vector<Vector3f>> offsettabledata(xdata.size(), vector<Vector3f>(zdata.size()));

	for (int i = 0; i < xdata.size(); i++)
	{
		for (int j = 0; j < zdata.size(); j++)
		{
			offsettabledata[i][j] = Vector3f(xdata[i], ydata[i][j], zdata[j]);
		}
	}

	return offsettabledata;

}

namespace Topology
{

	TopologyModel BuildTopology(vector<vector<Vector3f>> offsettable)
	{
		int n = offsettable.size();
		int m = offsettable[0].size();
		
		enum { x = 0, y = 1, z = 2 };
		vector<vector<shared_ptr<Vertex>>> vertex_arr(n,
			vector<shared_ptr<Vertex>>(m, nullptr));
		enum { d = 0, r, ru, rd };
		vector<vector<array<shared_ptr<WingedEdge>, 4>>> edge_arr(n,
			vector<array<shared_ptr<WingedEdge>, 4>>(m,
				array<shared_ptr<WingedEdge>, 4>({ nullptr, nullptr, nullptr, nullptr })));

		vector<vector<shared_ptr<Face>>> face_arr(n,
			vector<shared_ptr<Face>>(m, nullptr));

		vector<vector<FaceType>> facetype = EvaluateFaceTypes(offsettable);
		InitializeTopology(offsettable, facetype, vertex_arr, edge_arr, face_arr);

	}

	vector<vector<FaceType>> EvaluateFaceTypes(vector<vector<Vector3f>> datapoints)
	{
		enum { x = 0, y = 1, z = 2 };
		int n = datapoints.size();
		int m = datapoints[0].size();
		vector<vector<FaceType>> facetypes(n, vector<FaceType>(m));

		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < m; j++)
			{
				int is_zero[4] = { 0, 0, 0, 0 };
				int zero_count = 0;
				is_zero[0] = (datapoints[i][j](y) == 0.0 ? 1 : 0);
				is_zero[1] = (i + 1 < n ? (datapoints[i + 1][j](y) == 0.0 ? 1 : 0) : 0);
				is_zero[2] = (j + 1 < m ? (datapoints[i][j + 1](y) == 0.0 ? 1 : 0) : 0);
				is_zero[3] = ((i + 1 < n && j + 1 < m) ? (datapoints[i + 1][j + 1](y) == 0.0 ? 1 : 0) : 0);
				for (int idx = 0; idx < 4; idx++)
					zero_count += is_zero[idx];

				if (zero_count < 2) facetypes[i][j] = QUADFACE;
				else if (zero_count == 3)
				{
					if (is_zero[0]) facetypes[i][j] = LOWER_RIGHT_TRIFACE;
					else if (is_zero[1]) facetypes[i][j] = UPPER_RIGHT_TRIFACE;
					else if (is_zero[2]) facetypes[i][j] = LOWER_LEFT_TRIFACE;
					else facetypes[i][j] = UPPER_LEFT_TRIFACE;
				}
				else facetypes[i][j] = NULLFACE;
			}
		}

		return facetypes;
	}

	void InitializeTopology(const vector<vector<Vector3f>> &offsettable,
		const vector<vector<FaceType>> &facetype, 
		vector<vector<shared_ptr<Vertex>>> &vertex_arr,
		vector<vector<array<shared_ptr<WingedEdge>, 4>>> &edge_arr,
		vector<vector<shared_ptr<Face>>> &face_arr)
	{
		int n = vertex_arr.size();
		int m = vertex_arr[0].size();
		enum { d = 0, r, ru, rd };

		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < m; j++)
			{
				bool LOWER_LIMIT = i + 1 >= n;
				bool RIGHT_LIMIT = j + 1 >= m;


				switch (facetype[i][j])
				{
				case QUADFACE:
				{
					if (!vertex_arr[i][j])
						make_shared<Vertex>(offsettable[i][j]);
					
					if (!LOWER_LIMIT)
					{
						vertex_arr[i + 1][j] = vertex_arr[i + 1][j] ? vertex_arr[i + 1][j] : make_shared<Vertex>(offsettable[i + 1][j]);
						edge_arr[i][j][d] = make_shared<WingedEdge>();
					}

					if (!RIGHT_LIMIT)
					{
						vertex_arr[i][j + 1] = vertex_arr[i][j + 1] ? vertex_arr[i][j + 1] : make_shared<Vertex>(offsettable[i][j + 1]);
						edge_arr[i][j][r] = make_shared<WingedEdge>();
					}

					if (!LOWER_LIMIT && !RIGHT_LIMIT)
					{
						vertex_arr[i + 1][j + 1] = vertex_arr[i + 1][j + 1] ? vertex_arr[i + 1][j + 1] : make_shared<Vertex>(offsettable[i + 1][j + 1]);
						edge_arr[i][j + 1][d] = make_shared<WingedEdge>();
						edge_arr[i + 1][j][r] = make_shared<WingedEdge>();
						face_arr[i][j] = make_shared<Face>(QUADFACE);
					}

				} break;

				case UPPER_LEFT_TRIFACE:
				{
					if (!vertex_arr[i][j])
						make_shared<Vertex>(offsettable[i][j]);

					if (!LOWER_LIMIT)
					{
						vertex_arr[i + 1][j] = vertex_arr[i + 1][j] ? vertex_arr[i + 1][j] : make_shared<Vertex>(offsettable[i + 1][j]);
						edge_arr[i][j][d] = make_shared<WingedEdge>();
					}

					if (!RIGHT_LIMIT)
					{
						vertex_arr[i][j + 1] = vertex_arr[i][j + 1] ? vertex_arr[i][j + 1] : make_shared<Vertex>(offsettable[i][j + 1]);
						edge_arr[i][j][r] = make_shared<WingedEdge>();
					}

					if (!LOWER_LIMIT && !RIGHT_LIMIT)
					{
						edge_arr[i][j][ru] = make_shared<WingedEdge>();
						face_arr[i][j] = make_shared<Face>(UPPER_LEFT_TRIFACE);
					}


				} break;

				case LOWER_LEFT_TRIFACE:
				{
					if (!vertex_arr[i][j])
						make_shared<Vertex>(offsettable[i][j]);

					if (!LOWER_LIMIT)
					{
						vertex_arr[i + 1][j] = vertex_arr[i + 1][j] ? vertex_arr[i + 1][j] : make_shared<Vertex>(offsettable[i + 1][j]);
						edge_arr[i][j][d] = make_shared<WingedEdge>();

						if (!RIGHT_LIMIT)
						{
							vertex_arr[i + 1][j + 1] = vertex_arr[i + 1][j + 1] ? vertex_arr[i + 1][j + 1] : make_shared<Vertex>(offsettable[i + 1][j + 1]);
							edge_arr[i][j][rd] = make_shared<WingedEdge>();
							edge_arr[i + 1][j][r] = make_shared<WingedEdge>();
							face_arr[i][j] = make_shared<Face>(LOWER_LEFT_TRIFACE);
						}
					}

				} break;

				case UPPER_RIGHT_TRIFACE:
				{
					vertex_arr[i][j] = vertex_arr[i][j] ? vertex_arr[i][j] : make_shared<Vertex>(offsettable[i][j]);
					if (!RIGHT_LIMIT)
					{
						vertex_arr[i][j + 1] = vertex_arr[i][j + 1] ? vertex_arr[i][j + 1] : make_shared<Vertex>(offsettable[i][j + 1]);
						edge_arr[i][j][r] = make_shared<WingedEdge>();

						if (!LOWER_LIMIT)
						{
							vertex_arr[i + 1][j + 1] = vertex_arr[i + 1][j + 1] ? vertex_arr[i + 1][j + 1] : make_shared<Vertex>(offsettable[i + 1][j + 1]);
							edge_arr[i][j][rd] = make_shared<WingedEdge>();
							edge_arr[i][j + 1][d] = make_shared<WingedEdge>();
							face_arr[i][j] = make_shared<Face>(UPPER_RIGHT_TRIFACE);
						}
					}

				} break;

				case LOWER_RIGHT_TRIFACE:
				{
					if (!LOWER_LIMIT)
					{
						vertex_arr[i + 1][j] = vertex_arr[i + 1][j] ? vertex_arr[i + 1][j] : make_shared<Vertex>(offsettable[i + 1][j]);
					}

					if (!RIGHT_LIMIT)
					{
						vertex_arr[i][j + 1] = vertex_arr[i][j + 1] ? vertex_arr[i][j + 1] : make_shared<Vertex>(offsettable[i][j + 1]);
					}

					if (!LOWER_LIMIT && !RIGHT_LIMIT)
					{
						vertex_arr[i + 1][j + 1] = vertex_arr[i + 1][j + 1] ? vertex_arr[i + 1][j + 1] : make_shared<Vertex>(offsettable[i + 1][j + 1]);
						edge_arr[i][j][ru] = make_shared<WingedEdge>();
						edge_arr[i][j + 1][d] = make_shared<WingedEdge>();
						edge_arr[i + 1][j][r] = make_shared<WingedEdge>();
						face_arr[i][j] = make_shared<Face>(LOWER_RIGHT_TRIFACE);
					}

				} break;
				}
			}
		}
	}

	shared_ptr<Vertex> CheckVertexAndMake(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2, Vector3f datapoint)
	{
		if (v1 == v2)
		{

		}
	}

	void BuildConnection(const vector<vector<FaceType>>& facetype,
		vector<vector<shared_ptr<Vertex>>>& vertex_arr,
		vector<vector<array<shared_ptr<WingedEdge>, 4>>>& edge_arr,
		vector<vector<shared_ptr<Face>>>& face_arr)
	{


	}


}
