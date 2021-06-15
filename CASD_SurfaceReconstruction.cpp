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
		BuildConnection(facetype, vertex_arr, edge_arr, face_arr);

		TopologyModel model;
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < m; j++)
			{
				if (vertex_arr[i][j]) model.vertices.push_back(vertex_arr[i][j]);
				if (face_arr[i][j]) model.faces.push_back(face_arr[i][j]);
				for (int k = 0; k < 4; k++)
					if (edge_arr[i][j][k]) model.edges.push_back(edge_arr[i][j][k]);
			}
		}

		SetFaceEdges(model);

		return model;
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
					if (!vertex_arr[i][j]) vertex_arr[i][j] = make_shared<Vertex>(offsettable[i][j]);
					
					if (!LOWER_LIMIT)
					{
						if (!vertex_arr[i + 1][j]) vertex_arr[i + 1][j] = make_shared<Vertex>(offsettable[i + 1][j]);
						edge_arr[i][j][d] = make_shared<WingedEdge>();
					}

					if (!RIGHT_LIMIT)
					{
						if (!vertex_arr[i][j + 1]) vertex_arr[i][j + 1] = make_shared<Vertex>(offsettable[i][j + 1]);
						edge_arr[i][j][r] = make_shared<WingedEdge>();
					}

					if (!LOWER_LIMIT && !RIGHT_LIMIT)
					{
						if (!vertex_arr[i + 1][j + 1]) vertex_arr[i + 1][j + 1] = make_shared<Vertex>(offsettable[i + 1][j + 1]);
						edge_arr[i][j + 1][d] = make_shared<WingedEdge>();
						edge_arr[i + 1][j][r] = make_shared<WingedEdge>();
						face_arr[i][j] = make_shared<Face>(QUADFACE);
					}

				} break;

				case UPPER_LEFT_TRIFACE:
				{
					if (!vertex_arr[i][j]) vertex_arr[i][j] = make_shared<Vertex>(offsettable[i][j]);

					if (!LOWER_LIMIT)
					{
						if (!vertex_arr[i + 1][j]) vertex_arr[i + 1][j] = make_shared<Vertex>(offsettable[i + 1][j]);
						edge_arr[i][j][d] = make_shared<WingedEdge>();
					}

					if (!RIGHT_LIMIT)
					{
						if (!vertex_arr[i][j + 1]) vertex_arr[i][j + 1] = make_shared<Vertex>(offsettable[i][j + 1]);
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
					if (!vertex_arr[i][j]) vertex_arr[i][j] = make_shared<Vertex>(offsettable[i][j]);

					if (!LOWER_LIMIT)
					{
						if (!vertex_arr[i + 1][j]) vertex_arr[i + 1][j] = make_shared<Vertex>(offsettable[i + 1][j]);
						edge_arr[i][j][d] = make_shared<WingedEdge>();

						if (!RIGHT_LIMIT)
						{
							if (!vertex_arr[i + 1][j + 1]) vertex_arr[i + 1][j + 1] = make_shared<Vertex>(offsettable[i + 1][j + 1]);
							edge_arr[i][j][rd] = make_shared<WingedEdge>();
							edge_arr[i + 1][j][r] = make_shared<WingedEdge>();
							face_arr[i][j] = make_shared<Face>(LOWER_LEFT_TRIFACE);
						}
					}

				} break;

				case UPPER_RIGHT_TRIFACE:
				{
					if (!vertex_arr[i][j]) vertex_arr[i][j] = make_shared<Vertex>(offsettable[i][j]);

					if (!RIGHT_LIMIT)
					{
						if (!vertex_arr[i][j + 1]) vertex_arr[i][j + 1] = make_shared<Vertex>(offsettable[i][j + 1]);
						edge_arr[i][j][r] = make_shared<WingedEdge>();

						if (!LOWER_LIMIT)
						{
							if (!vertex_arr[i + 1][j + 1]) vertex_arr[i + 1][j + 1] = make_shared<Vertex>(offsettable[i + 1][j + 1]);
							edge_arr[i][j][rd] = make_shared<WingedEdge>();
							edge_arr[i][j + 1][d] = make_shared<WingedEdge>();
							face_arr[i][j] = make_shared<Face>(UPPER_RIGHT_TRIFACE);
						}
					}

				} break;

				case LOWER_RIGHT_TRIFACE:
				{
					if (!LOWER_LIMIT)
						if (!vertex_arr[i + 1][j]) vertex_arr[i + 1][j] = make_shared<Vertex>(offsettable[i + 1][j]);

					if (!RIGHT_LIMIT)
						if (!vertex_arr[i][j + 1]) vertex_arr[i][j + 1] = make_shared<Vertex>(offsettable[i][j + 1]);

					if (!LOWER_LIMIT && !RIGHT_LIMIT)
					{
						if (!vertex_arr[i + 1][j + 1]) vertex_arr[i + 1][j + 1] = make_shared<Vertex>(offsettable[i + 1][j + 1]);
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

	void BuildConnection(const vector<vector<FaceType>>& facetype,
		vector<vector<shared_ptr<Vertex>>>& vertex_arr,
		vector<vector<array<shared_ptr<WingedEdge>, 4>>>& edge_arr,
		vector<vector<shared_ptr<Face>>>& face_arr)
	{
		int n = vertex_arr.size();
		int m = vertex_arr[0].size();
		enum { d = 0, r, ru, rd };
		enum { START = 0, END = 1 };
		enum { LEFTPREV = 0, LEFTNEXT = 1, RIGHTPREV = 2, RIGHTNEXT = 3 };
		enum { LEFT = 0, RIGHT = 1};

		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < m; j++)
			{
				bool UPPER_LIMIT = i == 0;
				bool LOWER_LIMIT = i == n - 1;
				bool LEFT_LIMIT = j == 0;
				bool RIGHT_LIMIT = j == m - 1;

				// evaluate downward edge
				auto& edge_d = edge_arr[i][j][d];

				if (!edge_d)
				{
					// set start and end vertex
					if (!LOWER_LIMIT)
					{
						edge_d->vertex[START] = vertex_arr[i][j];
						edge_d->vertex[END] = vertex_arr[i + 1][j];
					}
					// set left face
					if (!RIGHT_LIMIT)
						if (facetype[i][j] != UPPER_RIGHT_TRIFACE && facetype[i][j] != LOWER_RIGHT_TRIFACE)
							edge_d->face[LEFT] = face_arr[i][j];

					// set right face
					if (!LEFT_LIMIT)
						if (facetype[i][j - 1] != UPPER_LEFT_TRIFACE && facetype[i][j - 1] != LOWER_LEFT_TRIFACE)
							edge_d->face[RIGHT] = face_arr[i][j - 1];

					// set left prev, next edge
					if (!LOWER_LIMIT && !RIGHT_LIMIT)
					{
						switch (edge_d->face[LEFT]->facetype)
						{
						case QUADFACE:
						{
							edge_d->edge[LEFTPREV] = edge_arr[i + 1][j][r];
							edge_d->edge[LEFTNEXT] = edge_arr[i][j][r];
						} break;

						case UPPER_LEFT_TRIFACE:
						{
							edge_d->edge[LEFTPREV] = edge_arr[i][j][ru];
							edge_d->edge[LEFTNEXT] = edge_arr[i][j][r];
						} break;

						case LOWER_LEFT_TRIFACE:
						{
							edge_d->edge[LEFTPREV] = edge_arr[i + 1][j][r];
							edge_d->edge[LEFTNEXT] = edge_arr[i][j][rd];
						} break;
						}
					}

					// set right prev, next edge
					if (!LOWER_LIMIT && !LEFT_LIMIT)
					{
						switch (edge_d->face[RIGHT]->facetype)
						{
						case QUADFACE:
						{
							edge_d->edge[RIGHTPREV] = edge_arr[i][j - 1][r];
							edge_d->edge[RIGHTNEXT] = edge_arr[i + 1][j - 1][r];
						} break;

						case UPPER_RIGHT_TRIFACE:
						{
							edge_d->edge[RIGHTPREV] = edge_arr[i][j - 1][r];
							edge_d->edge[RIGHTNEXT] = edge_arr[i][j - 1][rd];
						} break;

						case LOWER_RIGHT_TRIFACE:
						{
							edge_d->edge[RIGHTPREV] = edge_arr[i][j - 1][ru];
							edge_d->edge[RIGHTNEXT] = edge_arr[i + 1][j - 1][r];
						} break;
						}
					}
				}

				// evaluate right edge
				auto& edge_r = edge_arr[i][j][r];

				if (!edge_r)
				{
					// set start and end vertex
					if (!RIGHT_LIMIT)
					{
						edge_r->vertex[START] = vertex_arr[i][j];
						edge_r->vertex[END] = vertex_arr[i][j + 1];
					}
					// set left face
					if (!UPPER_LIMIT)
						if (facetype[i - 1][j] != UPPER_RIGHT_TRIFACE && facetype[i - 1][j] != UPPER_LEFT_TRIFACE)
							edge_r->face[LEFT] = face_arr[i - 1][j];

					// set right face
					if (!LOWER_LIMIT)
						if (facetype[i][j] != LOWER_RIGHT_TRIFACE && facetype[i][j] != LOWER_LEFT_TRIFACE)
							edge_r->face[RIGHT] = face_arr[i][j];

					// set left prev, next edge
					if (!UPPER_LIMIT && !LOWER_LIMIT)
					{
						switch (edge_r->face[LEFT]->facetype)
						{
						case QUADFACE:
						{
							edge_r->edge[LEFTPREV] = edge_arr[i - 1][j + 1][d];
							edge_r->edge[LEFTNEXT] = edge_arr[i - 1][j][d];
						} break;

						case LOWER_RIGHT_TRIFACE:
						{
							edge_r->edge[LEFTPREV] = edge_arr[i - 1][j + 1][d];
							edge_r->edge[LEFTNEXT] = edge_arr[i - 1][j][ru];
						} break;

						case LOWER_LEFT_TRIFACE:
						{
							edge_r->edge[LEFTPREV] = edge_arr[i - 1][j][rd];
							edge_r->edge[LEFTNEXT] = edge_arr[i - 1][j][d];
						} break;
						}
					}

					// set right prev, next edge
					if (!LOWER_LIMIT && !LEFT_LIMIT)
					{
						switch (edge_r->face[RIGHT]->facetype)
						{
						case QUADFACE:
						{
							edge_r->edge[RIGHTPREV] = edge_arr[i][j][d];
							edge_r->edge[RIGHTNEXT] = edge_arr[i][j + 1][d];
						} break;

						case UPPER_RIGHT_TRIFACE:
						{
							edge_r->edge[RIGHTPREV] = edge_arr[i][j][rd];
							edge_r->edge[RIGHTNEXT] = edge_arr[i][j + 1][d];
						} break;

						case UPPER_LEFT_TRIFACE:
						{
							edge_r->edge[RIGHTPREV] = edge_arr[i][j][d];
							edge_r->edge[RIGHTNEXT] = edge_arr[i][j][ru];
						} break;
						}
					}
				}
				

				// evaluate right upper edge
				auto& edge_ru = edge_arr[i][j][ru];

				if (!edge_ru)
				{
					// set start and end vertex
					if (!LOWER_LIMIT && !RIGHT_LIMIT)
					{
						edge_ru->vertex[START] = vertex_arr[i + 1][j];
						edge_ru->vertex[END] = vertex_arr[i][j + 1];
					}
					// set left face
					if (facetype[i][j] == UPPER_LEFT_TRIFACE)
						edge_ru->face[LEFT] = face_arr[i][j];

					// set right face
					if (facetype[i][j] == LOWER_RIGHT_TRIFACE)
						edge_ru->face[RIGHT] = face_arr[i][j];

					// set left/right prev, next edge
					if (!UPPER_LIMIT && !LOWER_LIMIT)
					{
						edge_ru->edge[LEFTPREV] = edge_arr[i][j][r];
						edge_ru->edge[LEFTNEXT] = edge_arr[i][j][d];
						edge_ru->edge[RIGHTPREV] = edge_arr[i + 1][j][r];
						edge_ru->edge[RIGHTNEXT] = edge_arr[i][j + 1][d];
					}
				}
				
				// evaluate right down edge
				auto& edge_rd = edge_arr[i][j][rd];

				if (!edge_rd)
				{
					// set start and end vertex
					if (!LOWER_LIMIT && !RIGHT_LIMIT)
					{
						edge_ru->vertex[START] = vertex_arr[i][j];
						edge_ru->vertex[END] = vertex_arr[i + 1][j + 1];
					}
					// set left face
					if (facetype[i][j] == UPPER_RIGHT_TRIFACE)
						edge_ru->face[LEFT] = face_arr[i][j];

					// set right face
					if (facetype[i][j] == LOWER_LEFT_TRIFACE)
						edge_ru->face[RIGHT] = face_arr[i][j];

					// set left/right prev, next edge
					if (!UPPER_LIMIT && !LOWER_LIMIT)
					{
						edge_ru->edge[LEFTPREV] = edge_arr[i][j + 1][d];
						edge_ru->edge[LEFTNEXT] = edge_arr[i][j][r];
						edge_ru->edge[RIGHTPREV] = edge_arr[i][j][d];
						edge_ru->edge[RIGHTNEXT] = edge_arr[i + 1][j][r];
					}
				}
			}
		}
	}

	void SetFaceEdges(TopologyModel& model)
	{
		enum { d = 0, r, ru, rd };
		enum { START = 0, END = 1 };
		enum { LEFTPREV = 0, LEFTNEXT = 1, RIGHTPREV = 2, RIGHTNEXT = 3 };
		enum { LEFT = 0, RIGHT = 1 };

		for (auto& edge : model.edges)
		{
			for (auto& face : model.faces)
			{
				int face_dir = 0;
				int edge_dir = 0;
				if (edge->face[LEFT] == face)
				{
					face_dir = LEFT;
					edge_dir = LEFTNEXT;
				}
				else if (edge->face[RIGHT] == face)
				{
					face_dir = RIGHT;
					edge_dir = RIGHTNEXT;
				}
				else
					continue;

				shared_ptr<WingedEdge> current = edge;

				int n_edge = 0;
				if (edge->face[face_dir]->facetype == QUADFACE)
					n_edge = 4;
				else if (edge->face[face_dir]->facetype == NULLFACE) {}
				else
					n_edge = 3;

				for (int k = 0; k < n_edge; k++)
				{
					face->edges[k] = current;
					current = current->edge[edge_dir];
				}
			}
		}
	}

}

namespace Geometry
{
	void ComputeTriSurfaceCP()
	{
		
	}

	void ComputeQuadSurfaceCP()
	{

	}

	void ComputeCurveCP(Topology::WingedEdge& edge)
	{

	}

	void ComputeAllFaceSurface(Topology::TopologyModel& model)
	{
		for (auto& face : model.faces)
		{
			if (face->surface) continue;

			for (auto& edge : face->edges)
			{

			}
		}
	}
}
