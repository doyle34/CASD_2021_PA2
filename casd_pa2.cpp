#include "casd_pa2.h"

// void DrawShip();

namespace CASD_PA2
{
	class CubicCurve
	{
	public:
		vector<Vector3f> control_points;
		CubicCurve() {};
		~CubicCurve() {};
	};

	class CubicSurface
	{
	public:
		vector<vector<Vector3f>> control_points;
		CubicSurface() {};
		~CubicSurface() {};
	};

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

	enum FaceType
	{
		QUADFACE,
		UPPER_LEFT_TRIFACE,
		LOWER_LEFT_TRIFACE,
		UPPER_RIGHT_TRIFACE,
		LOWER_RIGHT_TRIFACE,
		NULLFACE
	};

	class Face
	{
	public:
		FaceType facetype;
		vector<shared_ptr<WingedEdge>> edges;
		shared_ptr<CubicSurface> surface = nullptr;
		Face() {};
		Face(FaceType ty)
		{
			if (ty == QUADFACE)
			{
				this->edges.resize(4);
				this->facetype = ty;
			}
			else if (ty == NULLFACE) this->facetype = ty;
			else
			{
				this->edges.resize(3);
				this->facetype = ty;
			}
		}
		~Face() {};
	};

	class WingedEdge
	{
	public:
		shared_ptr<CubicCurve> curve = nullptr;
		array<shared_ptr<Vertex>, 2> vertex = { { nullptr, nullptr } };
		array<shared_ptr<WingedEdge>, 4> edge = { { nullptr, nullptr, nullptr, nullptr } };
		array<shared_ptr<Face>, 2> face = { { nullptr, nullptr } };
		WingedEdge() {};
		~WingedEdge() {};
	};

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

	void InitializeTopology(const vector<vector<Vector3f>>& offsettable,
		const vector<vector<FaceType>>& facetype,
		vector<vector<shared_ptr<Vertex>>>& vertex_arr,
		vector<vector<array<shared_ptr<WingedEdge>, 4>>>& edge_arr,
		vector<vector<shared_ptr<Face>>>& face_arr)
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
		enum { LEFT = 0, RIGHT = 1 };

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

				if (edge_d)
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
						if (edge_d->face[LEFT])
						{
							switch (edge_d->face[LEFT]->facetype)
							{
							case QUADFACE:
							{
								edge_d->edge[LEFTNEXT] = edge_arr[i + 1][j][r];
								edge_d->edge[LEFTPREV] = edge_arr[i][j][r];
							} break;

							case UPPER_LEFT_TRIFACE:
							{
								edge_d->edge[LEFTNEXT] = edge_arr[i][j][ru];
								edge_d->edge[LEFTPREV] = edge_arr[i][j][r];
							} break;

							case LOWER_LEFT_TRIFACE:
							{
								edge_d->edge[LEFTNEXT] = edge_arr[i + 1][j][r];
								edge_d->edge[LEFTPREV] = edge_arr[i][j][rd];
							} break;
							}
						}

					}

					// set right prev, next edge
					if (!LOWER_LIMIT && !LEFT_LIMIT)
					{
						if (edge_d->face[RIGHT])
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
				}

				// evaluate right edge
				auto& edge_r = edge_arr[i][j][r];

				if (edge_r)
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
						if (edge_r->face[LEFT])
						{
							switch (edge_r->face[LEFT]->facetype)
							{
							case QUADFACE:
							{
								edge_r->edge[LEFTNEXT] = edge_arr[i - 1][j + 1][d];
								edge_r->edge[LEFTPREV] = edge_arr[i - 1][j][d];
							} break;

							case LOWER_RIGHT_TRIFACE:
							{
								edge_r->edge[LEFTNEXT] = edge_arr[i - 1][j + 1][d];
								edge_r->edge[LEFTPREV] = edge_arr[i - 1][j][ru];
							} break;

							case LOWER_LEFT_TRIFACE:
							{
								edge_r->edge[LEFTNEXT] = edge_arr[i - 1][j][rd];
								edge_r->edge[LEFTPREV] = edge_arr[i - 1][j][d];
							} break;
							}
						}
						
					}

					// set right prev, next edge
					if (!LOWER_LIMIT && !LEFT_LIMIT)
					{
						if (edge_r->face[RIGHT])
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
				}


				// evaluate right upper edge
				auto& edge_ru = edge_arr[i][j][ru];

				if (edge_ru)
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
					edge_ru->edge[LEFTNEXT] = edge_arr[i][j][r];
					edge_ru->edge[LEFTPREV] = edge_arr[i][j][d];
					if (!LOWER_LIMIT)
						edge_ru->edge[RIGHTPREV] = edge_arr[i + 1][j][r];
					if (!RIGHT_LIMIT)
						edge_ru->edge[RIGHTNEXT] = edge_arr[i][j + 1][d];
				}

				// evaluate right down edge
				auto& edge_rd = edge_arr[i][j][rd];

				if (edge_rd)
				{
					// set start and end vertex
					if (!LOWER_LIMIT && !RIGHT_LIMIT)
					{
						edge_rd->vertex[START] = vertex_arr[i][j];
						edge_rd->vertex[END] = vertex_arr[i + 1][j + 1];
					}
					// set left face
					if (facetype[i][j] == UPPER_RIGHT_TRIFACE)
						edge_rd->face[LEFT] = face_arr[i][j];

					// set right face
					if (facetype[i][j] == LOWER_LEFT_TRIFACE)
						edge_rd->face[RIGHT] = face_arr[i][j];

					// set left/right prev, next edge
					if (!RIGHT_LIMIT)
						edge_rd->edge[LEFTNEXT] = edge_arr[i][j + 1][d];
					edge_rd->edge[LEFTPREV] = edge_arr[i][j][r];
					edge_rd->edge[RIGHTPREV] = edge_arr[i][j][d];
					if (!LOWER_LIMIT)
						edge_rd->edge[RIGHTNEXT] = edge_arr[i + 1][j][r];
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
				// Try CW first and CCW later
				bool isCW = true;
				int face_dir = 0;
				int edge_dir = 0;
				if (isCW)
				{
					if (edge->face[LEFT] == face)
					{
						face_dir = LEFT;
						edge_dir = LEFTPREV;
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
						if (!current->edge[edge_dir])
						{
							isCW = false;
							break;
						}
						current = current->edge[edge_dir];
					}
				}

				if (!isCW)
				{
					if (edge->face[LEFT] == face)
					{
						face_dir = LEFT;
						edge_dir = LEFTNEXT;
					}
					else if (edge->face[RIGHT] == face)
					{
						face_dir = RIGHT;
						edge_dir = RIGHTPREV;
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
						if (!current->edge[edge_dir])
							break;
						current = current->edge[edge_dir];
					}
				}
			}
		}
	}

	Vector3f DeCasteljau(CubicCurve c, float u)
	{
		const int n_order = 4; // assume only cubic curve
		Array<Vector3f, n_order, n_order> point;
		for (int i = 0; i < n_order; i++)
			point(0, i) = c.control_points[i];

		for (int step = 1; step < n_order; step++)
			for (int i = 0; i < n_order - step; i++)
				point(step, i) = (1 - u) * point(step - 1, i) + u * point(step - 1, i + 1);

		return point(n_order - 1, 0);
	}

	Vector3f DeCasteljau(CubicSurface s, float u, float v)
	{
		if (s.control_points.back().size() < 4) // check if triangle face
			return Vector3f(0, 0, 0);

		const int u_order = 4;
		const int v_order = 4;
		CubicCurve v_curve;
		CubicCurve u_curve;

		for (int i = 0; i < v_order; i++)
		{
			u_curve.control_points = s.control_points[i];
			v_curve.control_points.push_back(DeCasteljau(u_curve, u));
		}

		Vector3f point = DeCasteljau(v_curve, v);
		return point;
	}

	void ReconstructSurface(TopologyModel& model)
	{
		ComputeAllNormal(model);
		ComputeAllFaceSurface(model);
	}

	shared_ptr<Vector3f> ComputeANormal(
		shared_ptr<WingedEdge>& edge,
		shared_ptr<Vertex>& vertex)
	{
		enum { d = 0, r, ru, rd };
		enum { START = 0, END = 1 };
		enum { LEFTPREV = 0, LEFTNEXT = 1, RIGHTPREV = 2, RIGHTNEXT = 3 };
		enum { LEFT = 0, RIGHT = 1 };
		enum { CCW = 0, CW = 1 };
		int count = 0;
		bool isCW = true;

		shared_ptr<WingedEdge> current = edge;
		shared_ptr<WingedEdge> next;
		Vector3f sum(0, 0, 0);

		// try cw first and try ccw later
		do
		{
			if (current->vertex[START] == vertex)
				next = current->edge[LEFTPREV];
			else
				next = current->edge[RIGHTNEXT];
			
			if (!next)
			{
				isCW = false;
				break;
			}

			Vector3f current_end;
			if (current->vertex[START] == vertex)
				current_end = *current->vertex[END]->point;
			else
				current_end = *current->vertex[START]->point;

			Vector3f next_end;
			if (next->vertex[START] == vertex)
				next_end = *next->vertex[END]->point;
			else
				next_end = *next->vertex[START]->point;



			Vector3f v1 = current_end - (*vertex->point);
			Vector3f v2 = next_end - (*vertex->point);
			Vector3f norm = v1.cross(v2);

			sum += norm;
			count++;

			current = next;

		} while (current != edge);

		if (!isCW)
		{
			current = edge;
			do
			{
				if (current->vertex[START] == vertex)
					next = current->edge[RIGHTPREV];
				else
					next = current->edge[LEFTNEXT];

				if (!next) break;

				Vector3f current_end;
				if (current->vertex[START] == vertex)
					current_end = *current->vertex[END]->point;
				else
					current_end = *current->vertex[START]->point;

				Vector3f next_end;
				if (next->vertex[START] == vertex)
					next_end = *next->vertex[END]->point;
				else
					next_end = *next->vertex[START]->point;

				Vector3f v1 = current_end - (*vertex->point);
				Vector3f v2 = next_end - (*vertex->point);
				Vector3f norm = v2.cross(v1);

				sum += norm;
				count++;

				current = next;

			} while (current != edge);
		}

		sum /= (float)count;

		shared_ptr<Vector3f> pointnormal = make_shared<Vector3f>(sum);

		return pointnormal;
	}

	void ComputeAllNormal(TopologyModel& model)
	{
		for (auto& edge : model.edges)
		{
			for (auto& vertex : edge->vertex)
			{
				if (vertex->normal) continue;
				vertex->normal = ComputeANormal(edge, vertex);
			}
		}
	}

	void ComputeTriSurfaceCP(shared_ptr<Face>& face)
	{
		face->surface = make_shared<CubicSurface>();
		Vector3f b[4][4][4];
		b[0][3][0] = face->edges[0]->curve->control_points[0];
		b[0][2][1] = face->edges[0]->curve->control_points[1];
		b[0][1][2] = face->edges[0]->curve->control_points[2];
		b[0][0][3] = face->edges[1]->curve->control_points[0];
		b[1][0][2] = face->edges[1]->curve->control_points[1];
		b[2][0][1] = face->edges[1]->curve->control_points[2];
		b[3][0][0] = face->edges[2]->curve->control_points[0];
		b[2][1][0] = face->edges[2]->curve->control_points[1];
		b[1][2][0] = face->edges[2]->curve->control_points[2];

		// u = 0 ruled surface center point
		Vector3f b_u_111 = (b[1][2][0] + b[1][0][2]) / 2;

		// v = 0 ruled surface center point
		Vector3f b_v_111 = (b[0][1][2] + b[2][1][0]) / 2;

		// w = 0 ruled surface center point
		Vector3f b_w_111 = (b[0][2][1] + b[2][0][1]) / 2;

		// Trilinear surface center point
		Vector3f b_l_120 = (2 * b[0][3][0] + b[3][0][0]) / 3;
		Vector3f b_l_102 = (2 * b[0][0][3] + b[3][0][0]) / 3;
		Vector3f b_l_111 = (b_l_120 + b_l_102) / 2;

		b[1][1][1] = (b_u_111 + b_v_111 + b_w_111 - b_l_111) / 2;

		for (int i = 0; i < 4; i++)
		{
			vector<Vector3f> row;
			for (int j = 3 - i; j >= 0; j--)
			{
				int k = 3 - i - j;
				row.push_back(b[i][j][k]);
			}
			face->surface->control_points.push_back(row);
		}
	}

	void ComputeQuadSurfaceCP(shared_ptr<Face>& face)
	{
		face->surface = make_shared<CubicSurface>();

		Array<Vector3f, 4, 4> b;
		// v-ruled surface center 4 control points
		Array<Vector3f, 4, 4> b_v;
		b_v(1, 0) = face->edges[0]->curve->control_points[1];
		b(1, 0) = b_v(1, 0);
		b_v(2, 0) = face->edges[0]->curve->control_points[2];
		b(2, 0) = b_v(2, 0);
		b_v(1, 3) = face->edges[2]->curve->control_points[2];
		b(1, 3) = b_v(1, 3);
		b_v(2, 3) = face->edges[2]->curve->control_points[1];
		b(2, 3) = b_v(2, 3);
		b_v(1, 1) = (2 * b_v(1, 0) + b_v(1, 3)) / 3;
		b_v(1, 2) = (b_v(1, 0) + 2 * b_v(1, 3)) / 3;
		b_v(2, 1) = (2 * b_v(2, 0) + b_v(2, 3)) / 3;
		b_v(2, 2) = (b_v(2, 0) + 2 * b_v(2, 3)) / 3;

		// u-ruled surface center 4 control points
		Array<Vector3f, 4, 4> b_u;
		b_u(3, 1) = face->edges[1]->curve->control_points[1];
		b(3, 1) = b_u(3, 1);
		b_u(3, 2) = face->edges[1]->curve->control_points[2];
		b(3, 2) = b_u(3, 2);
		b_u(0, 1) = face->edges[3]->curve->control_points[2];
		b(0, 1) = b_u(0, 1);
		b_u(0, 2) = face->edges[3]->curve->control_points[1];
		b(0, 2) = b_u(0, 2);
		b_u(1, 1) = (2 * b_u(0, 1) + b_u(3, 1)) / 3;
		b_u(2, 1) = (b_u(0, 1) + 2 * b_u(3, 1)) / 3;
		b_u(1, 2) = (2 * b_u(0, 2) + b_u(3, 2)) / 3;
		b_u(2, 2) = (b_u(0, 2) + 2 * b_u(3, 2)) / 3;

		// bilinear surface center 4 control points
		Array<Vector3f, 4, 4> b_l;
		b_l(0, 0) = face->edges[0]->curve->control_points[0];
		b(0, 0) = b_l(0, 0);
		b_l(3, 0) = face->edges[1]->curve->control_points[0];
		b(3, 0) = b_l(3, 0);
		b_l(3, 3) = face->edges[2]->curve->control_points[0];
		b(3, 3) = b_l(3, 3);
		b_l(0, 3) = face->edges[3]->curve->control_points[0];
		b(0, 3) = b_l(0, 3);
		b_l(0, 1) = (2 * b_l(0, 0) + b_u(0, 3)) / 3;
		b_l(0, 2) = (b_u(0, 0) + 2 * b_u(0, 3)) / 3;
		b_l(3, 1) = (2 * b_l(3, 0) + b_u(3, 3)) / 3;
		b_l(3, 2) = (b_u(3, 0) + 2 * b_u(3, 3)) / 3;
		b_l(1, 1) = (2 * b_l(0, 1) + b_l(3, 1)) / 3;
		b_l(2, 1) = (b_l(0, 1) + 2 * b_l(3, 1)) / 3;
		b_l(1, 2) = (2 * b_l(0, 2) + b_l(3, 2)) / 3;
		b_l(2, 2) = (b_l(0, 2) + 2 * b_l(3, 2)) / 3;

		for (int i : {1, 2})
		{
			for (int j : {1, 2})
				b(i, j) = b_v(i, j) + b_u(i, j) - b_l(i, j);
		}

		for (int i = 0; i < 4; i++)
		{
			vector<Vector3f> row;
			for (int j = 0; j < 4; j++)
				row.push_back(b(i, j));

			face->surface->control_points.push_back(row);
		}
	}

	void ComputeCurveCP(shared_ptr<WingedEdge>& edge)
	{
		edge->curve = make_shared<CubicCurve>();
		enum { START = 0, END = 1 };
		float tmp_1 = 0.0f;
		float tmp_2 = 0.0f;
		Vector3f start = *edge->vertex[START]->point;
		Vector3f start_n = *edge->vertex[START]->normal;
		Vector3f end = *edge->vertex[END]->point;
		Vector3f end_n = *edge->vertex[END]->point;

		Vector3f a = end - start;
		Vector3f a_3 = a / 3;
		Vector3f start_c1 = a_3 - (a_3.dot(start_n) / start_n.norm() * start_n.normalized());
		Vector3f c2_end = a_3 - (a_3.dot(start_n) / end_n.norm() * end_n.normalized());
		Vector3f c1 = start_c1 + start;
		Vector3f c2 = end - c2_end;

		edge->curve->control_points.push_back(start);
		edge->curve->control_points.push_back(c1);
		edge->curve->control_points.push_back(c2);
		edge->curve->control_points.push_back(end);
	}

	void ComputeAllFaceSurface(TopologyModel& model)
	{
		for (auto& face : model.faces)
		{
			if (face->surface) continue;

			for (auto& edge : face->edges)
				ComputeCurveCP(edge); // face 주변의 edge에 대해 curve 생성 (control point 구하기)

			// face 주변의 edge들의 curve를 참고하여 coons patch control point 구하기
			if (face->facetype == QUADFACE)
				ComputeQuadSurfaceCP(face);
			else if (face->facetype == NULLFACE) {}
			else
				ComputeTriSurfaceCP(face);
		}
	}

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
				offsettabledata[i][j] = Vector3f(xdata[i], ydata[i][j] / 1000, zdata[j]);
			}
		}

		return offsettabledata;

	}


	void DrawSurface(const shared_ptr<Face>& face)
	{
		enum { x = 0, y = 1, z = 2 };

		Array<Vector3f, N_MESH_DIV + 1, N_MESH_DIV + 1> surface_points;
		float u = 0.0f;
		float v = 0.0f;
		for (int i = 0; i < N_MESH_DIV + 1; i++)
		{
			u = 0.0f + 1 / (float)N_MESH_DIV * i;
			for (int j = 0; j < N_MESH_DIV + 1; j++)
			{
				v = 0.0f + 1 / (float)N_MESH_DIV * j;
				surface_points(i, j) = DeCasteljau(*face->surface, u, v);
			}
		}

		glBegin(GL_TRIANGLES);
		glColor3f(1.0, 1.0, 1.0);

		for (int i = 0; i < N_MESH_DIV; i++)
		{
			for (int j = 0; j < N_MESH_DIV; j++)
			{
				// draw upper left triangle
				glVertex3f(surface_points(i, j)(x), surface_points(i, j)(y), surface_points(i, j)(z));
				glVertex3f(surface_points(i + 1, j)(x), surface_points(i + 1, j)(y), surface_points(i + 1, j)(z));
				glVertex3f(surface_points(i, j + 1)(x), surface_points(i, j + 1)(y), surface_points(i, j + 1)(z));

				// draw lower right triangle
				glVertex3f(surface_points(i + 1, j)(x), surface_points(i + 1, j)(y), surface_points(i + 1, j)(z));
				glVertex3f(surface_points(i + 1, j + 1)(x), surface_points(i + 1, j + 1)(y), surface_points(i + 1, j + 1)(z));
				glVertex3f(surface_points(i, j + 1)(x), surface_points(i, j + 1)(y), surface_points(i, j + 1)(z));
			}
		}

		glEnd();

	}

	void DrawAllSurface(const TopologyModel& model)
	{
		for (auto& face : model.faces)
		{
			if (face->facetype == QUADFACE)
				DrawSurface(face);
		}
	}

	void DrawVertexes(const TopologyModel& model)
	{
		glBegin(GL_POINTS);
		glPointSize(10);
		glColor3f(1.0, 0.0, 0.0);

		for (auto& v : model.vertices)
		{
			glVertex3f((*v->point)(0), (*v->point)(1), (*v->point)(2));
		}

		glEnd();
	}

	void DrawCP(const TopologyModel& model)
	{
		glBegin(GL_POINTS);
		glPointSize(100);
		glColor3f(1.0, 0.0, 0.0);

		for (auto& face : model.faces)
		{
			for (auto& cp_row : face->surface->control_points)
			{
				for (auto& cp : cp_row)
				{
					glVertex3f(cp(0), cp(1), cp(2));
				}
			}
		}

		glEnd();
	}
}