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
			
		vector<vector<shared_ptr<Vertex>>> raw_vertexlist(n,
			vector<shared_ptr<Vertex>>(m, nullptr));
		enum { d = 0, r, ru, rd };
		vector<vector<array<shared_ptr<WingedEdge>, 4>>> raw_edgelist(n,
			vector<array<shared_ptr<WingedEdge>, 4>>(m,
				array<shared_ptr<WingedEdge>, 4>({ nullptr, nullptr, nullptr, nullptr })));

		vector<vector<shared_ptr<Face>>> raw_facelist(n,
			vector<shared_ptr<Face>>(m, nullptr));


	}
}
