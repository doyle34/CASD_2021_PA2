#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include "CASD_SurfaceReconstruction.hpp"


using namespace Eigen;
using namespace std;

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