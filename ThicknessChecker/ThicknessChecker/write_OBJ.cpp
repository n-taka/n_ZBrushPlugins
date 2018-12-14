#include "ThicknessChecker.h"

#include "stdafx.h"

#include <iostream>
#include <limits>
#include <iomanip>
#include <fstream>
#include <cstdio>
#include <cassert>

// originally from igl/writeOBJ.
// there is some modificaton for handling MRGB in ZBrush.

bool write_OBJ(
	const std::string& fileName,
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& V,
	const Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& F,
	const Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& VC,
	const Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& FG
)
{
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> TC;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> CN;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> FTC;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> FN;

	FILE * obj_file = fopen(fileName.c_str(), "w");
	if (NULL == obj_file)
	{
		printf("IOError: %s could not be opened for writing...", fileName.c_str());
		return false;
	}
	// Loop over V
	int vertCount = 0;
	vertCount += V.rows();
	for (int i = 0; i < (int)V.rows(); i++)
	{
		fprintf(obj_file, "v");
		for (int j = 0; j < (int)V.cols(); ++j)
		{
			fprintf(obj_file, " %0.17g", V(i, j));
		}
		fprintf(obj_file, "\n");
	}

	// export MRGB info
	if (VC.rows() > 0)
	{
		for (int v = 0; v < vertCount; v++)
		{
			if (v % 64 == 0)
			{
				fprintf(obj_file, "#MRGB ");
			}

			fprintf(obj_file, "%02x%02x%02x%02x", VC(v, 0), VC(v, 1), VC(v, 2), VC(v, 3));

			if (v % 64 == 63 || (v == (vertCount - 1)))
			{
				fprintf(obj_file, "\n");
			}
		}
	}

	// loop over F
	int currentGroup = -1;
	for (int i = 0; i < (int)F.rows(); ++i)
	{
		if (FG(i, 0) != currentGroup)
		{
			currentGroup = FG(i, 0);
			fprintf(obj_file, "g Group%d\n", currentGroup);
		}
		fprintf(obj_file, "f");
		for (int j = 0; j < (int)F.cols(); ++j)
		{
			// OBJ is 1-indexed
			fprintf(obj_file, " %u", F(i, j) + 1);
		}
		fprintf(obj_file, "\n");
	}
	fclose(obj_file);
	return true;
}