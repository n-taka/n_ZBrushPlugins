#include "MagicaVoxelizer.h"

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
	const std::vector< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >& VV,
	const std::vector< Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> >& FF,
	const std::vector< Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> >& VCV
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
	for (int model = 0; model < VV.size(); ++model)
	{
		const auto& V = VV.at(model);
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
	}

	//bool write_N = CN.rows() > 0;

	//if (write_N)
	//{
	//	for (int i = 0; i < (int)CN.rows(); i++)
	//	{
	//		fprintf(obj_file, "vn %0.17g %0.17g %0.17g\n",
	//			CN(i, 0),
	//			CN(i, 1),
	//			CN(i, 2)
	//		);
	//	}
	//	fprintf(obj_file, "\n");
	//}

	//bool write_texture_coords = TC.rows() > 0;

	//if (write_texture_coords)
	//{
	//	for (int i = 0; i < (int)TC.rows(); i++)
	//	{
	//		fprintf(obj_file, "vt %0.17g %0.17g\n", TC(i, 0), TC(i, 1));
	//	}
	//	fprintf(obj_file, "\n");
	//}

	// export MRGB info
	// TODO
	for (int model = 0; model < FF.size(); ++model)
	{
		const auto& VC = VCV.at(model);
		for (int v = 0; v < vertCount; v++)
		{
			if (v % 64 == 0)
			{
				fprintf(obj_file, "#MRGB ");
			}

			fprintf(obj_file, "%02x%02x%02x%02x", VC(v, 3), VC(v, 0), VC(v, 1), VC(v, 2));

			if (v % 64 == 63 || (v == (vertCount - 1)))
			{
				fprintf(obj_file, "\n");
			}
		}
	}

	// loop over F
	for (int model = 0; model < FF.size(); ++model)
	{
		const auto& F = FF.at(model);
		fprintf(obj_file, "g Group%d\n", model);
		for (int i = 0; i < (int)F.rows(); ++i)
		{
			fprintf(obj_file, "f");
			for (int j = 0; j < (int)F.cols(); ++j)
			{
				// OBJ is 1-indexed
				fprintf(obj_file, " %u", F(i, j) + 1);

				//if (write_texture_coords)
				//	fprintf(obj_file, "/%u", FTC(i, j) + 1);
				//if (write_N)
				//{
				//	if (write_texture_coords)
				//		fprintf(obj_file, "/%u", FN(i, j) + 1);
				//	else
				//		fprintf(obj_file, "//%u", FN(i, j) + 1);
				//}
			}
			fprintf(obj_file, "\n");
		}
	}
	fclose(obj_file);
	return true;
}