// Sample.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "Eigen/Core"
#include "igl/read_triangle_mesh.h"
#include "igl/signed_distance.h"
#include <iostream>


extern "C" __declspec(dllexport) float magicaVoxelize(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

extern "C" __declspec(dllexport) float magicaVoxelize(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	//// input
	// someText: file name to be opened
	// optValue: voxel resolution in Y coordinate
	////
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V;
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> F;
	Eigen::Matrix<double, Eigen::Dynamic, 1> S;
	Eigen::Matrix<double, Eigen::Dynamic, 1> I;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> N;

	// read triangle from file
	igl::read_triangle_mesh(someText, V, F);

	// generate voxel for signed distance computation (query point is center of each voxel)
	Eigen::Matrix<double, 1, Eigen::Dynamic> BBSize = V.colwise().maxCoeff() - V.colwise().minCoeff();
	Eigen::Matrix<double, 1, Eigen::Dynamic> BBCenter = (V.colwise().maxCoeff() + V.colwise().minCoeff())/2;
	const double oneVoxelSize = BBSize(0, 1) / optValue;
	Eigen::Matrix<int, 1, Eigen::Dynamic> voxelCount;
	voxelCount.resize(1, 3);
	voxelCount << int(ceil(BBSize(0, 0) / oneVoxelSize)), int(ceil(BBSize(0, 1) / oneVoxelSize)), int(ceil(BBSize(0, 2) / oneVoxelSize));
	P.resize(voxelCount(0, 0)*voxelCount(0, 1)*voxelCount(0, 2), 3);

	// build queries
	const Eigen::Matrix<double, 1, Eigen::Dynamic> bottomCorner = (V.colwise().minCoeff().array() + oneVoxelSize);
#define VOX(x, y, z) ((z)*(voxelCount(0,1))*(voxelCount(0,0)) + (y)*(voxelCount(0,0)) + (x))
	for (int z = 0; z < voxelCount(0, 2); ++z)
	{
		for (int y = 0; y < voxelCount(0, 1); ++y)
		{
			for (int x = 0; x < voxelCount(0, 0); ++x)
			{
				const int voxelIndex = VOX(x, y, z);
				P.row(voxelIndex) = bottomCorner;
				P(voxelIndex, 0) += oneVoxelSize * x;
				P(voxelIndex, 1) += oneVoxelSize * y;
				P(voxelIndex, 2) += oneVoxelSize * z;
			}
		}
	}

	// signed distance computation
	const igl::SignedDistanceType signedDistanceType = igl::SIGNED_DISTANCE_TYPE_DEFAULT;
	igl::signed_distance(P, V, F, signedDistanceType, S, I, C, N);

	// voxelize
	// TODO

	//std::cout << V << std::endl << std::endl;
	//std::cout << F << std::endl << std::endl;
	std::cout << "test function!" << std::endl;
	return 0.0f;
}