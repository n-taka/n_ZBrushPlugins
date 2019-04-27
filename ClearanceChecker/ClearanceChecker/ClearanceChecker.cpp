// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include "ClearanceChecker.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>

#include "igl/facet_components.h"
#include "igl/remove_unreferenced.h"
#include "igl/parallel_for.h"
#include "igl/signed_distance.h"
#include "igl/writeOBJ.h"

////
// implementation
////
#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

extern "C" DLLEXPORT float version(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData)
{
	return 1.0f;
}

extern "C" DLLEXPORT float checkClearance(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData)
{
	std::ofstream logFile;
	std::vector<Mesh> meshes;
	floatParams params;
	parseParams(someText, optValue, outputBuffer, logFile, meshes, params);
	splitIslands(meshes);
	std::unordered_map<int, std::unordered_map<int, float>> clearance;
	computeClearance(meshes, logFile, clearance);

	logFile.close();
	sprintf(outputBuffer, "AAA");

	return 1.0f;
}

void splitIslands(
	std::vector<Mesh> &meshes)
{
	for (int m = 0; m < meshes.size(); ++m)
	{
		Mesh &mesh = meshes.at(m);
		std::vector<Mesh> &islands = mesh.islands;
		Eigen::Matrix<int, Eigen::Dynamic, 1> C;
		igl::facet_components(mesh.F, C);
		islands.resize(C.maxCoeff() + 1);
		for (int c = 0; c < C.maxCoeff() + 1; ++c)
		{
			std::vector<int> fc_vec;
			for (int f = 0; f < mesh.F.rows(); ++f)
			{
				if (C(f, 0) == c)
				{
					fc_vec.push_back(mesh.F(f, 0));
					fc_vec.push_back(mesh.F(f, 1));
					fc_vec.push_back(mesh.F(f, 2));
				}
			}
			Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> islandF_before = Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(&fc_vec[0], fc_vec.size() / 3, 3);
			Mesh &island = islands.at(c);
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &islandV = island.V;
			Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &islandF = island.F;
			Eigen::Matrix<int, Eigen::Dynamic, 1> I, J;
			igl::remove_unreferenced(mesh.V, islandF_before, islandV, islandF, I, J);
		}
	}
}

void computeClearance(
	const std::vector<Mesh> &meshes,
	std::ofstream &logFile,
	std::unordered_map<int, std::unordered_map<int, float>> &clearance)
{
	auto computeSignedDistPair = [&](const int &meshIdx0, const int &meshIdx1) {
		// compute distance from meshIdx0 --> meshIdx1
		// If meshIdx0 == meshIdx1, we use islands
		float minDist = std::numeric_limits<float>::max();
		const Mesh &mesh0 = meshes.at(meshIdx0);
		const Mesh &mesh1 = meshes.at(meshIdx1);
		for (int i0 = 0; i0 < mesh0.islands.size(); ++i0)
		{
			for (int i1 = 0; i1 < mesh1.islands.size(); ++i1)
			{
				std::cout << mesh0.fileName << " (" << i0 << ") -- " << mesh1.fileName << " (" << i1 << ")" << std::endl;
				logFile << mesh0.fileName << " (" << i0 << ") -- " << mesh1.fileName << " (" << i1 << ")" << std::endl;
				if (meshIdx0 == meshIdx1 && i0 == i1)
				{
					std::cout << "skipped." << std::endl;
					logFile << "skipped." << std::endl;
					continue;
				}
				const Mesh &island0 = mesh0.islands.at(i0);
				const Mesh &island1 = mesh1.islands.at(i1);
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &P = island0.V;
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V = island1.V;
				const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F = island1.F;
				Eigen::Matrix<float, Eigen::Dynamic, 1> S;
				Eigen::Matrix<int, Eigen::Dynamic, 1> I;
				Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C, N;

				igl::signed_distance(P, V, F, igl::SIGNED_DISTANCE_TYPE_DEFAULT, S, I, C, N);
				std::cout << S.minCoeff() << std::endl;
				logFile << S.minCoeff() << std::endl;
				minDist = std::min(minDist, S.minCoeff());
			}
		}
		return minDist;
	};

	for (int m0 = 0; m0 < meshes.size(); ++m0)
	{
		for (int m1 = 0; m1 < meshes.size(); ++m1)
		{
			clearance[m0][m1] = computeSignedDistPair(m0, m1);
		}
	}
	for (int m0 = 0; m0 < meshes.size(); ++m0)
	{
		for (int m1 = 0; m1 < meshes.size(); ++m1)
		{
			std::cout << clearance[m0][m1] << "\t";
			logFile << clearance[m0][m1] << "\t";
		}
		std::cout << std::endl;
		logFile << std::endl;
	}
}
