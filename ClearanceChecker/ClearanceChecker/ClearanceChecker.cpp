// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include "ClearanceChecker.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <unordered_set>

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
	//// input
	// someText: file name to be opened
	// [0]: dir name
	// [1]: comma-separated input filename(s)
	////

	////
	// [begin] decode parameters
	std::string ZBtext(someText);
	std::string separator(",");
	size_t separator_length = separator.length();
	std::vector<std::string> ZBtextList;

	if (separator_length == 0)
	{
		ZBtextList.push_back(ZBtext);
	}
	else
	{
		size_t offset = std::string::size_type(0);
		while (true)
		{
			size_t pos = ZBtext.find(separator, offset);
			if (pos == std::string::npos)
			{
				std::string filename = ZBtext.substr(offset);
				ZBtextList.push_back(filename);
				break;
			}
			std::string filename = ZBtext.substr(offset, pos - offset);
			ZBtextList.push_back(filename);

			offset = pos + separator_length;
		}
	}

	std::ofstream logFile(ZBtextList.at(0) + "log.txt");

	std::vector<meshWithAABB> meshes(ZBtextList.size() - 1);
	std::string tmp;
	for (int i = 1; i < ZBtextList.size(); ++i)
	{
		tmp = ZBtextList.at(i);
		ZBtextList.at(i) = ZBtextList.at(0) + tmp;

		std::cout << i << "-th mesh: " << ZBtextList.at(i) << std::endl;
		logFile << i << "-th mesh: " << ZBtextList.at(i) << std::endl;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V = meshes.at(i - 1).V;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F = meshes.at(i - 1).F;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &VC = meshes.at(i - 1).VC;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FG = meshes.at(i - 1).FG;
		igl::AABB<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, 3> &AABB = meshes.at(i - 1).AABB;
		read_OBJ(ZBtextList.at(i), V, F, VC, FG);
	}
	// [end] parameter decoding end.
	////

	std::unordered_map<int, std::unordered_map<int, float>> clearance;
	computeClearance(meshes, clearance);

	logFile.close();
	sprintf(outputBuffer, "AAA");

	return 1.0f;
}

void computeClearance(
	const std::vector<meshWithAABB> &meshes,
	std::unordered_map<int, std::unordered_map<int, float>> &clearance)
{
}
