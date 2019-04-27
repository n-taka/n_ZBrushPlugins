#include "ClearanceChecker.h"

#include "stdafx.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

void parseParams(
	const char *someText,
	const double optValue,
	const char *outputBuffer,
	std::ofstream &logFile,
	std::vector<Mesh> &meshes,
	floatParams &params)
{
	//// input
	// someText: file name to be opened
	// [0]: dir name
	// [1]: comma-separated input filename(s)
	////

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

	logFile.open(ZBtextList.at(0) + "log.txt");

	{
		union {
			char c[sizeof(float)];
			float f;
		} loader;
		memcpy(loader.c, outputBuffer, sizeof(float));
		params.height = loader.f;
		memcpy(loader.c, outputBuffer + sizeof(float), sizeof(float));
		params.minimumClearance = loader.f;

		std::cout << "Height: " << params.height << std::endl;
		std::cout << "Minimum clearance: " << params.minimumClearance << std::endl;
		logFile << "Height: " << params.height << std::endl;
		logFile << "Minimum clearance: " << params.minimumClearance << std::endl;
	}

	{
		Eigen::Matrix<float, 1, Eigen::Dynamic> largest;
		largest.resize(1, 3);
		largest.setConstant(std::numeric_limits<float>::min());
		Eigen::Matrix<float, 1, Eigen::Dynamic> smallest;
		smallest.resize(1, 3);
		smallest.setConstant(std::numeric_limits<float>::max());
		meshes.resize(ZBtextList.size() - 1);
		for (int i = 1; i < ZBtextList.size(); ++i)
		{
			Mesh &mesh = meshes.at(i - 1);
			mesh.fileName = ZBtextList.at(i);
			std::cout << i << "-th mesh: " << mesh.fileName << std::endl;
			logFile << i << "-th mesh: " << mesh.fileName << std::endl;
			ZBtextList.at(i) = ZBtextList.at(0) + mesh.fileName;

			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V = mesh.V;
			Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F = mesh.F;
			Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &VC = mesh.VC;
			Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FG = mesh.FG;
			read_OBJ(ZBtextList.at(i), V, F, VC, FG);

			largest = largest.cwiseMax(V.colwise().maxCoeff());
			smallest = largest.cwiseMin(V.colwise().minCoeff());
		}
		std::cout << largest << std::endl;
		std::cout << smallest << std::endl;
		// scale the meshes so that the height of the hole model becomes params.height
		const float ratio = params.height / (largest(0, 1) - smallest(0, 1));
		for (auto &mesh : meshes)
		{
			mesh.V *= ratio;
		}
	}
}
