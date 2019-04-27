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

	meshes.resize(ZBtextList.size() - 1);
	logFile.open(ZBtextList.at(0) + "log.txt");
	std::string tmp;
	for (int i = 1; i < ZBtextList.size(); ++i)
	{
		Mesh &mesh = meshes.at(i - 1);
		mesh.fileName = ZBtextList.at(i);
		ZBtextList.at(i) = ZBtextList.at(0) + mesh.fileName;

		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V = mesh.V;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F = mesh.F;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &VC = mesh.VC;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FG = mesh.FG;
		read_OBJ(ZBtextList.at(i), V, F, VC, FG);
	}

	union {
		char c[sizeof(float)];
		float f;
	} loader;
	memcpy(loader.c, outputBuffer, sizeof(float));
	params.height = loader.f;
}
