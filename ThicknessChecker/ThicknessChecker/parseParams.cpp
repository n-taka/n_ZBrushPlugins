#include "ThicknessChecker.h"

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

// "../../testModels/,dog_high.obj,Radeon RX Vega M GH Graphics\0"

void parseParams(
	const char *someText,
	const double optValue,
	const char *outputBuffer,
	std::ofstream &logFile,
	std::vector<Mesh> &meshes,
	Params &params)
{
	//// input
	// someText: file name to be opened
	// [0]: dir name
	// [1]: input filename
	// [2]: accelerator name
	////

	std::string ZBtext(someText);
#if defined(__APPLE__)
	// for Mac, ZBrush somehow add prefix "!:"...?
	ZBtext = ZBtext.substr(2);
#endif
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
			int i;
		} loader;
		memcpy(loader.c, outputBuffer, sizeof(float));
		params.height = loader.f;
		memcpy(loader.c, outputBuffer + sizeof(float), sizeof(float));
		params.preferredThickness = loader.f;
		memcpy(loader.c, outputBuffer + sizeof(float) * 2, sizeof(float));
		params.minimumThickness = loader.f;
		memcpy(loader.c, outputBuffer + sizeof(float) * 3, sizeof(int));
		params.chunkSize = loader.i;

		std::cout << "Height               : " << params.height << std::endl;
		std::cout << "Preferred thickness  : " << params.preferredThickness << std::endl;
		std::cout << "Minimum thickness    : " << params.minimumThickness << std::endl;
		std::cout << "chunkSize            : " << params.chunkSize << std::endl;
		logFile << "Height               : " << params.height << std::endl;
		logFile << "Preferred thickness  : " << params.preferredThickness << std::endl;
		logFile << "Minimum thickness    : " << params.minimumThickness << std::endl;
		logFile << "chunkSize            : " << params.chunkSize << std::endl;
	}

	{
		meshes.resize(1);

		Mesh &mesh = meshes.at(0);
		mesh.fileName = ZBtextList.at(1);
		std::cout << 1 << "-th mesh: " << mesh.fileName << std::endl;
		logFile << 1 << "-th mesh: " << mesh.fileName << std::endl;
		ZBtextList.at(1) = ZBtextList.at(0) + mesh.fileName;

		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V = mesh.V;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F = mesh.F;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &VC = mesh.VC;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FG = mesh.FG;
		read_OBJ(ZBtextList.at(1), V, F, VC, FG);

		params.scale = (params.height / (V.col(1).maxCoeff() - V.col(1).minCoeff()));
		V *= params.scale;
	}

	{
		params.outputFilePath = ZBtextList.at(0);
		params.outputFilePath.append("thickness.obj");
		params.acceleratorName = ZBtextList.at(2);
	}
	return;
}
