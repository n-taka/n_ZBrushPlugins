// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include "ThicknessChecker.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>

#include "igl/jet.h"

////
// implementation
////
#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif


extern "C" DLLEXPORT float version(char* someText, double optValue, char* outputBuffer, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	return 1.2f;
}

extern "C" DLLEXPORT float checkThickness(char* someText, double optValue, char* outputBuffer, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	//// input
	// someText: file name to be opened
	// [0]: dir name
	// [1]: input filename
	// [2]: output filename (thickness as color)
	// [3]: accelerator name (optional)
	// optValue: encoded value for minimumThickness, betterThickness, height
	////

	////
	// [begin] decode parameters
	std::string ZBtext(someText);
	std::string separator(",");
	size_t separator_length = separator.length();
	std::vector<std::string> ZBtextList({});

	if (separator_length == 0) {
		ZBtextList.push_back(ZBtext);
	}
	else {
		size_t offset = std::string::size_type(0);
		while (true) {
			size_t pos = ZBtext.find(separator, offset);
			if (pos == std::string::npos) {
				ZBtextList.push_back(ZBtext.substr(offset));
				break;
			}
			ZBtextList.push_back(ZBtext.substr(offset, pos - offset));
			offset = pos + separator_length;
		}
	}
	std::string tmp;
	for (int i = 1; i < 3; ++i)
	{
		tmp = ZBtextList.at(i);
		ZBtextList.at(i) = ZBtextList.at(0) + tmp;
	}

	std::ofstream logFile(ZBtextList.at(0) + "log.txt");

	const std::string& inputFileName = ZBtextList.at(1);
	const std::string& outputFileName = ZBtextList.at(2);
	const std::string& acceleratorName = ZBtextList.at(3);

	std::cout << "Input file: " << inputFileName << std::endl;
	std::cout << "Output file: " << outputFileName << std::endl;
	std::cout << "Accelerator: " << acceleratorName << std::endl << std::endl;
	logFile << "Input file: " << inputFileName << std::endl;
	logFile << "Output file: " << outputFileName << std::endl;
	logFile << "Accelerator: " << acceleratorName << std::endl << std::endl;
	// [end] parameter decoding end.
	////

	////
	// [begin] read triangle from file
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> V;
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> F;
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> VC;
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> FG;
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> F_RAWSDF;

	read_OBJ(inputFileName, V, F, VC, FG);

	//const float minimumThickness = float(optValue - std::floor(optValue / 1024.0) * 1024.0);
	//const float preferredThickness = float(optValue / 1024.0 - std::floor(optValue / (1024.0 * 1024.0)) * 1024.0);
	//const float height = float(optValue / (1024.0 * 1024.0) - std::floor(optValue / (1024.0 * 1024.0 * 1024.0)) * 1024.0);
	union {
		char c[sizeof(float)];
		float f;
		int i;
	} loader;
	memcpy(loader.c, outputBuffer, sizeof(float));
	const float height = loader.f;
	memcpy(loader.c, outputBuffer + sizeof(float), sizeof(float));
	const float preferredThickness = loader.f;
	memcpy(loader.c, outputBuffer + sizeof(float) + sizeof(float), sizeof(float));
	const float minimumThickness = loader.f;
	memcpy(loader.c, outputBuffer + sizeof(float) + sizeof(float) + sizeof(float), sizeof(int));
	const int chunkSize = loader.i;

	std::cout << "minimumThickness   : " << minimumThickness << std::endl;
	std::cout << "preferredThickness : " << preferredThickness << std::endl;
	std::cout << "height             : " << height << std::endl;
	std::cout << "chunkSize          : " << chunkSize << std::endl << std::endl;
	logFile << "minimumThickness   : " << minimumThickness << std::endl;
	logFile << "preferredThickness : " << preferredThickness << std::endl;
	logFile << "height             : " << height << std::endl;
	logFile << "chunkSize          : " << chunkSize << std::endl << std::endl;
	// scale for make height (Y) is user-given height.
	const double scale = (height / (V.col(1).maxCoeff() - V.col(1).minCoeff()));
	V *= scale;
	// [end] read triangle from file
	////

	////
	// set up accelerator
	concurrency::accelerator acc = selectAccelerator(acceleratorName);
	////

	////
	// compute shape diameter function
	std::chrono::system_clock::time_point start, end;
	start = std::chrono::system_clock::now();
	computeSDF(V, F, chunkSize, acc, F_RAWSDF);
	end = std::chrono::system_clock::now();
	double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	std::cout << "elapsed time for computation: " << elapsed << " [ms]" << std::endl;
	logFile << "elapsed time for computation: " << elapsed << " [ms]" << std::endl;
	//std::cout << F_RAWSDF << std::endl;
	//return 0.0f;
	// todo
	////

	////
	// convert F_SDF to V_SDF (simple average)
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V_SDF;
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> V_Count;
	V_SDF.resize(V.rows(), 1);
	V_SDF.setZero();
	V_Count.resize(V.rows(), 1);
	V_Count.setZero();
	for (int f = 0; f < F.rows(); ++f)
	{
		for (int v = 0; v < F.cols(); ++v)
		{
			V_SDF(F(f, v), 0) += F_RAWSDF(f, 0);
			V_Count(F(f, v), 0) += 1;
		}
	}
	for (int v = 0; v < V.rows(); ++v)
	{
		V_SDF(v, 0) /= double(V_Count(v, 0));
	}
	////

	////
	// export with
	// jet color (update polypaint, keep polygroup)
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> VC_Thicknessd;
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> VC_Thicknessi;
	igl::jet(V_SDF, preferredThickness, minimumThickness, VC_Thicknessd);
	//std::cout << V_SDF.transpose() << std::endl;
	//igl::jet(V_SDF, V_SDF.minCoeff(), V_SDF.maxCoeff(), VC_Thicknessd);
	std::cout << V_SDF.minCoeff() << std::endl;
	std::cout << V_SDF.maxCoeff() << std::endl;
	VC_Thicknessi.resize(VC_Thicknessd.rows(), 4);
	for (int v = 0; v < V.rows(); ++v)
	{
		VC_Thicknessi(v, 0) = 255; // A (M?)
		VC_Thicknessi(v, 1) = int(VC_Thicknessd(v, 0) * 255); // R
		VC_Thicknessi(v, 2) = int(VC_Thicknessd(v, 1) * 255); // G
		VC_Thicknessi(v, 3) = int(VC_Thicknessd(v, 2) * 255); // B
	}
	V /= scale;
	write_OBJ(outputFileName, V, F, VC_Thicknessi, FG);
	////
	logFile.close();

	return 1.0f;
}
