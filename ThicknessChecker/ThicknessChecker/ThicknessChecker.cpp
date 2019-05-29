// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "ThicknessChecker.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include "igl/jet.h"

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
	return 1.2f;
}

extern "C" DLLEXPORT float checkThickness(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData)
{
	std::ofstream logFile;
	std::vector<Mesh> meshes;
	Params params;
	parseParams(someText, optValue, outputBuffer, logFile, meshes, params);
#if defined(_WIN32) || defined(_WIN64)
	// Windows: GPU or CPU
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> F_RAWSDF;
	std::chrono::system_clock::time_point start, end;
	start = std::chrono::system_clock::now();

	if (params.acceleratorName != "CPU")
	{
		// GPU
		concurrency::accelerator acc = selectAccelerator(params.acceleratorName, logFile);
		AMP_computeSDF(meshes.at(0).V, meshes.at(0).F, params.chunkSize, acc, F_RAWSDF);
		end = std::chrono::system_clock::now();
		double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		std::cout << "elapsed time for computation: " << elapsed << " [ms]" << std::endl;
		logFile << "elapsed time for computation: " << elapsed << " [ms]" << std::endl;
	}
	else
	{
		// CPU
		std::cout << "accelerator: CPU" << std::endl;
		logFile << "accelerator: CPU" << std::endl;
		const int numOfThreads = std::max(static_cast<int>(std::thread::hardware_concurrency()), 1);
		std::cout << "Compute with " << numOfThreads << " threads." << std::endl;
		logFile << "Compute with " << numOfThreads << " threads." << std::endl;

		CPU_computeSDF(meshes.at(0).V, meshes.at(0).F, F_RAWSDF);
		end = std::chrono::system_clock::now();
		double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		std::cout << "elapsed time for computation: " << elapsed << " [ms]" << std::endl;
		logFile << "elapsed time for computation: " << elapsed << " [ms]" << std::endl;
	}


#else
// Mac: CPU
	std::cout << "accelerator: CPU" << std::endl;
	logFile << "accelerator: CPU" << std::endl;

	CPU_computeSDF(meshes.at(0).V, meshes.at(0).F, F_RAWSDF);
	end = std::chrono::system_clock::now();
	double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	std::cout << "elapsed time for computation: " << elapsed << " [ms]" << std::endl;
	logFile << "elapsed time for computation: " << elapsed << " [ms]" << std::endl;
#endif

	////
	// convert F_SDF to V_SDF (simple average)
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V_SDF;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> V_Count;
	V_SDF.resize(meshes.at(0).V.rows(), 1);
	V_SDF.setZero();
	V_Count.resize(meshes.at(0).V.rows(), 1);
	V_Count.setZero();
	for (int f = 0; f < meshes.at(0).F.rows(); ++f)
	{
		for (int v = 0; v < meshes.at(0).F.cols(); ++v)
		{
			V_SDF(meshes.at(0).F(f, v), 0) += F_RAWSDF(f, 0);
			V_Count(meshes.at(0).F(f, v), 0) += 1;
		}
	}
	for (int v = 0; v < meshes.at(0).V.rows(); ++v)
	{
		V_SDF(v, 0) /= double(V_Count(v, 0));
	}
	////

	////
	// export with
	// jet color (update polypaint, keep polygroup)
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> VC_Thicknessd;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> VC_Thicknessi;
	igl::jet(V_SDF, params.preferredThickness, params.minimumThickness, VC_Thicknessd);
	//std::cout << V_SDF.transpose() << std::endl;
	//igl::jet(V_SDF, V_SDF.minCoeff(), V_SDF.maxCoeff(), VC_Thicknessd);
	std::cout << V_SDF.minCoeff() << std::endl;
	std::cout << V_SDF.maxCoeff() << std::endl;
	VC_Thicknessi.resize(VC_Thicknessd.rows(), 4);
	for (int v = 0; v < meshes.at(0).V.rows(); ++v)
	{
		VC_Thicknessi(v, 0) = 255;							  // A (M?)
		VC_Thicknessi(v, 1) = int(VC_Thicknessd(v, 0) * 255); // R
		VC_Thicknessi(v, 2) = int(VC_Thicknessd(v, 1) * 255); // G
		VC_Thicknessi(v, 3) = int(VC_Thicknessd(v, 2) * 255); // B
	}
	meshes.at(0).V /= params.scale;
	write_OBJ(params.outputFilePath, meshes.at(0).V, meshes.at(0).F, VC_Thicknessi, meshes.at(0).FG);
	////
	logFile.close();

	return 1.0f;
}
