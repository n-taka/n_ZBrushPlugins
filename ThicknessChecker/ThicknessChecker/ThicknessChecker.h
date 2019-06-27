#pragma once
#define __CL_ENABLE_EXCEPTIONS
#if defined(_WIN32) || defined(_WIN64)
#include <Windows.h>
#include <CL/cl.hpp>
#else
#include <OpenCL/cl.hpp>
#endif

#include <vector>
#include <fstream>
#include "Eigen/Core"

#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

extern "C" DLLEXPORT float version(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData);

extern "C" DLLEXPORT float checkThickness(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData);

extern "C" DLLEXPORT float getAccelerator(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData);

cl::Device selectAccelerator(const std::string &acceleratorName, std::ofstream &logFile);

typedef struct Mesh_
{
	std::string fileName;
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> V;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> VC;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FG;
	std::vector<struct Mesh_> islands;
} Mesh;

typedef struct Params_
{
	float height;
	float preferredThickness;
	float minimumThickness;
	float scale;
	int chunkSize;
	std::string outputFilePath;
	std::string acceleratorName;
} Params;

void parseParams(
	const char *someText,
	const double optValue,
	const char *outputBuffer,
	std::ofstream &logFile,
	std::vector<Mesh> &meshes,
	Params &params);

bool read_OBJ(
	const std::string &fileName,
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V,
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F,
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &VC,
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FG);

bool write_OBJ(
	const std::string &fileName,
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &VC,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FG);

void openCL_computeSDF(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F,
	const int &chunkSize,
	const cl::Device &device,
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &FaceSDF);
