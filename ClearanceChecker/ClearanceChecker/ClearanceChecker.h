#pragma once
#if defined(_WIN32) || defined(_WIN64)
#include <Windows.h>
#endif

#include <unordered_map>
#include <vector>
#include <fstream>
#include "Eigen/Core"

#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

extern "C" DLLEXPORT float version(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData);

extern "C" DLLEXPORT float checkClearance(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData);

typedef struct Mesh_
{
	std::string fileName;
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> V;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> VC;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FG;
	std::vector<struct Mesh_> islands;
} Mesh;

typedef struct floatParams_
{
	float height;
	float minimumClearance;
} floatParams;

//////
// parse parameters given from ZBrush
// this function also opens logFile
//////
void parseParams(
	const char *someText,
	const double optValue,
	const char *outputBuffer,
	std::ofstream &logFile,
	std::vector<Mesh> &meshes,
	floatParams &params);

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

void splitIslands(
	std::vector<Mesh> &meshes);

void computeClearance(
	const std::vector<Mesh> &meshes,
	std::ofstream &logFile,
	std::unordered_map<int, std::unordered_map<int, float>> &clearance);

float debug(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F);
