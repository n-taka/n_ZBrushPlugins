#pragma once
#if defined(_WIN32) || defined(_WIN64)
#include <Windows.h>
#endif

#include <unordered_map>
#include "Eigen/Core"
#include "igl/AABB.h"

#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

extern "C" DLLEXPORT float version(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData);

extern "C" DLLEXPORT float checkClearance(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData);

typedef struct meshWithAABB_
{
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> V;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> VC;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FG;
	igl::AABB<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, 3> AABB;
} meshWithAABB;

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

void computeClearance(
	const std::vector<meshWithAABB> &meshes,
	std::unordered_map<int, std::unordered_map<int, float>> &clearance);

float debug(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F);
