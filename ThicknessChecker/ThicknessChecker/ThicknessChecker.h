#pragma once
#if defined(_WIN32) || defined(_WIN64)
#include <Windows.h>
#endif

#include "Eigen/Core"

#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

extern "C" DLLEXPORT float version(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

extern "C" DLLEXPORT float AMP_getAccelerator(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

extern "C" DLLEXPORT float checkThickness(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

bool read_OBJ(
	const std::string& fileName,
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& V,
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& F,
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& VC,
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& FG
);

bool write_OBJ(
	const std::string& fileName,
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& V,
	const Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& F,
	const Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& VC,
	const Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& FG
);
