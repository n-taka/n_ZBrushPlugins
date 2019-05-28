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

enum BOOLEAN_TYPE {
	NONE = 0,
	ORIGINAL = 1,
	ADDITION = 2,
	SUBTRACTION = 3,
	INTERSECTION = 4
};

extern "C" DLLEXPORT float version(char* someText, double optValue, char* outputBuffer, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

extern "C" DLLEXPORT float booleanTopological(char* someText, double optValue, char* outputBuffer, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

bool read_OBJ(
	const std::string& fileName,
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& V,
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& F,
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& VC,
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FG
);

bool write_OBJ(
	const std::string& fileName,
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& V,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& F,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& VC,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FG
);

bool compute_boolean(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& VA,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FA,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& VCA,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FGA,
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& VB,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FB,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& VCB,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FGB,
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& VC,
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FC,
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& VCC,
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FGC,
	BOOLEAN_TYPE& type
);

float debug(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& F
);
