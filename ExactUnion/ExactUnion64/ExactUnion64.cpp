// ExactUnion64.cpp : Defines the exported functions for the DLL application.
//

#define DLLEXPORT __declspec(dllexport)

#include "igl/readOBJ.h"
#include "igl/writeOBJ.h"
#include "Eigen/Core"

extern "C" {
	float DLLEXPORT computeUnion(char* someText, double optValue, char* outputBuffer, int optBuffer1Size,
		char* pOptBuffer2, int optBuffer2Size, char** zData)
	{
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V;
		Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> F;
		igl::readOBJ(someText, V, F);
		V.col(1) *= 2;
		igl::writeOBJ(someText, V, F);

		return 0.0f;
	}
}
