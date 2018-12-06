// Sample.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include <iostream>

extern "C" __declspec(dllexport) float test(char* pDontCare, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

extern "C" __declspec(dllexport) float test(char* pDontCare, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	std::cout << "test function!" << std::endl;
	return 0.0f;
}