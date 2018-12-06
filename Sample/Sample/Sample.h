#pragma once
#include <Windows.h>

extern "C" __declspec(dllexport) float test(char* pDontCare, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);
