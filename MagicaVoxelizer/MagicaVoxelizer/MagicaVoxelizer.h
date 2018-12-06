#pragma once
#include <Windows.h>

extern "C" __declspec(dllexport) float magicaVoxelize(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);
