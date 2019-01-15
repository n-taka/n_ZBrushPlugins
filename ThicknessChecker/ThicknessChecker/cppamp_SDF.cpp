// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include "ThicknessChecker.h"

////
// implementation
////
#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

