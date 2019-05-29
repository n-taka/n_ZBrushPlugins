// SampleTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "ThicknessChecker.h"

int main(int argc, char *argv[])
{
	//char dummyc0[100] = "../../testModels/,Dog.obj,NVIDIA GeForce RTX 2070 with Max-Q Design\0";
	char dummyc0[100] = "../../testModels/,Dog.obj,CPU\0";
	char dummyc1[4096], dummyc2[4096];
	char dummyc3[] = "hello";
	const double height = 100.0;
	const double preferThickness = 5.0;
	const double minThickness = 3.0;
	const int chunkSize = 500000;
	double dummyd = height * 1024.0 * 1024.0 + preferThickness * 1024.0 + minThickness;
	int dummyi0 = 0;
	int dummyi1 = 0;
	union {
		char c[sizeof(float)];
		float f;
		int i;
	} loader;
	loader.f = height;
	memcpy(dummyc1, loader.c, sizeof(float));
	loader.f = preferThickness;
	memcpy(dummyc1 + sizeof(float), loader.c, sizeof(float));
	loader.f = minThickness;
	memcpy(dummyc1 + sizeof(float) + sizeof(float), loader.c, sizeof(float));
	loader.i = chunkSize;
	memcpy(dummyc1 + sizeof(float) + sizeof(float) + sizeof(float), loader.c, sizeof(int));

	// getAccelerator(dummyc0, dummyd, dummyc1, dummyi0, dummyc2, dummyi1, (char **)&dummyc3);
	//std::cout << dummyc1 << std::endl;

	checkThickness(dummyc0, dummyd, dummyc1, dummyi0, dummyc2, dummyi1, (char **)&dummyc3);
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started:
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
