// SampleTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "ClearanceChecker/stdafx.h"
#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
	union {
		char c[sizeof(float)];
		float f;
	} loader;

	std::string baseDir(
		"../../testModels/");
	char dummyc0[100];
	sprintf(dummyc0, "%s", baseDir.append(",box.obj,PolySphere.OBJ\0").c_str());
	char dummyc1[4096], dummyc2[4096];
	char dummyc3[] = "hello";
	double dummyd = 0.0;
	int dummyi0 = 0;
	int dummyi1 = 0;
	const float height = 100.0f;
	const float minimumClearance = 0.03f;
	loader.f = height;
	memcpy(dummyc1, loader.c, sizeof(float));
	loader.f = minimumClearance;
	memcpy(dummyc1 + sizeof(float), loader.c, sizeof(float));

	checkClearance(dummyc0, dummyd, dummyc1, dummyi0, dummyc2, dummyi1, (char **)&dummyc3);
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
