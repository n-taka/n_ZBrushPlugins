// SampleTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "../BooleanTopological/stdafx.h"
#include <iostream>


int main(int argc, char* argv[])
{
	char dummyc0[100] = "../../testModels/,hand_tri.obj:O,cube_tri.obj:S\0";
	char dummyc1[4096], dummyc2[4096];
	char dummyc3[] = "hello";
	double dummyd = 0.0;
	int dummyi0 = 0;
	int dummyi1 = 0;

	booleanTopological(dummyc0, dummyd, dummyc1, dummyi0, dummyc2, dummyi1, (char**)&dummyc3);
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