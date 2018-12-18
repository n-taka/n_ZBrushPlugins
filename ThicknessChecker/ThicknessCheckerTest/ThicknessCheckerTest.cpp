// SampleTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "../ThicknessChecker/stdafx.h"
#include <iostream>
#include <fstream>

int main(int argc, char* argv[])
{

#if defined(_WIN32) || defined(_WIN64)
	::ShowWindow(::GetConsoleWindow(), SW_HIDE);
#endif

	std::cout << "Please do not touch ZBrush until this window closes!!!!" << std::endl;
	char Path[MAX_PATH + 1];

	if (0 != GetModuleFileName(NULL, Path, MAX_PATH))
	{
		char drive[MAX_PATH + 1]
			, dir[MAX_PATH + 1]
			, fname[MAX_PATH + 1]
			, ext[MAX_PATH + 1];

		_splitpath_s(Path, drive, dir, fname, ext);

		//printf("完全パス : %s\n", Path);
		//printf("ドライブ : %s\n", drive);
		//printf("ディレクトリ パス : %s\n", dir);
		//printf("ベース ファイル名 (拡張子なし) : %s\n", fname);
		//printf("ファイル名の拡張子 : %s\n", ext);

		std::string str(dir);
		str.append(",thickness.obj");
		str.append(",parameter.mem");
		str.append(",thickness_color.obj");
		str.append(",thickness_group.obj\0");
		char dummyc0[200];
		//char dummyc0[200] = "../../testModels/a.obj,_color,_group\0";
		strcpy_s(dummyc0, str.c_str());
		char dummyc1[100], dummyc2[100];
		char dummyc3[] = "hello";

		float height = 100;
		float preferredThickness = 3;
		float minimumThickness = 1;

		double dummyd = double(height) * 1024 * 1024 + double(preferredThickness) * 1024 + double(minimumThickness);
		int dummyi0 = 0;
		int dummyi1 = 0;
		detectThickness(dummyc0, dummyd, dummyc1, dummyi0, dummyc2, dummyi1, (char**)&dummyc3);
		return 0;
	}
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
