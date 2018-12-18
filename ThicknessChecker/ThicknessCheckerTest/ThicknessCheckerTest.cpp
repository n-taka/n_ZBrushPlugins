// SampleTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "../ThicknessChecker/stdafx.h"
#include <iostream>
#include <fstream>
#include <dlfcn.h>

#if defined(_WIN32) || defined(_WIN64)
#else
#define MAX_PATH (260)
#endif

std::string GetFullPath()
{
    Dl_info module_info;
    if (dladdr(reinterpret_cast<void*>(GetFullPath), &module_info) == 0)
    {
        return std::string();
    }
    return std::string(module_info.dli_fname);
}

int main(int argc, char* argv[])
{

#if defined(_WIN32) || defined(_WIN64)
	::ShowWindow(::GetConsoleWindow(), SW_HIDE);
#endif

	std::cout << "Please do not touch ZBrush until this window closes!!!!" << std::endl;
    std::string path = GetFullPath();
    
    size_t dirEnd = path.find_last_of("/");
    if(dirEnd == std::string::npos)
    {
        // for windows environment.
        dirEnd = path.find_last_of("\\");
    }
    
    if(dirEnd == std::string::npos)
    {
        return 1;
    }
    std::string dir = path.substr(0, dirEnd+1);
    std::string failFile = dir;
    failFile.append("fail");
    try
    {
    dir.append(",thickness.OBJ"); // ZBrush use CAPITAL extension (this is very important for OSX)
    dir.append(",parameter.mem");
    dir.append(",thickness_color.OBJ");
    dir.append(",thickness_group.OBJ\0");

    char dummyc0[MAX_PATH];
		//char dummyc0[200] = "../../testModels/a.obj,_color,_group\0";
#if defined(_WIN32) || defined(_WIN64)
        strcpy_s(dummyc0, dir.c_str());
#else
        strcpy(dummyc0, dir.c_str());
#endif
		char dummyc1[MAX_PATH], dummyc2[MAX_PATH];
		char dummyc3[] = "hello";

		float height = 100;
		float preferredThickness = 3;
		float minimumThickness = 1;

		double dummyd = double(height) * 1024 * 1024 + double(preferredThickness) * 1024 + double(minimumThickness);
		int dummyi0 = 0;
		int dummyi1 = 0;
        detectThickness(dummyc0, dummyd, dummyc1, dummyi0, dummyc2, dummyi1, (char**)&dummyc3);
    }
    catch(int e)
    {
        std::ofstream fail(failFile.c_str(), std::ios::out);
        fail << "fail" << std::endl;
        fail.close();
        return e;
    }
    return 0;
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
