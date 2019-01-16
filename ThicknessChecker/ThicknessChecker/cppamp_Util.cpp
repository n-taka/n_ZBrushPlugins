// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include "ThicknessChecker.h"

#include <vector>
#include <amp.h>
#include <iostream>

////
// implementation
////
#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

namespace {
	void getAccelDiscription(const concurrency::accelerator& accel) {
		std::wcout << "accelerator: " << accel.get_description() << std::endl;
		std::cout << "version of the accelerator: " << accel.get_version() << std::endl;
		std::cout << "memory: " << accel.get_dedicated_memory() / 1024. / 1000. << " [GB]" << std::endl;;
		std::cout << "is supporting double precision: " << (accel.get_supports_double_precision() ? "yes" : "no") << std::endl;
		std::cout << "is attached to a display: " << (accel.get_has_display() ? "yes" : "no") << std::endl;
		std::cout << "is supporting cpu shared memory: " << (accel.get_supports_cpu_shared_memory() ? "yes" : "no") << std::endl;
		return;
	}
}

extern "C" DLLEXPORT float AMP_getAccelerator(char* someText, double optValue, char* outputBuffer, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	std::vector<concurrency::accelerator> accels;
	accels = concurrency::accelerator::get_all();

	accels.erase(std::remove_if(accels.begin(), accels.end(), [](const concurrency::accelerator& accel) {return accel.get_is_emulated(); }), accels.end());

	std::string str("");

	for (int accIdx = 0; accIdx < accels.size(); ++accIdx)
	{
		concurrency::accelerator& acc = accels.at(accIdx);

		// accelerator name
		std::wstring wstr = acc.get_description();
		std::string tmpStr(wstr.begin(), wstr.end());
		str += tmpStr;
		str += ",";
#if 0
		// accelerator version
		str += "Version: ";
		str += std::to_string(acc.get_version());
		str += "\n";
#endif
		// GPU memory
		str += "GPU memory: ";
		// I don't know ... which one is correct, 1GB = 10^3 x 10^3 or 2^10 x 2^10 ??
		str += std::to_string(acc.get_dedicated_memory() / 1024.0 / 1000.0);
		str += " [GB]\n";
#if 0
		// double precision
		str += "Double precision: ";
		str += (acc.get_supports_double_precision() ? "yes" : "no");
		str += "\n";
		// Attached to a display
		str += "Attached to a display: ";
		str += (acc.get_has_display() ? "yes" : "no");
		str += "\n";
#endif
		// CPU shared memory
		str += "CPU shared memory: ";
		str += (acc.get_supports_cpu_shared_memory() ? "yes" : "no");
		str += "\n";

		if (accIdx < accels.size() - 1)
		{
			str += ";";
		}
	}

	sprintf(outputBuffer, "%s", str.c_str());

	return 0.0f;
}
