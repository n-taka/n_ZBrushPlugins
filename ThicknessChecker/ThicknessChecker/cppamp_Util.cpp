// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "ThicknessChecker.h"

#include <vector>
#include <iostream>

////
// implementation
////
#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

namespace
{
#if defined(_WIN32) || defined(_WIN64)
// this function is only for windows
void getAccelDiscription(const concurrency::accelerator &accel, std::ofstream &logFile)
{
	std::wcout << "accelerator: " << accel.get_description() << std::endl;
	std::cout << "version of the accelerator: " << accel.get_version() << std::endl;
	std::cout << "memory: " << accel.get_dedicated_memory() / 1024. / 1000. << " [GB]" << std::endl;
	std::cout << "is supporting double precision: " << (accel.get_supports_double_precision() ? "yes" : "no") << std::endl;
	std::cout << "is attached to a display: " << (accel.get_has_display() ? "yes" : "no") << std::endl;
	std::cout << "is supporting cpu shared memory: " << (accel.get_supports_cpu_shared_memory() ? "yes" : "no") << std::endl;

	std::wstring wtmp = accel.get_description();
	std::string tmp(wtmp.begin(), wtmp.end());
	logFile << "accelerator: " << tmp << std::endl;
	logFile << "version of the accelerator: " << accel.get_version() << std::endl;
	logFile << "memory: " << accel.get_dedicated_memory() / 1024. / 1000. << " [GB]" << std::endl;
	logFile << "is supporting double precision: " << (accel.get_supports_double_precision() ? "yes" : "no") << std::endl;
	logFile << "is attached to a display: " << (accel.get_has_display() ? "yes" : "no") << std::endl;
	logFile << "is supporting cpu shared memory: " << (accel.get_supports_cpu_shared_memory() ? "yes" : "no") << std::endl;
	return;
}
#endif
} // namespace

extern "C" DLLEXPORT float getAccelerator(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData)
{
#define ACC_NAME
	//#define ACC_VERSION
#define ACC_VRAM
	//#define ACC_PRECISION
	//#define ACC_SHARED_RAM

	std::string str("");

#if defined(_WIN32) || defined(_WIN64)
	std::vector<concurrency::accelerator> accels;
	accels = concurrency::accelerator::get_all();

	accels.erase(std::remove_if(accels.begin(), accels.end(), [](const concurrency::accelerator &accel) { return accel.get_is_emulated(); }), accels.end());

	for (int accIdx = 0; accIdx < accels.size(); ++accIdx)
	{
		concurrency::accelerator &acc = accels.at(accIdx);

#ifdef ACC_NAME
		// accelerator name
		std::wstring wstr = acc.get_description();
		std::string tmpStr(wstr.begin(), wstr.end());
		str += tmpStr;
		str += ",";
#endif
#ifdef ACC_VERSION
		// accelerator version
		str += "Version: ";
		str += std::to_string(acc.get_version());
		str += "\n";
#endif
#ifdef ACC_VRAM
		// GPU memory
		str += "GPU memory: ";
		// I don't know ... which one is correct, 1GB = 10^3 x 10^3 or 2^10 x 2^10 ??
		str += std::to_string(acc.get_dedicated_memory() / 1024.0 / 1000.0);
		str += " [GB]\n";
#endif
#ifdef ACC_PRECISION
		// double precision
		str += "Double precision: ";
		str += (acc.get_supports_double_precision() ? "yes" : "no");
		str += "\n";
		// Attached to a display
		str += "Attached to a display: ";
		str += (acc.get_has_display() ? "yes" : "no");
		str += "\n";
#endif
#ifdef ACC_SHARED_RAM
		// CPU shared memory
		str += "CPU shared memory: ";
		str += (acc.get_supports_cpu_shared_memory() ? "yes" : "no");
		//str += "\n";
#endif
		str += ";";
	}

#endif

	// for both of Windows/Mac, we support CPU computation
	str += "CPU,Do not use GPU";

	sprintf(outputBuffer, "%s", str.c_str());
	return 1.0f;
}

#if defined(_WIN32) || defined(_WIN64)
// this function is only for windows
concurrency::accelerator selectAccelerator(const std::string &acceleratorName, std::ofstream &logFile)
{
	std::vector<concurrency::accelerator> accels;
	accels = concurrency::accelerator::get_all();

	accels.erase(std::remove_if(accels.begin(), accels.end(), [](const concurrency::accelerator &accel) { return accel.get_is_emulated(); }), accels.end());

	for (int accIdx = 0; accIdx < accels.size(); ++accIdx)
	{
		concurrency::accelerator &acc = accels.at(accIdx);

		// accelerator name
		std::wstring wstr = acc.get_description();
		std::string tmpStr(wstr.begin(), wstr.end());
		if (tmpStr == acceleratorName)
		{
			std::cout << "[[ selected accelerator ]]" << std::endl;
			logFile << "[[ selected accelerator ]]" << std::endl;
			getAccelDiscription(acc, logFile);

			return acc;
		}
	}

	concurrency::accelerator defaultAcc = concurrency::accelerator(concurrency::accelerator::default_accelerator);
	std::cout << "[[ selected accelerator ]]" << std::endl;
	logFile << "[[ selected accelerator ]]" << std::endl;
	getAccelDiscription(defaultAcc, logFile);
	return defaultAcc;
}
#endif
