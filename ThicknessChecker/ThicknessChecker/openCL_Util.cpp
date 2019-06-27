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
// this function is only for windows
void getAccelDiscription(const cl::Device &device, std::ofstream &logFile)
{
	std::cout << device.getInfo<CL_DEVICE_NAME>() << std::endl;
	std::cout << "Device vender: ";
	std::cout << device.getInfo<CL_DEVICE_VENDOR>() << std::endl;
	std::cout << "Device version: ";
	std::cout << device.getInfo<CL_DEVICE_VERSION>() << std::endl;
	std::cout << "Driver version: ";
	std::cout << device.getInfo<CL_DRIVER_VERSION>() << std::endl;
	std::cout << "Extensions: ";
	std::cout << device.getInfo<CL_DEVICE_EXTENSIONS>() << std::endl;

	logFile << device.getInfo<CL_DEVICE_NAME>() << std::endl;
	logFile << "Device vender: ";
	logFile << device.getInfo<CL_DEVICE_VENDOR>() << std::endl;
	logFile << "Device version: ";
	logFile << device.getInfo<CL_DEVICE_VERSION>() << std::endl;
	logFile << "Driver version: ";
	logFile << device.getInfo<CL_DRIVER_VERSION>() << std::endl;
	logFile << "Extensions: ";
	logFile << device.getInfo<CL_DEVICE_EXTENSIONS>() << std::endl;
	return;
}
} // namespace

extern "C" DLLEXPORT float getAccelerator(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData)
{
#define ACC_NAME
	// #define ACC_VENDER
	// #define ACC_DEVICE_VERSION
	// #define ACC_DRIVER_VERSION
	// #define ACC_EXT

	std::string str("");
	try
	{
		std::vector<cl::Platform> platforms;
		cl::Platform::get(&platforms);
		if (platforms.size() == 0)
		{
			std::cerr << "No platform found." << std::endl;
			return 0.0f;
		}
		for (auto &platform : platforms)
		{
			std::vector<cl::Device> devices;
			platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);
			for (auto &device : devices)
			{
#ifdef ACC_NAME
				// accelerator name
				str += device.getInfo<CL_DEVICE_NAME>();
				str.erase(std::find(str.begin(), str.end(), '\0'), str.end());
				str += "\n";
#endif
#ifdef ACC_VENDER
				// accelerator name
				str += "Device vender: ";
				str += device.getInfo<CL_DEVICE_VENDOR>();
				str.erase(std::find(str.begin(), str.end(), '\0'), str.end());
				str += "\n";
#endif
#ifdef ACC_DEVICE_VERSION
				// accelerator version
				str += "Device version: ";
				str += device.getInfo<CL_DEVICE_VERSION>();
				str.erase(std::find(str.begin(), str.end(), '\0'), str.end());
				str += "\n";
#endif
#ifdef ACC_DRIVER_VERSION
				// accelerator version
				str += "Driver version: ";
				str += device.getInfo<CL_DRIVER_VERSION>();
				str.erase(std::find(str.begin(), str.end(), '\0'), str.end());
				str += "\n";
#endif
#ifdef ACC_EXT
				str += "Extensions: ";
				str += device.getInfo<CL_DEVICE_EXTENSIONS>();
				str.erase(std::find(str.begin(), str.end(), '\0'), str.end());
				str += "\n";
#endif
			}
		}
		sprintf(outputBuffer, "%s", str.c_str());
		return 1.0f;
	}
	catch (cl::Error const &ex)
	{
		std::cerr << "OpenCL Error: " << ex.what() << " (code " << ex.err() << ")" << std::endl;
		return 0.0f;
	}
	catch (std::exception const &ex)
	{
		std::cerr << "Exception: " << ex.what() << std::endl;
		return 0.0f;
	}
}

cl::Device selectAccelerator(const std::string &acceleratorName, std::ofstream &logFile)
{
	try
	{
		std::vector<cl::Platform> platforms;
		cl::Platform::get(&platforms);
		if (platforms.size() == 0)
		{
			std::cerr << "No platform found." << std::endl;
		}
		for (auto &platform : platforms)
		{
			std::vector<cl::Device> devices;
			platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);
			for (auto &device : devices)
			{
				// accelerator name
				std::string tmpStr = device.getInfo<CL_DEVICE_NAME>();
				tmpStr.erase(std::find(tmpStr.begin(), tmpStr.end(), '\0'), tmpStr.end());
				if (acceleratorName == tmpStr)
				{
					std::cout << "[[ selected accelerator ]]" << std::endl;
					logFile << "[[ selected accelerator ]]" << std::endl;
					getAccelDiscription(device, logFile);

					return device;
				}
			}
		}
	}
	catch (cl::Error const &ex)
	{
		std::cerr << "OpenCL Error: " << ex.what() << " (code " << ex.err() << ")" << std::endl;
	}
	catch (std::exception const &ex)
	{
		std::cerr << "Exception: " << ex.what() << std::endl;
	}
	return cl::Device();
}
