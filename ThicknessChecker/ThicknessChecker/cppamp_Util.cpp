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

extern "C" DLLEXPORT float AMP_getAccelerator(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	std::vector<concurrency::accelerator> accels;
	accels = concurrency::accelerator::get_all();

	accels.erase(std::remove_if(accels.begin(), accels.end(), [](const concurrency::accelerator& accel) {return accel.get_is_emulated(); }), accels.end());
	for (const auto& acc : accels)
	{
		getAccelDiscription(acc);
	}

	return 0.0f;
}
