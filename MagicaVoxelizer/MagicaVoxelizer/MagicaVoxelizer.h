#pragma once
#include <Windows.h>
#include <vector>
#include <string>
#include <fstream>

extern "C" __declspec(dllexport) float magicaVoxelize(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

class MagicaVoxelChunk
{
	enum class chunkType
	{
		MAIN,
		PACK,
		SIZE,
		XYZI,
		RGBA,
		MATT
	};

	chunkType type;
	char id[4];
	int contentSizeInByte;
	std::vector<int> content;
	std::vector<float> matContent; // only used for MATT chunk;
	std::vector<MagicaVoxelChunk> children;

	int sizeOfChunk(const bool recursive = true) const;
	int writeChunkToFile(std::ofstream& file) const;
	bool writeVoxelToFile(const std::string& fileName) const;
};
