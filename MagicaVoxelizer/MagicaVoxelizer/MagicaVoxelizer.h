#pragma once
#ifdef WIN32
#include <Windows.h>
#endif

#include <vector>
#include <string>
#include <fstream>
#include <map>

#include "Eigen/Core"

#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

extern "C" DLLEXPORT float magicaVoxelize(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

extern "C" DLLEXPORT float magicaDeVoxelize(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

class MagicaVoxelChunk
{
public:
	enum class chunkType
	{
		MAIN,
		PACK,
		SIZE,
		XYZI,
		RGBA,
		MATT,
		nTRN,
		nGRP,
		nSHP,
		MATL,
		LAYR,
		NONE
	};
	static const std::map<std::string, chunkType> strToType;
	static const std::map<chunkType, std::string> typeToStr;
	static const std::vector<unsigned int> default_palette;

	chunkType type;
	char id[4];
	int contentSizeInByte;
	std::vector<int> content;
	std::vector<float> matContent; // only used for MATT chunk;
	std::vector<MagicaVoxelChunk> children;

	// MagicaVoxelizer.cpp
	MagicaVoxelChunk(const std::string& type);
	~MagicaVoxelChunk();
	void appendChild(const MagicaVoxelChunk& child);
	bool writeVoxelToFile(const std::string& fileName) const;

	bool readVoxelFromFile(const std::string& fileName);

private:

	int sizeOfChunk(const bool recursive = true) const;
	int writeChunkToFile(std::ofstream& file) const;
};

bool read_OBJ(
	const std::string& fileName,
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& V,
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& F,
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& VC
);

bool write_OBJ(
	const std::string& fileName,
	const std::vector< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >& VV,
	const std::vector< Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> >& FF,
	const std::vector< Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> >& VCV
);
