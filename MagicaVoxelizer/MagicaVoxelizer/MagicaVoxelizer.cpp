// Sample.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "Eigen/Core"
#include "igl/read_triangle_mesh.h"
#include "igl/signed_distance.h"
#include <iostream>
#include <fstream>

// see https://github.com/ephtracy/voxel-model/blob/master/MagicaVoxel-file-format-vox.txt
	////
	// implementation
	////
int MagicaVoxelChunk::sizeOfChunk(const bool recursive) const
{
	// chunkID + contentSize + childrenSize + itsContent
	int size = sizeof(char) * 4 + sizeof(int) + sizeof(int) + sizeof(int)*content.size() + sizeof(float)*matContent.size();
	if (recursive)
	{
		for (const auto& c : children)
			size += (c.sizeOfChunk(true));
	}
	return size;
}

int MagicaVoxelChunk::writeChunkToFile(std::ofstream& file) const
{
	int byteWritten = 0;
	bool success = true;
	try
	{
		// chunk id
		switch (type)
		{
		case chunkType::MAIN:
			file.write("MAIN", sizeof(unsigned char) * 4);
			break;
		case chunkType::PACK:
			file.write("PACK", sizeof(unsigned char) * 4);
			break;
		case chunkType::SIZE:
			file.write("SIZE", sizeof(unsigned char) * 4);
			break;
		case chunkType::XYZI:
			file.write("XYZI", sizeof(unsigned char) * 4);
			break;
		case chunkType::RGBA:
			file.write("RGBA", sizeof(unsigned char) * 4);
			break;
		case chunkType::MATT:
			file.write("MATT", sizeof(unsigned char) * 4);
			break;
		default:
			break;
		}
		byteWritten += sizeof(unsigned char) * 4;

		// byte of content
		const int byteOfContent = sizeof(int)*content.size() + sizeof(float)*matContent.size();
		file.write((const char*)&byteOfContent, sizeof(int) * 1);
		byteWritten += sizeof(int) * 1;

		// byte of children chunks
		const int byteOfChildren = sizeOfChunk(true) - (sizeof(char) * 4 + sizeof(int) + sizeof(int) + sizeof(int)*content.size() + sizeof(float)*matContent.size());
		file.write((const char*)&byteOfChildren, sizeof(int) * 1);
		byteWritten += sizeof(int) * 1;

		// content
		switch (type)
		{
		case chunkType::MAIN:
			// no content
			break;
		case chunkType::PACK:
		case chunkType::SIZE:
		case chunkType::XYZI:
		case chunkType::RGBA:
			file.write((const char*)&content[0], sizeof(int) * content.size());
			byteWritten += sizeof(int) * content.size();
			// content.size() ==
			// PACK: 1
			// SIZE: 3
			// XYZI: (1+N)
			// RGBA: 256
			// matContent.size() == 0
			break;
		case chunkType::MATT:
			file.write((const char*)&content[0], sizeof(int) * 2);
			byteWritten += sizeof(int) * 2;
			file.write((const char*)&matContent[0], sizeof(float) * 1);
			byteWritten += sizeof(float) * 1;
			file.write((const char*)&content[2], sizeof(int) * 1);
			byteWritten += sizeof(int) * 1;
			file.write((const char*)&matContent[1], sizeof(float) * (matContent.size() - 1));
			byteWritten += sizeof(float) * (matContent.size() - 1);
			// content.size() == 3
			// matContent.size() == (1+N)
			break;
		default:
			break;
		}

	}
	catch (int e) {
		return false;
	}

	// children
	for (const auto& c : children)
	{
		byteWritten += c.writeChunkToFile(file);
		if (!success) {
			return -1;
		}
	}
	return byteWritten;
}

bool MagicaVoxelChunk::writeVoxelToFile(const std::string& fileName) const
{
	if (type == chunkType::MAIN)
	{
		std::ofstream file;
		file.open(fileName, std::ios::out | std::ios::binary | std::ios::trunc);
		if (!file)
		{
			std::cerr << "cannot open file." << std::endl;
			return false;
		}

		// write header
		int version = 150;
		file.write("VOX ", sizeof(unsigned char) * 4);
		file.write((const char*)&version, sizeof(int));

		return (writeChunkToFile(file) > 0);
	}
	else
	{
		std::cerr << "writeToFile(const std::string&) const must be called with MAIN chunk." << std::endl;
		return false;
	}
}

extern "C" __declspec(dllexport) float magicaVoxelize(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData);

extern "C" __declspec(dllexport) float magicaVoxelize(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	//// input
	// someText: file name to be opened
	// optValue: voxel resolution in Y coordinate
	////
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V;
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> F;
	Eigen::Matrix<double, Eigen::Dynamic, 1> S;
	Eigen::Matrix<double, Eigen::Dynamic, 1> I;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> N;

	// read triangle from file
	igl::read_triangle_mesh(someText, V, F);

	// generate voxel for signed distance computation (query point is center of each voxel)
	Eigen::Matrix<double, 1, Eigen::Dynamic> BBSize = V.colwise().maxCoeff() - V.colwise().minCoeff();
	Eigen::Matrix<double, 1, Eigen::Dynamic> BBCenter = (V.colwise().maxCoeff() + V.colwise().minCoeff()) / 2;
	const double oneVoxelSize = BBSize(0, 1) / optValue;
	Eigen::Matrix<int, 1, Eigen::Dynamic> voxelCount;
	voxelCount.resize(1, 3);
	voxelCount << int(ceil(BBSize(0, 0) / oneVoxelSize)), int(ceil(BBSize(0, 1) / oneVoxelSize)), int(ceil(BBSize(0, 2) / oneVoxelSize));
	P.resize(voxelCount(0, 0)*voxelCount(0, 1)*voxelCount(0, 2), 3);

	// build queries
	const Eigen::Matrix<double, 1, Eigen::Dynamic> bottomCorner = (V.colwise().minCoeff().array() + oneVoxelSize);
#define VOX(x, y, z) ((z)*(voxelCount(0,1))*(voxelCount(0,0)) + (y)*(voxelCount(0,0)) + (x))
	for (int z = 0; z < voxelCount(0, 2); ++z)
	{
		for (int y = 0; y < voxelCount(0, 1); ++y)
		{
			for (int x = 0; x < voxelCount(0, 0); ++x)
			{
				const int voxelIndex = VOX(x, y, z);
				P.row(voxelIndex) = bottomCorner;
				P(voxelIndex, 0) += oneVoxelSize * x;
				P(voxelIndex, 1) += oneVoxelSize * y;
				P(voxelIndex, 2) += oneVoxelSize * z;
			}
		}
	}

	// signed distance computation (0<: outside, <0: inside)
	const igl::SignedDistanceType signedDistanceType = igl::SIGNED_DISTANCE_TYPE_DEFAULT;
	igl::signed_distance(P, V, F, signedDistanceType, S, I, C, N);

	// voxelize
	// TODO

	//std::cout << V << std::endl << std::endl;
	//std::cout << F << std::endl << std::endl;
	std::cout << "test function!" << std::endl;
	return 0.0f;
}