// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include <fstream>
#include "igl/copyleft/marching_cubes.h"
#include "igl/writeOBJ.h"
#include "MagicaVoxelizer.h"

#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

extern "C" DLLEXPORT float magicaDeVoxelize(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	std::string ZBtext(someText);
	std::string separator(",");
	size_t separator_length = separator.length();

	std::vector<std::string> ZBtextList({});

	if (separator_length == 0) {
		ZBtextList.push_back(ZBtext);
	}
	else {
		size_t offset = std::string::size_type(0);
		while (true) {
			size_t pos = ZBtext.find(separator, offset);
			if (pos == std::string::npos) {
				ZBtextList.push_back(ZBtext.substr(offset));
				break;
			}
			ZBtextList.push_back(ZBtext.substr(offset, pos - offset));
			offset = pos + separator_length;
		}
	}

#if !(defined(_WIN32) || defined(_WIN64))
	// if Mac, ZBrush gives me invalid prefix with FileNameResolvePath ...
    for(auto& s : ZBtextList)
    {
        s.erase(s.begin(), s.begin()+2);
    }
#endif
    
	MagicaVoxelChunk mainChunk(std::string("MAIN", 4));
	mainChunk.readVoxelFromFile(ZBtextList.at(0));

	std::vector< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > SS;
	std::vector< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > PP;
	std::vector< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > VV;
	std::vector< Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> > FF;
	std::vector< Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> > VCV;

	std::vector<unsigned int> palette = MagicaVoxelChunk::default_palette;

	if (optValue <= 0)
	{
		for (int c = 0; c < mainChunk.children.size(); ++c)
		{
			const auto& chunk = mainChunk.children.at(c);
			// find RGBA chunk (if omitted, use default palette)
			if (chunk.type == MagicaVoxelChunk::chunkType::RGBA)
			{
				memcpy(palette.data(), chunk.content.data(), sizeof(int) * 256);
				break;
			}
		}
	}

	for (int c = 0; c < mainChunk.children.size(); ++c)
	{
		const auto& chunk = mainChunk.children.at(c);
		// find SIZE chunk (currently we assume there is only one model in .vox file.)
		if (chunk.type == MagicaVoxelChunk::chunkType::SIZE)
		{
			Eigen::Matrix<int, 1, Eigen::Dynamic> voxelCount;
			voxelCount.resize(1, 3);
			// xyz (MagicaVoxel) --> zxy(ZBrush)
			voxelCount << chunk.content.at(1) + 2, chunk.content.at(2) + 2, chunk.content.at(0) + 2;

			SS.push_back(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>());
			PP.push_back(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>());
			VV.push_back(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>());
			FF.push_back(Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>());
			VCV.push_back(Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>());

			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& S = SS.back();
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& P = PP.back();
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& V = VV.back();
			Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& F = FF.back();
			Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& VC = VCV.back();

			P.resize(voxelCount(0, 0)*voxelCount(0, 1)*voxelCount(0, 2), 3);
			S.resize(voxelCount(0, 0)*voxelCount(0, 1)*voxelCount(0, 2), 1);
			S.setOnes();
			
#define VOX(x, y, z) ((z)*(voxelCount(0,1))*(voxelCount(0,0)) + (y)*(voxelCount(0,0)) + (x))
			for (int z = 0; z < voxelCount(0, 2); ++z)
			{
				for (int y = 0; y < voxelCount(0, 1); ++y)
				{
					for (int x = 0; x < voxelCount(0, 0); ++x)
					{
						const int voxelIndex = VOX(x, y, z);
						P.row(voxelIndex) << x, y, z;
					}
				}
			}
			P.rowwise() -= P.colwise().mean();

			const auto& xyziChunk = mainChunk.children.at(c + 1);
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> VoxelP(xyziChunk.content.at(0), 3);
			Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> VoxelC(xyziChunk.content.at(0), 4);
			for (int v = 0; v < xyziChunk.content.at(0); ++v)
			{
				const unsigned int z = ((xyziChunk.content.at(1 + v) & 0x000000ff) >> 0);
				const unsigned int x = ((xyziChunk.content.at(1 + v) & 0x0000ff00) >> 8);
				const unsigned int y = ((xyziChunk.content.at(1 + v) & 0x00ff0000) >> 16);
				const unsigned int i = ((xyziChunk.content.at(1 + v) & 0xff000000) >> 24);
				S(VOX(x + 1, y + 1, z + 1), 0) = -1;
				VoxelP.row(v) = P.row(VOX(x + 1, y + 1, z + 1));
				unsigned int color = palette.at(((i - 1) < 0 ? 0 : (i - 1)));
				const unsigned int r = ((color & 0x000000ff) >> 0);
				const unsigned int g = ((color & 0x0000ff00) >> 8);
				const unsigned int b = ((color & 0x00ff0000) >> 16);
				const unsigned int a = ((color & 0xff000000) >> 24);
				VoxelC.row(v) << r, g, b, a;
			}
			igl::copyleft::marching_cubes(S, P, voxelCount(0, 0), voxelCount(0, 1), voxelCount(0, 2), V, F);
			VC.resize(V.rows(), 4);

			for (int v = 0; v < V.rows(); v++) {
				int nearestVoxel;
				(VoxelP.rowwise() - V.row(v)).rowwise().squaredNorm().minCoeff(&nearestVoxel);
				VC.row(v) = VoxelC.row(nearestVoxel);
			}
		}
	}

	write_OBJ(ZBtextList.at(1), VV, FF, VCV);


	return 0.0f;
}
