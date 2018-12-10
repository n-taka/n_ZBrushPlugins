// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include <fstream>
#include "igl/signed_distance.h"
#include "MagicaVoxelizer.h"

// see https://github.com/ephtracy/voxel-model/blob/master/MagicaVoxel-file-format-vox.txt
////
// implementation
////
#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

extern "C" DLLEXPORT float magicaVoxelize(char* someText, double optValue, char* pOptBuffer1, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
    
	//// input
	// someText: file name to be opened
	// optValue: voxel resolution in Y coordinate
	////
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

#ifndef WIN32
    // if Mac, ZBrush gives me invalid prefix with FileNameResolvePath ...
    for(auto& s : ZBtextList)
    {
        s.erase(s.begin(), s.begin()+2);
    }
#endif

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V;
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> VC;
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> F;
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> FC;
	Eigen::Matrix<double, Eigen::Dynamic, 1> S;
	Eigen::Matrix<   int, Eigen::Dynamic, 1> I;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> N;

	// read triangle from file
	read_OBJ(ZBtextList.at(0), V, F, VC);
	FC.resize(F.rows(), 4);
	// convert vertex color to Face color

	for (int f = 0; f < FC.rows(); f++)
	{
		// simply compute average
		FC.row(f) = (VC.row(F(f, 0)) + VC.row(F(f, 1)) + VC.row(F(f, 2))) / 3;
	}

    // generate voxel for signed distance computation (query point is center of each voxel)
    Eigen::Matrix<double, 1, 3> BBSize = V.colwise().maxCoeff() - V.colwise().minCoeff();
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
	MagicaVoxelChunk mainChunk = MagicaVoxelChunk("MAIN");
	MagicaVoxelChunk packChunk = MagicaVoxelChunk("PACK");
	MagicaVoxelChunk sizeChunk = MagicaVoxelChunk("SIZE");
	MagicaVoxelChunk xyziChunk = MagicaVoxelChunk("XYZI");

	// currently we assume that there is only 1 model.
	packChunk.content.at(0) = 1;

	// xyz (ZBrush) --> yzx(MagicaVoxel)
	sizeChunk.content.at(0) = voxelCount(0, 2);
	sizeChunk.content.at(1) = voxelCount(0, 0);
	sizeChunk.content.at(2) = voxelCount(0, 1);

	int voxelNum = (S.array() <= 0).count();
	xyziChunk.content.resize(1 + voxelNum);
	xyziChunk.content.at(0) = voxelNum;
	int voxelIdx = 0;
	for (int z = 0; z < voxelCount(0, 2); ++z)
	{
		for (int y = 0; y < voxelCount(0, 1); ++y)
		{
			for (int x = 0; x < voxelCount(0, 0); ++x)
			{
				const int voxelIndex = VOX(x, y, z);
				if (S(voxelIndex, 0) <= 0)
				{
					union converter {
						unsigned char c[4];
						int i;
					};
					union converter conv;
					conv.c[0] = static_cast<unsigned char>(z);
					conv.c[1] = static_cast<unsigned char>(x);
					conv.c[2] = static_cast<unsigned char>(y);
					// pickup from first 216 colors (becaus of implementation cost...)
					// 255, 204, 153, 102, 51, 0
					int rIdx = std::round(double(FC(I(voxelIndex, 0), 1)) / 51.0);
					int gIdx = std::round(double(FC(I(voxelIndex, 0), 2)) / 51.0);
					int bIdx = std::round(double(FC(I(voxelIndex, 0), 3)) / 51.0);
					if (rIdx == 0 && gIdx == 0 && bIdx == 0)
					{
						conv.c[3] = 254;
					}
					else
					{
						conv.c[3] = 216 - (bIdx + gIdx * 6 + rIdx * 6 * 6);
					}
					xyziChunk.content.at(1 + voxelIdx) = conv.i;
					voxelIdx++;
				}
			}
		}
	}

	// write to file
	mainChunk.appendChild(packChunk);
	mainChunk.appendChild(sizeChunk);
	mainChunk.appendChild(xyziChunk);
	mainChunk.writeVoxelToFile(ZBtextList.at(1));

	return 0.0f;
}
