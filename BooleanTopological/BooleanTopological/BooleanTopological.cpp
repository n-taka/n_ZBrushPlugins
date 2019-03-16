// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include "BooleanTopological.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <unordered_set>

#include "igl/copyleft/cgal/intersect_other.h"
#include "igl/adjacency_list.h"
#include "igl/triangle_triangle_adjacency.h"
#include "igl/vertex_triangle_adjacency.h"
#include "igl/signed_distance.h"
#include "igl/writeOBJ.h"

////
// implementation
////
#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif


extern "C" DLLEXPORT float version(char* someText, double optValue, char* outputBuffer, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	return 1.0f;
}

extern "C" DLLEXPORT float booleanTopological(char* someText, double optValue, char* outputBuffer, int optBuffer1Size, char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	//// input
	// someText: file name to be opened
	// [0]: dir name
	// [1]: comma-separated input filename(s) with flag
	// filename:O,filename:A,filename:S,filename:I
	// original, addition, subtraction, intersection ...
	// original mesh (base mesh that boolean will be applied) must come first.
	////

	////
	// [begin] decode parameters
	std::string ZBtext(someText);
	std::string separator(",");
	std::string flagSeparator(":");
	size_t separator_length = separator.length();
	size_t flagSeparator_length = flagSeparator.length();
	std::vector< std::pair<std::string, BOOLEAN_TYPE> > ZBtextList({});

	if (separator_length == 0) {
		ZBtextList.push_back({ ZBtext, ORIGINAL });
	}
	else {
		size_t offset = std::string::size_type(0);
		while (true) {
			size_t pos = ZBtext.find(separator, offset);
			if (pos == std::string::npos) {
				std::string filename_type = ZBtext.substr(offset);
				size_t pos1 = filename_type.find(flagSeparator);
				if (pos1 != std::string::npos) {
					BOOLEAN_TYPE b = ORIGINAL;
					if (filename_type.substr(pos1 + flagSeparator_length) == "A")
					{
						b = ADDITION;
					}
					else if (filename_type.substr(pos1 + flagSeparator_length) == "S")
					{
						b = SUBTRACTION;
					}
					else if (filename_type.substr(pos1 + flagSeparator_length) == "I")
					{
						b = INTERSECTION;
					}
					ZBtextList.push_back({ filename_type.substr(0, pos1), b });
				}
				else
				{
					BOOLEAN_TYPE b = NONE;
				}
				break;
			}
			std::string filename_type = ZBtext.substr(offset, pos - offset);
			size_t pos1 = filename_type.find(flagSeparator);
			if (pos1 != std::string::npos) {
				BOOLEAN_TYPE b = ORIGINAL;
				if (filename_type.substr(pos1 + flagSeparator_length) == "A")
				{
					b = ADDITION;
				}
				else if (filename_type.substr(pos1 + flagSeparator_length) == "S")
				{
					b = SUBTRACTION;
				}
				else if (filename_type.substr(pos1 + flagSeparator_length) == "I")
				{
					b = INTERSECTION;
				}
				ZBtextList.push_back({ filename_type.substr(0, pos1), b });
			}
			else
			{
				BOOLEAN_TYPE b = NONE;
				ZBtextList.push_back({ filename_type, b });
			}

			offset = pos + separator_length;
		}
	}
	std::string tmp;
	for (int i = 1; i < ZBtextList.size(); ++i)
	{
		tmp = ZBtextList.at(i).first;
		ZBtextList.at(i).first = ZBtextList.at(0).first + tmp;
	}

	std::ofstream logFile(ZBtextList.at(0).first + "log.txt");

	std::string baseFileName = ZBtextList.at(1).first;

	std::cout << "Base mesh: " << baseFileName << std::endl;
	logFile << "Base mesh: " << baseFileName << std::endl;
	// [end] parameter decoding end.
	////

	////
	// [begin] read triangle from file
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> V_A;
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> F_A;
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> VC_A;
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> FG_A;

	read_OBJ(baseFileName, V_A, F_A, VC_A, FG_A);
	// [end] read triangle from file
	////

	for (int m = 2; m < ZBtextList.size(); ++m)
	{
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> V_B;
		Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> F_B;
		Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> VC_B;
		Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> FG_B;

		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> V_C;
		Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> F_C;
		Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> VC_C;
		Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> FG_C;

		read_OBJ(ZBtextList.at(m).first, V_B, F_B, VC_B, FG_B);

		if (ZBtextList.at(m).second == ORIGINAL)
		{

		}
		else if (ZBtextList.at(m).second == ADDITION)
		{
			std::cout << "Addition, ";
			logFile << "Addition, ";
		}
		else if (ZBtextList.at(m).second == SUBTRACTION)
		{
			std::cout << "Subtraction, ";
			logFile << "Subtraction, ";
		}
		else if (ZBtextList.at(m).second == INTERSECTION)
		{
			std::cout << "Intersection, ";
			logFile << "Intersection, ";
		}
		std::cout << "mesh: " << ZBtextList.at(m).first << std::endl;
		logFile << "mesh: " << ZBtextList.at(m).first << std::endl;

		compute_boolean(
			V_A, F_A, VC_A, FG_A,
			V_B, F_B, VC_B, FG_B,
			V_C, F_C, VC_C, FG_C,
			ZBtextList.at(m).second);

		igl::writeOBJ("tmp.obj", V_C, F_C);
	}

	////
	// export with
	// jet color (update polypaint, keep polygroup)
	//Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> VC_Thicknessd;
	//Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic> VC_Thicknessi;
	////std::cout << V_SDF.transpose() << std::endl;
	////igl::jet(V_SDF, V_SDF.minCoeff(), V_SDF.maxCoeff(), VC_Thicknessd);
	//std::cout << V_SDF.minCoeff() << std::endl;
	//std::cout << V_SDF.maxCoeff() << std::endl;
	//VC_Thicknessi.resize(VC_Thicknessd.rows(), 4);
	//for (int v = 0; v < V.rows(); ++v)
	//{
	//	VC_Thicknessi(v, 0) = 255; // A (M?)
	//	VC_Thicknessi(v, 1) = int(VC_Thicknessd(v, 0) * 255); // R
	//	VC_Thicknessi(v, 2) = int(VC_Thicknessd(v, 1) * 255); // G
	//	VC_Thicknessi(v, 3) = int(VC_Thicknessd(v, 2) * 255); // B
	//}
	//write_OBJ(outputFileName, V, F, VC_Thicknessi, FG);
	////
	logFile.close();

	return 1.0f;
}

bool compute_boolean(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& VA,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FA,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& VCA,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FGA,
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& VB,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FB,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& VCB,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FGB,
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& VC,
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FC,
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& VCC,
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& FGC,
	BOOLEAN_TYPE& type
)
{
	////
	// HDS
	////
	std::vector< std::vector<int> > VA_adj;
	igl::adjacency_list(FA, VA_adj);
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> TTA, TTiA;
	igl::triangle_triangle_adjacency(FA, TTA, TTiA);
	std::vector< std::vector<int> > VFA;
	std::vector< std::vector<int> > VFiA;
	igl::vertex_triangle_adjacency(VA, FA, VFA, VFiA);

	std::vector< std::vector<int> > VB_adj;
	igl::adjacency_list(FB, VB_adj);
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> TTB, TTiB;
	igl::triangle_triangle_adjacency(FB, TTB, TTiB);
	std::vector< std::vector<int> > VFB;
	std::vector< std::vector<int> > VFiB;
	igl::vertex_triangle_adjacency(VB, FB, VFB, VFiB);

	////
	// detect polypaint markers
	////
	std::unordered_map<int, int> colorVote;
	for (int vIdx = 0; vIdx < VA.rows(); ++vIdx)
	{
		int encodedColor = (VCA(vIdx, 0) * 256 + VCA(vIdx, 1)) * 256 + VCA(vIdx, 2);
		if (colorVote.find(encodedColor) == colorVote.end())
		{
			colorVote[encodedColor] = 0;
		}
		colorVote[encodedColor] += 1;
	}
	int baseColor = -1;
	int maxVote = -1;
	for (const auto& c_v : colorVote)
	{
		if (c_v.second > maxVote)
		{
			baseColor = c_v.first;
			maxVote = c_v.second;
		}
	}
	std::unordered_set<int> toBeChecked;
	for (int vIdx = 0; vIdx < VA.rows(); ++vIdx)
	{
		int encodedColor = (VCA(vIdx, 0) * 256 + VCA(vIdx, 1)) * 256 + VCA(vIdx, 2);
		if (encodedColor != baseColor)
		{
			toBeChecked.insert(vIdx);
		}
	}
	std::vector<std::vector<int> > colorClusters;
	while (!toBeChecked.empty())
	{
		colorClusters.push_back({});
		int beginVIdx = *(toBeChecked.begin());
		toBeChecked.erase(beginVIdx);

		std::vector<int> stack({ beginVIdx });
		while (!stack.empty())
		{
			int top = stack.back();
			colorClusters.back().push_back(top);
			stack.pop_back();

			for (const int& nvIdx : VA_adj.at(top))
			{
				if (toBeChecked.find(nvIdx) != toBeChecked.end())
				{
					stack.push_back(nvIdx);
					toBeChecked.erase(nvIdx);
				}
			}
		}
	}
#if 1
	// for ignoring buggy user input (i.e. marker outside of the (VB,FB)), we carefully check the position of the color marker.
	std::vector<int> markerInV;
	for (int c = 0; c < colorClusters.size(); ++c)
	{
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> colorPoint;
		colorPoint.resize(colorClusters.at(c).size(), 3);
		for (int cvIdx = 0; cvIdx < colorClusters.at(c).size(); ++cvIdx)
		{
			colorPoint.row(cvIdx) = VA.row(colorClusters.at(c).at(cvIdx));
		}
		Eigen::Matrix<float, Eigen::Dynamic, 1> S;
		Eigen::Matrix<int, Eigen::Dynamic, 1> I;
		Eigen::Matrix<float, Eigen::Dynamic, 3> C;
		Eigen::Matrix<float, Eigen::Dynamic, 3> N;
		igl::signed_distance(colorPoint, VB, FB, igl::SIGNED_DISTANCE_TYPE_DEFAULT, S, I, C, N);

		int minIdx = -1;
		float minDist = S.minCoeff(&minIdx);

		if (minDist <= 0)
		{
			markerInV.push_back(colorClusters.at(c).at(minIdx));
		}
	}
#else
	std::vector<int> markerInV;
	for (int c = 0; c < colorClusters.size(); ++c)
	{
		std::cout << *(colorClusters.at(c).begin()) << std::endl;
		markerInV.push_back(*(colorClusters.at(c).begin()));
	}
#endif

	////
	// remesh based on intersection
	////
	// see https://github.com/libigl/libigl/blob/master/include/igl/copyleft/cgal/remesh_self_intersections.h
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> VD;
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> FD;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> IF;
	Eigen::Matrix<int, Eigen::Dynamic, 1> MapToOriginalF;
	Eigen::Matrix<int, Eigen::Dynamic, 1> MapToUniqueV;
	// note: bacause stitch_all is false (default value), VC.block(0, 0, VF.rows()+VB.rows(), 3) == [VA;VB]
	igl::copyleft::cgal::intersect_other(
		VA, FA, VB, FB, igl::copyleft::cgal::RemeshSelfIntersectionsParam(), IF, VD, FD, MapToOriginalF, MapToUniqueV
	);
	std::for_each(FD.data(), FD.data() + FD.size(), [&MapToUniqueV](int & a) {a = MapToUniqueV(a); });
	////
	// HDS
	////
	std::vector< std::vector<int> > VD_adj;
	igl::adjacency_list(FD, VD_adj);
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> TTD, TTiD;
	igl::triangle_triangle_adjacency(FD, TTD, TTiD);
	std::vector< std::vector<int> > VFD;
	std::vector< std::vector<int> > VFiD;
	igl::vertex_triangle_adjacency(VD, FD, VFD, VFiD);

	////
	// enumerates vertices shared with islands
	////
	std::vector<int> countV(VD.rows(), 0);
	for (int vIdx = 0; vIdx < VD.rows(); ++vIdx)
	{
		countV.at(MapToUniqueV(vIdx, 0)) += 1;
	}
	std::unordered_set<int> stitchV;
	for (int vIdx = 0; vIdx < countV.size(); ++vIdx)
	{
		if (countV.at(vIdx) >= 2)
		{
			stitchV.insert(vIdx);
		}
	}

	////
	// find shared vertices of interest
	////
	std::unordered_set<int> colorRegion(markerInV.begin(), markerInV.end());
	std::vector<int> stack = markerInV;
	while (!stack.empty())
	{
		int currentV = stack.back();
		stack.pop_back();
		for (const auto& nv : VD_adj.at(currentV))
		{
			if (colorRegion.find(nv) == colorRegion.end())
			{
				colorRegion.insert(nv);
				if (stitchV.find(nv) == stitchV.end())
				{
					stack.push_back(nv);
				}
			}
		}
	}
	std::unordered_set<int> stitchV_filtered;
	for (const auto& sv : stitchV)
	{
		if (colorRegion.find(sv) != colorRegion.end())
		{
			stitchV_filtered.insert(sv);
		}
	}
	Eigen::MatrixXf tmpV;
	Eigen::MatrixXi tmpF;
	tmpV.resize(stitchV_filtered.size(), 3);
	tmpF.resize(0, 3);
	int vvv = 0;
	for (const auto& sv : stitchV_filtered)
	{
		tmpV.row(vvv++) = VD.row(sv);
	}
	igl::writeOBJ("ppp.obj", tmpV, tmpF);
	igl::writeOBJ("remeshes.obj", VD, FD);


	////
	// enumerate divided F round the boudary of interest
	////
	std::unordered_set<int> boundaryFD; // in terms of edge-connectivity
	for (int fIdx = 0; fIdx < FD.rows(); ++fIdx)
	{
		int stitchVCount = 0;
		const int& uv0 = FD(fIdx, 0);
		const int& uv1 = FD(fIdx, 1);
		const int& uv2 = FD(fIdx, 2);
		if (stitchV_filtered.find(uv0) != stitchV_filtered.end())
		{
			stitchVCount++;
		}
		if (stitchV_filtered.find(uv1) != stitchV_filtered.end())
		{
			stitchVCount++;
		}
		if (stitchV_filtered.find(uv2) != stitchV_filtered.end())
		{
			stitchVCount++;
		}
		if (stitchVCount >= 2)
		{
			boundaryFD.insert(fIdx);
		}
	}

	std::vector<int> countF(FA.rows() + FB.rows(), 0);
	for (const auto& fdIdx : boundaryFD)
	{
		countF.at(MapToOriginalF(fdIdx, 0)) += 1;
	}
	std::unordered_set<int> dividedFAFB;
	for (int fIdx = 0; fIdx < countF.size(); ++fIdx)
	{
		if (countF.at(fIdx) >= 2)
		{
			dividedFAFB.insert(fIdx);
		}
	}

	// roll-back the remeshing
	// and stitch based on boolean type
	if (type == INTERSECTION)
	{
		// pick marked element
		// todo
	}
	else if (type == ADDITION)
	{
		// todo
	}
	else if (type == SUBTRACTION)
	{
		// todo
	}

	return true;
}