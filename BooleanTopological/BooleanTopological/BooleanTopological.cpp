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
	// detect polypaint markers
	////
	std::unordered_map<int, int> colorVote;
	for (int vIdx = 0; vIdx < VA.rows(); ++vIdx)
	{
		int encodedColor = (VCA(vIdx, 0) * 256 + VCA(vIdx, 1)) + VCA(vIdx, 2);
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

	std::vector< std::vector<int> > V_adj;
	igl::adjacency_list(FA, V_adj);
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> TT, TTi;
	igl::triangle_triangle_adjacency(FA, TT, TTi);

	std::unordered_set<int> toBeChecked;
	for (int vIdx = 0; vIdx < VA.rows(); ++vIdx)
	{
		int encodedColor = (VCA(vIdx, 0) * 256 + VCA(vIdx, 1)) + VCA(vIdx, 2);
		if (encodedColor != baseColor)
		{
			toBeChecked.insert(vIdx);
		}
	}

	std::vector<std::unordered_set<int> > colorClusters;
	while (!toBeChecked.empty())
	{
		colorClusters.push_back({});
		int beginVIdx = *(toBeChecked.begin());
		toBeChecked.erase(beginVIdx);

		std::vector<int> stack({ beginVIdx });
		while (!stack.empty())
		{
			int top = stack.back();
			colorClusters.back().insert(top);
			stack.pop_back();
			
			for (const int& nvIdx : V_adj.at(top))
			{
				if (toBeChecked.find(nvIdx) != toBeChecked.end())
				{
					stack.push_back(nvIdx);
					toBeChecked.erase(nvIdx);
				}
			}
		}
	}

	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> colorPoint;
	colorPoint.resize(colorClusters.size(), 3);
	for (int c = 0; c < colorClusters.size(); ++c)
	{
		colorPoint.row(c) = VA.row(*(colorClusters.at(c).begin()));
	}
	Eigen::Matrix<float, Eigen::Dynamic, 1> S;
	Eigen::Matrix<int, Eigen::Dynamic, 1> I;
	Eigen::Matrix<float, Eigen::Dynamic, 3> C;
	Eigen::Matrix<float, Eigen::Dynamic, 3> N;
	igl::signed_distance(colorPoint, VB, FB, igl::SIGNED_DISTANCE_TYPE_DEFAULT, S, I, C, N);

	std::vector<int> markerIn;
	std::vector<int> markerOut;
	for (int c = 0; c < colorClusters.size(); ++c)
	{
		if (S(c, 0) <= 0)
		{
			markerIn.push_back(*(colorClusters.at(c).begin()));
		}
		else
		{
			markerOut.push_back(*(colorClusters.at(c).begin()));
		}
	}

	for (const auto& in : markerIn)
	{
		std::cout << "in: " << in << std::endl;
	}
	for (const auto& out : markerOut)
	{
		std::cout << "out: " << out << std::endl;
	}

	////
	// remesh based on intersection
	////
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> IF;
	Eigen::Matrix<int, Eigen::Dynamic, 1> MapToOriginalF;
	Eigen::Matrix<int, Eigen::Dynamic, 1> MapToUniqueV;

	igl::copyleft::cgal::intersect_other(
		VA, FA, VB, FB, igl::copyleft::cgal::RemeshSelfIntersectionsParam(), IF, VC, FC, MapToOriginalF, MapToUniqueV
	);

	std::vector<int> count(MapToUniqueV.rows(), 0);
	for (int i = 0; i < count.size(); ++i)
	{
		count.at(MapToUniqueV(i, 0)) += 1;
	}
	//for (int i = 0; i < count.size(); ++i)
	//{
	//	if (count.at(i) >= 2) {
	//		std::cout << i << "-th vertex: " << count.at(i) << std::endl;
	//	}
	//}

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