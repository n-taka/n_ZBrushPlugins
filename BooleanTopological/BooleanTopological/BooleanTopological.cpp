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
#include "igl/copyleft/cgal/remesh_self_intersections.h"
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
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> VAB;
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> FAB;
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> VCAB;
	Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic> FGAB;
	{
		VAB.resize(VA.rows() + VB.rows(), 3);
		VAB.block(0, 0, VA.rows(), 3) = VA;
		VAB.block(VA.rows(), 0, VB.rows(), 3) = VB;

		FAB.resize(FA.rows() + FB.rows(), 3);
		FAB.block(0, 0, FA.rows(), 3) = FA;
		FAB.block(FA.rows(), 0, FB.rows(), 3) = FB.array() + VA.rows();

		VCAB.resize(VCA.rows() + VCB.rows(), 4);
		VCAB.block(0, 0, VCA.rows(), 4) = VCA;
		VCAB.block(VCA.rows(), 0, VCB.rows(), 4) = VCB;

		FGAB.resize(FGA.rows() + FGB.rows(), 1);
		FGAB.block(0, 0, FGA.rows(), 1) = FGA;
		FGAB.block(FGA.rows(), 0, FGB.rows(), 1) = FGB;
	}

	igl::writeOBJ("merged.obj", VAB, FAB);

	////
	// HDS
	////
	std::vector< std::vector<int> > VAB_adj;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> TTAB, TTiAB;
	std::vector< std::vector<int> > VFAB, VFiAB;
	{
		igl::adjacency_list(FAB, VAB_adj);
		igl::triangle_triangle_adjacency(FAB, TTAB, TTiAB);
		igl::vertex_triangle_adjacency(VAB, FAB, VFAB, VFiAB);
	}

	////
	// detect polypaint markers
	////
	std::vector<std::vector<int> > colorClusters;
	{
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

				for (const int& nvIdx : VAB_adj.at(top))
				{
					if (toBeChecked.find(nvIdx) != toBeChecked.end())
					{
						stack.push_back(nvIdx);
						toBeChecked.erase(nvIdx);
					}
				}
			}
		}
	}
#if 1
	// for ignoring buggy user input (i.e. marker outside of the (VB,FB)), we carefully check the position of the color marker.
	std::vector<int> markerInV;
	{
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
	}
#else
	std::vector<int> markerInV;
	{
		for (int c = 0; c < colorClusters.size(); ++c)
		{
			std::cout << *(colorClusters.at(c).begin()) << std::endl;
			markerInV.push_back(*(colorClusters.at(c).begin()));
		}
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
	{
		igl::copyleft::cgal::remesh_self_intersections(
			VAB, FAB, igl::copyleft::cgal::RemeshSelfIntersectionsParam(), VD, FD, IF, MapToOriginalF, MapToUniqueV
		);
	}

	////
	// HDS
	////
	std::vector< std::vector<int> > VD_adj;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> TTD, TTiD;
	std::vector< std::vector<int> > VFD, VFiD;
	{
		igl::adjacency_list(FD, VD_adj);
		igl::triangle_triangle_adjacency(FD, TTD, TTiD);
		igl::vertex_triangle_adjacency(VD, FD, VFD, VFiD);

		std::for_each(FD.data(), FD.data() + FD.size(), [&MapToUniqueV](int & a) {a = MapToUniqueV(a); });
	}

	////
	// enumerates vertices shared with islands
	////
	std::unordered_set<int> stitchV;
	{
		std::vector<int> countV(VD.rows(), 0);
		for (int vIdx = 0; vIdx < VD.rows(); ++vIdx)
		{
			countV.at(MapToUniqueV(vIdx, 0)) += 1;
		}
		for (int vIdx = 0; vIdx < countV.size(); ++vIdx)
		{
			if (countV.at(vIdx) >= 2)
			{
				stitchV.insert(vIdx);
			}
		}
	}

	////
	// filter fake boudary edge whose endpoints are both stitchV (if mesh resolution is too low, this might happen)
	////
	std::unordered_map< int, std::unordered_map<int, std::pair<int, int> > > edgeRefCount;
	for (int fIdx = 0; fIdx < FD.rows(); ++fIdx)
	{
		for (int e = 0; e < 3; ++e)
		{
			const int& ve = FD(fIdx, e);
			const int& vep = FD(fIdx, (e + 1) % 3);
			if (stitchV.find(ve) != stitchV.end() && stitchV.find(vep) != stitchV.end())
			{
				if (MapToOriginalF(fIdx, 0) < FA.rows())
				{
					edgeRefCount[std::min(ve, vep)][std::max(ve, vep)].first += 1;
				}
				else
				{
					edgeRefCount[std::min(ve, vep)][std::max(ve, vep)].second += 1;
				}
			}
		}
	}

	////
	// enumerate patches whose borders are stitchV
	////
	std::vector< std::unordered_set<int> > patchA_inB, patchA_outB, patchB_inA, patchB_outA;
	{
		std::unordered_set<int> toBeCheckedF;
		for (int fIdx = 0; fIdx < FD.rows(); ++fIdx)
		{
			toBeCheckedF.insert(fIdx);
		}
		while (!toBeCheckedF.empty())
		{
			std::unordered_set<int> currentPatchF;
			const int searchBeginF = *toBeCheckedF.begin();
			std::vector<int> stack({ searchBeginF });
			toBeCheckedF.erase(searchBeginF);
			while (!stack.empty())
			{
				const int currentFIdx = stack.back();
				currentPatchF.insert(currentFIdx);
				stack.pop_back();
				for (int e = 0; e < 3; ++e)
				{
					int ve = FD(currentFIdx, e);
					int vep = FD(currentFIdx, (e + 1) % 3);
					if (((edgeRefCount[std::min(ve, vep)][std::max(ve, vep)].first < 2) || (edgeRefCount[std::min(ve, vep)][std::max(ve, vep)].second < 2))
						&& (currentPatchF.find(TTD(currentFIdx, e)) == currentPatchF.end()))
					{
						// if this edge is not the stitch boundary
						stack.push_back(TTD(currentFIdx, e));
						toBeCheckedF.erase(TTD(currentFIdx, e));
					}
				}
			}

			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> queryPoint;
			queryPoint.resize(1, 3);
			queryPoint.row(0) = (VD.row(FD(searchBeginF, 0)) + VD.row(FD(searchBeginF, 1)) + VD.row(FD(searchBeginF, 2))) / 3.0;
			Eigen::Matrix<float, Eigen::Dynamic, 1> S;
			Eigen::Matrix<int, Eigen::Dynamic, 1> I;
			Eigen::Matrix<float, Eigen::Dynamic, 3> C;
			Eigen::Matrix<float, Eigen::Dynamic, 3> N;
			if (MapToOriginalF(searchBeginF, 0) < FA.rows())
			{
				// this patch belongs to (VA,FA)
				// check this patch is in/out of (VB,FB)
				igl::signed_distance(queryPoint, VB, FB, igl::SIGNED_DISTANCE_TYPE_DEFAULT, S, I, C, N);
				if (S(0, 0) <= 0)
				{
					// inside
					patchA_inB.push_back(currentPatchF);
				}
				else
				{
					// outside
					patchA_outB.push_back(currentPatchF);
				}
			}
			else if (MapToOriginalF(searchBeginF, 0) < FA.rows() + FB.rows())
			{
				// this patch belongs to (VB,FB)
				// check this patch is in/out of (VA,FA)
				igl::signed_distance(queryPoint, VA, FA, igl::SIGNED_DISTANCE_TYPE_DEFAULT, S, I, C, N);
				if (S(0, 0) <= 0)
				{
					// inside
					patchB_inA.push_back(currentPatchF);
				}
				else
				{
					// outside
					patchB_outA.push_back(currentPatchF);
				}
			}
		}
	}
	std::unordered_set<int> vvvS;
	for (const auto& v0 : edgeRefCount)
	{
		for (const auto& v1 : v0.second)
		{
			if (v1.second.first >= 2 && v1.second.second >= 2)
			{
				vvvS.insert(v0.first);
				vvvS.insert(v1.first);
			}
		}
	}
	{
		Eigen::MatrixXf tmpV;
		Eigen::MatrixXi tmpF;
		for (int i = 0; i < vvvS.size(); ++i)
		{
			tmpV.resize(vvvS.size(), 3);
			tmpF.resize(0, 3);
			int vvv = 0;
			for (const auto& sv : vvvS)
			{
				tmpV.row(vvv++) = VD.row(sv);
			}
		}
		igl::writeOBJ("ridge.obj", tmpV, tmpF);
	}
	{
		Eigen::MatrixXf tmpV;
		Eigen::MatrixXi tmpF;
		for (int i = 0; i < stitchV.size(); ++i)
		{
			tmpV.resize(stitchV.size(), 3);
			tmpF.resize(0, 3);
			int vvv = 0;
			for (const auto& sv : stitchV)
			{
				tmpV.row(vvv++) = VD.row(sv);
			}
		}
		igl::writeOBJ("stitch.obj", tmpV, tmpF);
	}

	for (int i = 0; i < patchA_inB.size(); ++i)
	{
		Eigen::MatrixXf tmpV;
		Eigen::MatrixXi tmpF;
		tmpV.resize(patchA_inB.at(i).size() * 3, 3);
		tmpF.resize(0, 3);
		int vvv = 0;
		for (const auto& sv : patchA_inB.at(i))
		{
			tmpV.row(vvv++) = VD.row(FD(sv, 0));
			tmpV.row(vvv++) = VD.row(FD(sv, 1));
			tmpV.row(vvv++) = VD.row(FD(sv, 2));
		}
		char buf[100];
		sprintf(buf, "patchA_inB_%d.obj", i);
		igl::writeOBJ(buf, tmpV, tmpF);
	}
	for (int i = 0; i < patchA_outB.size(); ++i)
	{
		Eigen::MatrixXf tmpV;
		Eigen::MatrixXi tmpF;
		tmpV.resize(patchA_outB.at(i).size() * 3, 3);
		tmpF.resize(0, 3);
		int vvv = 0;
		for (const auto& sv : patchA_outB.at(i))
		{
			tmpV.row(vvv++) = VD.row(FD(sv, 0));
			tmpV.row(vvv++) = VD.row(FD(sv, 1));
			tmpV.row(vvv++) = VD.row(FD(sv, 2));
		}
		char buf[100];
		sprintf(buf, "patchA_outB_%d.obj", i);
		igl::writeOBJ(buf, tmpV, tmpF);
	}
	for (int i = 0; i < patchB_inA.size(); ++i)
	{
		Eigen::MatrixXf tmpV;
		Eigen::MatrixXi tmpF;
		tmpV.resize(patchB_inA.at(i).size() * 3, 3);
		tmpF.resize(0, 3);
		int vvv = 0;
		for (const auto& sv : patchB_inA.at(i))
		{
			tmpV.row(vvv++) = VD.row(FD(sv, 0));
			tmpV.row(vvv++) = VD.row(FD(sv, 1));
			tmpV.row(vvv++) = VD.row(FD(sv, 2));
		}
		char buf[100];
		sprintf(buf, "patchB_inA_%d.obj", i);
		igl::writeOBJ(buf, tmpV, tmpF);
	}
	for (int i = 0; i < patchB_outA.size(); ++i)
	{
		Eigen::MatrixXf tmpV;
		Eigen::MatrixXi tmpF;
		tmpV.resize(patchB_outA.at(i).size() * 3, 3);
		tmpF.resize(0, 3);
		int vvv = 0;
		for (const auto& sv : patchB_outA.at(i))
		{
			tmpV.row(vvv++) = VD.row(FD(sv, 0));
			tmpV.row(vvv++) = VD.row(FD(sv, 1));
			tmpV.row(vvv++) = VD.row(FD(sv, 2));
		}
		char buf[100];
		sprintf(buf, "patchB_outA_%d.obj", i);
		igl::writeOBJ(buf, tmpV, tmpF);
	}


	return true;

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