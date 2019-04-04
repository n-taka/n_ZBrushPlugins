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
#include "igl/remove_unreferenced.h"
#include "igl/is_vertex_manifold.h"
#include "igl/writeOBJ.h"

////
// implementation
////
#if defined(_WIN32) || defined(_WIN64)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __attribute__((visibility("default")))
#endif

extern "C" DLLEXPORT float version(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData)
{
	return 1.0f;
}

extern "C" DLLEXPORT float booleanTopological(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData)
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
	std::vector<std::pair<std::string, BOOLEAN_TYPE>> ZBtextList({});

	if (separator_length == 0)
	{
		ZBtextList.push_back({ZBtext, ORIGINAL});
	}
	else
	{
		size_t offset = std::string::size_type(0);
		while (true)
		{
			size_t pos = ZBtext.find(separator, offset);
			if (pos == std::string::npos)
			{
				std::string filename_type = ZBtext.substr(offset);
				size_t pos1 = filename_type.find(flagSeparator);
				if (pos1 != std::string::npos)
				{
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
					ZBtextList.push_back({filename_type.substr(0, pos1), b});
				}
				else
				{
					BOOLEAN_TYPE b = NONE;
				}
				break;
			}
			std::string filename_type = ZBtext.substr(offset, pos - offset);
			size_t pos1 = filename_type.find(flagSeparator);
			if (pos1 != std::string::npos)
			{
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
				ZBtextList.push_back({filename_type.substr(0, pos1), b});
			}
			else
			{
				BOOLEAN_TYPE b = NONE;
				ZBtextList.push_back({filename_type, b});
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
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F_A;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> VC_A;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FG_A;

	read_OBJ(baseFileName, V_A, F_A, VC_A, FG_A);
	// [end] read triangle from file
	////

	for (int m = 2; m < ZBtextList.size(); ++m)
	{
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> V_B;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F_B;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> VC_B;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FG_B;

		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> V_C;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F_C;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> VC_C;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FG_C;

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

		igl::writeOBJ("boolean_mesh.obj", V_C, F_C);
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
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &VA,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FA,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &VCA,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FGA,
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &VB,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FB,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &VCB,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FGB,
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &VC,
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FC,
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &VCC,
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &FGC,
	BOOLEAN_TYPE &type)
{
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> VAB;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FAB;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> VCAB;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FGAB;
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
	std::vector<std::vector<int>> VAB_adj;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> TTAB, TTiAB;
	std::vector<std::vector<int>> VFAB, VFiAB;
	{
		igl::adjacency_list(FAB, VAB_adj);
		igl::triangle_triangle_adjacency(FAB, TTAB, TTiAB);
		igl::vertex_triangle_adjacency(VAB, FAB, VFAB, VFiAB);
	}

	////
	// detect polypaint markers
	////
	std::vector<std::vector<int>> colorClusters;
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
		for (const auto &c_v : colorVote)
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

			std::vector<int> stack({beginVIdx});
			while (!stack.empty())
			{
				int top = stack.back();
				colorClusters.back().push_back(top);
				stack.pop_back();

				for (const int &nvIdx : VAB_adj.at(top))
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
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FD, FD_stitchAll;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> IF;
	Eigen::Matrix<int, Eigen::Dynamic, 1> MapToOriginalF;
	Eigen::Matrix<int, Eigen::Dynamic, 1> MapToUniqueV;
	// note: bacause stitch_all is false (default value), VD.block(0, 0, VF.rows()+VB.rows(), 3) == [VA;VB]
	{
		igl::copyleft::cgal::remesh_self_intersections(
			VAB, FAB, igl::copyleft::cgal::RemeshSelfIntersectionsParam(), VD, FD, IF, MapToOriginalF, MapToUniqueV);
	}
	igl::writeOBJ("remeshed.obj", VD, FD);

	////
	// HDS
	////
	std::vector<std::vector<int>> VD_adj;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> TTD, TTiD;
	std::vector<std::vector<int>> VFD, VFiD;
	{
		igl::adjacency_list(FD, VD_adj);
		igl::triangle_triangle_adjacency(FD, TTD, TTiD);
		igl::vertex_triangle_adjacency(VD, FD, VFD, VFiD);

		FD_stitchAll = FD;
		std::for_each(FD_stitchAll.data(), FD_stitchAll.data() + FD_stitchAll.size(), [&MapToUniqueV](int &a) { a = MapToUniqueV(a); });
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
	std::unordered_map<int, std::unordered_map<int, std::pair<int, int>>> edgeRefCount;
	for (int fIdx = 0; fIdx < FD_stitchAll.rows(); ++fIdx)
	{
		for (int e = 0; e < 3; ++e)
		{
			const int &ve = FD_stitchAll(fIdx, e);
			const int &vep = FD_stitchAll(fIdx, (e + 1) % 3);
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
	// enumerate patches whose borders are stitchV that shared between (VA, FA) and (VB, FB) (ignore self-intersections)
	////
	std::vector<std::unordered_set<int>> patchFA_inB, patchFA_outB, patchFB_inA, patchFB_outA;
	std::vector<std::unordered_set<int>> patchVA_inB, patchVA_outB, patchVB_inA, patchVB_outA;
	{
		std::unordered_set<int> toBeCheckedF;
		for (int fIdx = 0; fIdx < FD_stitchAll.rows(); ++fIdx)
		{
			toBeCheckedF.insert(fIdx);
		}
		while (!toBeCheckedF.empty())
		{
			std::unordered_set<int> currentPatchF;
			std::unordered_set<int> currentPatchV;
			const int searchBeginF = *toBeCheckedF.begin();
			std::vector<int> stack({searchBeginF});
			toBeCheckedF.erase(searchBeginF);
			while (!stack.empty())
			{
				const int currentFIdx = stack.back();
				currentPatchF.insert(currentFIdx);
				for (int vIdx = 0; vIdx < 3; ++vIdx)
				{
					currentPatchV.insert(FD_stitchAll(currentFIdx, vIdx));
				}
				stack.pop_back();
				for (int e = 0; e < 3; ++e)
				{
					int ve = FD_stitchAll(currentFIdx, e);
					int vep = FD_stitchAll(currentFIdx, (e + 1) % 3);
					if (((edgeRefCount[std::min(ve, vep)][std::max(ve, vep)].first < 2) || (edgeRefCount[std::min(ve, vep)][std::max(ve, vep)].second < 2)) && (currentPatchF.find(TTD(currentFIdx, e)) == currentPatchF.end()))
					{
						// if this edge is not the stitch boundary
						stack.push_back(TTD(currentFIdx, e));
						toBeCheckedF.erase(TTD(currentFIdx, e));
					}
				}
			}

			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> queryPoint;
			queryPoint.resize(1, 3);
			queryPoint.row(0) = (VD.row(FD_stitchAll(searchBeginF, 0)) + VD.row(FD_stitchAll(searchBeginF, 1)) + VD.row(FD_stitchAll(searchBeginF, 2))) / 3.0;
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
					patchFA_inB.push_back(currentPatchF);
					patchVA_inB.push_back(currentPatchV);
				}
				else
				{
					// outside
					patchFA_outB.push_back(currentPatchF);
					patchVA_outB.push_back(currentPatchV);
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
					patchFB_inA.push_back(currentPatchF);
					patchVB_inA.push_back(currentPatchV);
				}
				else
				{
					// outside
					patchFB_outA.push_back(currentPatchF);
					patchVB_outA.push_back(currentPatchV);
				}
			}
		}
	}
#if 0
	for (int i = 0; i < patchVA_inB.size(); ++i)
	{
		Eigen::MatrixXf tmpV;
		Eigen::MatrixXi tmpF;
		tmpV.resize(patchVA_inB.at(i).size(), 3);
		tmpF.resize(0, 3);
		int vvv = 0;
		for (const auto& sv : patchVA_inB.at(i))
		{
			tmpV.row(vvv++) = VD.row(sv);
		}
		char buf[100];
		sprintf(buf, "patchA_inB_%d.obj", i);
		igl::writeOBJ(buf, tmpV, tmpF);
	}
	for (int i = 0; i < patchFB_inA.size(); ++i)
	{
		Eigen::MatrixXf tmpV;
		Eigen::MatrixXi tmpF;
		tmpV.resize(patchFB_inA.at(i).size(), 3);
		tmpF.resize(0, 3);
		int vvv = 0;
		for (const auto& sv : patchFB_inA.at(i))
		{
			tmpV.row(vvv++) = (VD.row(FD(sv, 0)) + VD.row(FD(sv, 1)) + VD.row(FD(sv, 2))) / 3.0;
		}
		char buf[100];
		sprintf(buf, "patchB_inA_%d.obj", i);
		igl::writeOBJ(buf, tmpV, tmpF);
	}

	for (const auto& v : patchFB_inA.at(9))
	{
		for (int lv = 0; lv < 3; lv++) {
			std::cout << FD(v, lv) << " ";
		}
	}
	std::cout << std::endl << std::endl;
	for (const auto& v : patchFB_inA.at(10))
	{
		for (int lv = 0; lv < 3; lv++) {
			std::cout << FD(v, lv) << " ";
		}
	}
	std::cout << std::endl << std::endl;
#endif

	////
	// find patches of interest
	////
	auto getPatchIdA = [&](const int &v0, const int v1) {
		// return index of the patch of A that have v1 -> v0 as its halfedge
		for (int pIdx = 0; pIdx < patchVA_inB.size(); ++pIdx)
		{
			if ((patchVA_inB.at(pIdx).find(v0) != patchVA_inB.at(pIdx).end()) && (patchVA_inB.at(pIdx).find(v1) != patchVA_inB.at(pIdx).end()))
			{
				for (const auto &fIdx : patchFA_inB.at(pIdx))
				{
					for (int e = 0; e < 3; ++e)
					{
						const int &ev0 = FD_stitchAll(fIdx, e);
						const int &ev1 = FD_stitchAll(fIdx, (e + 1) % 3);
						if ((ev0 == v1) && (ev1 == v0))
						{
							return pIdx;
						}
					}
				}
			}
		}
		return -1;
	};
	auto getPatchIdB = [&](const int &v0, const int v1) {
		// return index of the patch of B that have v1 -> v0 as its halfedge
		for (int pIdx = 0; pIdx < patchVB_inA.size(); ++pIdx)
		{
			if ((patchVB_inA.at(pIdx).find(v0) != patchVB_inA.at(pIdx).end()) && (patchVB_inA.at(pIdx).find(v1) != patchVB_inA.at(pIdx).end()))
			{
				for (const auto &fIdx : patchFB_inA.at(pIdx))
				{
					for (int e = 0; e < 3; ++e)
					{
						const int &ev0 = FD_stitchAll(fIdx, e);
						const int &ev1 = FD_stitchAll(fIdx, (e + 1) % 3);
						if ((ev0 == v1) && (ev1 == v0))
						{
							return pIdx;
						}
					}
				}
			}
		}
		return -1;
	};

	std::unordered_set<int> patchA_inB_ofInterest, patchB_inA_ofInterest;
	std::vector<int> stackA, stackB;
	for (const auto &cv : markerInV)
	{
		for (int pIdx = 0; pIdx < patchVA_inB.size(); ++pIdx)
		{
			if (patchVA_inB.at(pIdx).find(cv) != patchVA_inB.at(pIdx).end())
			{
				patchA_inB_ofInterest.insert(pIdx);
				stackA.push_back(pIdx);
				break;
			}
		}
	}

	while (!stackA.empty() || stackB.empty())
	{
		if (!stackA.empty())
		{
			const int currentPatch = stackA.back();
			stackA.pop_back();
			for (const auto &fIdx : patchFA_inB.at(currentPatch))
			{
				for (int e = 0; e < 3; ++e)
				{
					const int &ev0 = FD_stitchAll(fIdx, e);
					const int &ev1 = FD_stitchAll(fIdx, (e + 1) % 3);
					if ((stitchV.find(ev0) != stitchV.end()) && (stitchV.find(ev1) != stitchV.end()) && (edgeRefCount[std::min(ev0, ev1)][std::max(ev0, ev1)].first >= 2) && (edgeRefCount[std::min(ev0, ev1)][std::max(ev0, ev1)].second >= 2))
					{
						const int patchBAround = getPatchIdB(ev0, ev1);
						if (patchBAround >= 0)
						{
							if (patchB_inA_ofInterest.find(patchBAround) == patchB_inA_ofInterest.end())
							{
								patchB_inA_ofInterest.insert(patchBAround);
								stackB.push_back(patchBAround);
							}
						}
						else
						{
							const int patchAAround = getPatchIdA(ev0, ev1);
							if (patchAAround >= 0)
							{
								if (patchA_inB_ofInterest.find(patchAAround) == patchA_inB_ofInterest.end())
								{
									patchA_inB_ofInterest.insert(patchAAround);
									stackA.push_back(patchAAround);
								}
							}
						}
					}
				}
			}
		}
		else
		{
			// pick from stackB
			const int currentPatch = stackB.back();
			stackB.pop_back();
			for (const auto &fIdx : patchFB_inA.at(currentPatch))
			{
				for (int e = 0; e < 3; ++e)
				{
					const int &ev0 = FD_stitchAll(fIdx, e);
					const int &ev1 = FD_stitchAll(fIdx, (e + 1) % 3);
					if ((stitchV.find(ev0) != stitchV.end()) && (stitchV.find(ev1) != stitchV.end()) && (edgeRefCount[std::min(ev0, ev1)][std::max(ev0, ev1)].first >= 2) && (edgeRefCount[std::min(ev0, ev1)][std::max(ev0, ev1)].second >= 2))
					{
						const int patchAAround = getPatchIdA(ev0, ev1);
						if (patchAAround >= 0)
						{
							if (patchA_inB_ofInterest.find(patchAAround) == patchA_inB_ofInterest.end())
							{
								patchA_inB_ofInterest.insert(patchAAround);
								stackA.push_back(patchAAround);
							}
						}
						else
						{
							const int patchBAround = getPatchIdB(ev0, ev1);
							if (patchBAround >= 0)
							{
								if (patchB_inA_ofInterest.find(patchBAround) == patchB_inA_ofInterest.end())
								{
									patchB_inA_ofInterest.insert(patchBAround);
									stackB.push_back(patchBAround);
								}
							}
						}
					}
				}
			}
		}
	}

#if 0
	for (const auto& pa : patchA_inB_ofInterest)
	{
		std::cout << "A in B: " << pa << std::endl;
	}
	for (const auto& pb : patchB_inA_ofInterest)
	{
		std::cout << "B in A: " << pb << std::endl;
	}
#endif
	std::vector<bool> needStitch(VD.rows(), false);
	// for (const auto &pIdx : patchA_inB_ofInterest)
	// {
	// 	for (const auto &f : patchFA_inB.at(pIdx))
	// 	{
	// 		for (int lv = 0; lv < 3; ++lv)
	// 		{
	// 			needStitch.at(FD(f, lv)) = true;
	// 		}
	// 	}
	// }
	// for (const auto &pIdx : patchB_inA_ofInterest)
	// {
	// 	for (const auto &f : patchFB_inA.at(pIdx))
	// 	{
	// 		for (int lv = 0; lv < 3; ++lv)
	// 		{
	// 			needStitch.at(FD(f, lv)) = true;
	// 		}
	// 	}
	// }

	std::unordered_map<int, std::vector<int>> stitchFrom;
	for (int vIdx = 0; vIdx < VD.rows(); ++vIdx)
	{
		stitchFrom[MapToUniqueV(vIdx)].push_back(vIdx);
	}
	////
	// update MapToUniqueV for avoiding accidental stitch (when the mesh is self-intersecting)
	////
	// 20190404: if vertices (more than four vertices) are merged into single vertex, we need to split them into two or so...
	// some bug with intersection...
	// for (int vIdx = 0; vIdx < needStitch.size(); ++vIdx)
	// {
	// 	if (needStitch.at(vIdx))
	// 	{
	// 		if (!needStitch.at(MapToUniqueV(vIdx, 0)))
	// 		{
	// 			for (const auto &from : stitchFrom.at(MapToUniqueV(vIdx, 0)))
	// 			{
	// 				if (needStitch.at(from))
	// 				{
	// 					MapToUniqueV(vIdx, 0) = from;
	// 					break;
	// 				}
	// 			}
	// 		}
	// 	}
	// }
	for (const auto &stitchInfo : stitchFrom)
	{
		const int &stitch_into = stitchInfo.first;
		const std::vector<int> &stitch_from = stitchInfo.second;
		if (stitch_from.size() >= 2)
		{
			// 1. find a pair of (a vertex belongs to (VA,FA), a vertex belongs to (VB,FB))
			std::vector<int> vertA, vertB;
			for (const auto &targetV : stitch_from)
			{
				int group = -1; // 0: A, 1: B
				for (const auto &nf : VFD.at(targetV))
				{
					for (int fv = 0; fv < 3; ++fv)
					{
						if (FD(nf, fv) < VA.rows())
						{
							group = 0;
							break;
						}
						else if (FD(nf, fv) < VA.rows() + VB.rows())
						{
							group = 1;
							break;
						}
					}
					if (group > -1)
					{
						break;
					}
				}
				if (group == 0)
				{
					vertA.push_back(targetV);
				}
				else if (group == 1)
				{
					vertB.push_back(targetV);
				}
			}
			if (vertA.size() == vertB.size())
			{
				for (int vi = 0; vi < vertA.size(); ++vi)
				{
					// stitch vertA.at(i) and vertB.at(i)
					// currently, we stitch the vertices into vertA(i)
					MapToUniqueV(vertA.at(vi), 0) = vertA.at(vi);
					MapToUniqueV(vertB.at(vi), 0) = vertA.at(vi);
					needStitch.at(vertA.at(vi)) = true;
					needStitch.at(vertB.at(vi)) = true;
				}
			}
		}
	}

	// 20190404: maybe need to check we only stitch "a vertex of patchA" and "a vertex of patchB"

	//
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> partialStitch;
	partialStitch.resize(VD.rows(), 1);
	for (int vIdx = 0; vIdx < partialStitch.rows(); ++vIdx)
	{
		partialStitch(vIdx, 0) = (needStitch.at(vIdx)) ? MapToUniqueV(vIdx, 0) : vIdx;
	}
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FD_stitchPartial = FD;
	std::for_each(FD_stitchPartial.data(), FD_stitchPartial.data() + FD_stitchPartial.size(), [&partialStitch](int &a) { a = partialStitch(a); });

	// stitch based on boolean type
	if (type == INTERSECTION || type == ADDITION)
	{
		// remove all patches *EXCEPT* patches of interest (INTERSECTION)
		// remove patches of interest (ADDITION)
		std::vector<bool> ignored(FD_stitchPartial.rows(), (type == INTERSECTION));
		int fCount = (type == ADDITION) ? (FD_stitchPartial.rows()) : (0);
		for (const auto &pIdx : patchA_inB_ofInterest)
		{
			for (const auto &fa : patchFA_inB.at(pIdx))
			{
				ignored.at(fa) = (type == ADDITION);
				fCount += ((type == ADDITION) ? -1 : 1);
			}
		}
		for (const auto &pIdx : patchB_inA_ofInterest)
		{
			for (const auto &fb : patchFB_inA.at(pIdx))
			{
				ignored.at(fb) = (type == ADDITION);
				fCount += ((type == ADDITION) ? -1 : 1);
			}
		}
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FE;
		FE.resize(fCount, 3);
		int nextfIdx = 0;
		for (int fIdx = 0; fIdx < FD_stitchPartial.rows(); ++fIdx)
		{
			if (!ignored.at(fIdx))
			{
				FE.row(nextfIdx++) = FD_stitchPartial.row(fIdx);
			}
		}

		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> I, J;
		igl::remove_unreferenced(VD, FE, VC, FC, I, J);
	}
	else if (type == SUBTRACTION)
	{
		// remove patches of A of interest
		// remove all patches derived from B
		// invert patches of B of interest
		std::vector<int> use(FD_stitchPartial.rows(), 1); // 0: ignore, 1: not-ignore, 2: not-ignore(flip)
		int fCount = FD_stitchPartial.rows();
		for (const auto &pIdx : patchA_inB_ofInterest)
		{
			for (const auto &fa : patchFA_inB.at(pIdx))
			{
				use.at(fa) = 0;
				fCount--;
			}
		}
		for (int pIdx = 0; pIdx < patchFB_inA.size(); ++pIdx)
		{
			for (const auto &fb : patchFB_inA.at(pIdx))
			{
				use.at(fb) = 0;
				fCount--;
			}
		}
		for (int pIdx = 0; pIdx < patchFB_outA.size(); ++pIdx)
		{
			for (const auto &fb : patchFB_outA.at(pIdx))
			{
				use.at(fb) = 0;
				fCount--;
			}
		}
		for (const auto &pIdx : patchB_inA_ofInterest)
		{
			for (const auto &fb : patchFB_inA.at(pIdx))
			{
				if (use.at(fb) == 0)
				{
					use.at(fb) = 2;
					fCount++;
				}
				else
				{
					use.at(fb) = 2;
				}
			}
		}
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> FE;
		FE.resize(fCount, 3);
		int nextfIdx = 0;
		for (int fIdx = 0; fIdx < FD_stitchPartial.rows(); ++fIdx)
		{
			if (use.at(fIdx) == 1)
			{
				FE.row(nextfIdx++) = FD_stitchPartial.row(fIdx);
			}
			else if (use.at(fIdx) == 2)
			{
				// flipped
				FE(nextfIdx, 0) = FD_stitchPartial(fIdx, 0);
				FE(nextfIdx, 1) = FD_stitchPartial(fIdx, 2);
				FE(nextfIdx, 2) = FD_stitchPartial(fIdx, 1);
				nextfIdx++;
			}
		}

		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> I, J;
		igl::remove_unreferenced(VD, FE, VC, FC, I, J);
	}
#if 0
	Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> B;
	igl::is_vertex_manifold(FC, B);
	for (int b = 0; b < B.rows(); ++b)
	{
		if (!B(b)) {
			std::cout << "non manifold: " << J(b) << std::endl;
		}
	}
#endif

	return true;
}