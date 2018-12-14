#include "ThicknessChecker.h"

#include "stdafx.h"

#include "igl/list_to_matrix.h"

#include <iostream>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <iterator>
#include <map>

// originally from igl/readOBJ.
// there is some modificaton for handling MRGB in ZBrush.

bool read_OBJ(
	const std::string& fileName,
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& V,
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& F,
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& VC,
	Eigen::Matrix<   int, Eigen::Dynamic, Eigen::Dynamic>& FG
)
{
	// currently assume triangle mesh.

	FILE * obj_file = fopen(fileName.c_str(), "r");
	if (NULL == obj_file)
	{
		fprintf(stderr, "IOError: %s could not be opened...\n",
			fileName.c_str());
		return false;
	}

	std::vector<std::vector<double> > vV, vTC, vN;
	std::string C("");
	std::vector<std::vector<int> > vF, vFTC, vFN;
	std::string currentGroupStr("Default");
	std::vector<std::string> vGstr;
	std::map<std::string, int> groupStrToId;
	groupStrToId[currentGroupStr] = 0;

	vV.clear();
	vTC.clear();
	vN.clear();
	vF.clear();
	vFTC.clear();
	vFN.clear();

	// variables and constants to assist parsing the .obj file
	// Constant strings to compare against
	std::string v("v");
	std::string vn("vn");
	std::string vt("vt");
	std::string f("f");
	std::string g("g");
	std::string MRGB("#MRGB");
	std::string tic_tac_toe("#");
#ifndef IGL_LINE_MAX
#  define IGL_LINE_MAX 2048
#endif

	char line[IGL_LINE_MAX];
	int line_no = 1;
	while (fgets(line, IGL_LINE_MAX, obj_file) != NULL)
	{
		char type[IGL_LINE_MAX];
		// Read first word containing type
		if (sscanf(line, "%s", type) == 1)
		{
			// Get pointer to rest of line right after type
			char * l = &line[strlen(type)];
			if (type == v)
			{
				std::istringstream ls(&line[1]);
				std::vector<double> vertex{ std::istream_iterator<double>(ls), std::istream_iterator<double>() };

				if (vertex.size() < 3)
				{
					fprintf(stderr,
						"Error: readOBJ() vertex on line %d should have at least 3 coordinates",
						line_no);
					fclose(obj_file);
					return false;
				}

				vV.push_back(vertex);
			}
			else if (type == vn)
			{
				double x[3];
				int count =
					sscanf(l, "%lf %lf %lf\n", &x[0], &x[1], &x[2]);
				if (count != 3)
				{
					fprintf(stderr,
						"Error: readOBJ() normal on line %d should have 3 coordinates",
						line_no);
					fclose(obj_file);
					return false;
				}
				std::vector<double> normal(count);
				for (int i = 0; i < count; i++)
				{
					normal[i] = x[i];
				}
				vN.push_back(normal);
			}
			else if (type == vt)
			{
				double x[3];
				int count =
					sscanf(l, "%lf %lf %lf\n", &x[0], &x[1], &x[2]);
				if (count != 2 && count != 3)
				{
					fprintf(stderr,
						"Error: readOBJ() texture coords on line %d should have 2 "
						"or 3 coordinates (%d)",
						line_no, count);
					fclose(obj_file);
					return false;
				}
				std::vector<double> tex(count);
				for (int i = 0; i < count; i++)
				{
					tex[i] = x[i];
				}
				vTC.push_back(tex);
			}
			else if (type == f)
			{
				const auto & shift = [&vV](const int i)->int
				{
					return i < 0 ? i + vV.size() : i - 1;
				};
				const auto & shift_t = [&vTC](const int i)->int
				{
					return i < 0 ? i + vTC.size() : i - 1;
				};
				const auto & shift_n = [&vN](const int i)->int
				{
					return i < 0 ? i + vN.size() : i - 1;
				};
				std::vector<int> f;
				std::vector<int> ftc;
				std::vector<int> fn;
				// Read each "word" after type
				char word[IGL_LINE_MAX];
				int offset;
				while (sscanf(l, "%s%n", word, &offset) == 1)
				{
					// adjust offset
					l += offset;
					// Process word
					long int i, it, in;
					if (sscanf(word, "%ld/%ld/%ld", &i, &it, &in) == 3)
					{
						f.push_back(shift(i));
						ftc.push_back(shift_t(it));
						fn.push_back(shift_n(in));
					}
					else if (sscanf(word, "%ld/%ld", &i, &it) == 2)
					{
						f.push_back(shift(i));
						ftc.push_back(shift_t(it));
					}
					else if (sscanf(word, "%ld//%ld", &i, &in) == 2)
					{
						f.push_back(shift(i));
						fn.push_back(shift_n(in));
					}
					else if (sscanf(word, "%ld", &i) == 1)
					{
						f.push_back(shift(i));
					}
					else
					{
						fprintf(stderr,
							"Error: readOBJ() face on line %d has invalid element format\n",
							line_no);
						fclose(obj_file);
						return false;
					}
				}
				if (
					(f.size() > 0 && fn.size() == 0 && ftc.size() == 0) ||
					(f.size() > 0 && fn.size() == f.size() && ftc.size() == 0) ||
					(f.size() > 0 && fn.size() == 0 && ftc.size() == f.size()) ||
					(f.size() > 0 && fn.size() == f.size() && ftc.size() == f.size()))
				{
					// No matter what add each type to lists so that lists are the
					// correct lengths
					vF.push_back(f);
					vFTC.push_back(ftc);
					vFN.push_back(fn);
					vGstr.push_back(currentGroupStr);
				}
				else
				{
					fprintf(stderr,
						"Error: readOBJ() face on line %d has invalid format\n", line_no);
					fclose(obj_file);
					return false;
				}
			}
			else if (type == g)
			{
				char body[IGL_LINE_MAX];
				int count = sscanf(l, "%s\n", body);
				std::string gStr(body, count);
				if (groupStrToId.find(gStr) == groupStrToId.end())
				{
					groupStrToId[gStr] = groupStrToId.size();
				}
				currentGroupStr = gStr;
			}
			else if (strlen(type) >= 1 && type == MRGB)
			{
				char body[IGL_LINE_MAX];
				int count = sscanf(l, "%s\n", body);
				C.append(body);
			}
			else if (strlen(type) >= 1 && (type[0] == '#' ||
				type[0] == 'g' ||
				type[0] == 's' ||
				strcmp("usemtl", type) == 0 ||
				strcmp("mtllib", type) == 0))
			{
				//ignore comments or other shit
			}
			else
			{
				//ignore any other lines
				fprintf(stderr,
					"Warning: readOBJ() ignored non-comment line %d:\n  %s",
					line_no,
					line);
			}
		}
		else
		{
			// ignore empty line
		}
		line_no++;
	}
	fclose(obj_file);

	//////////////////////////////////
	// convert to Eigen Matrix Style
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> TC;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> CN;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> FTC;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> FN;

	bool V_rect = igl::list_to_matrix(vV, V);
	const char * format = "Failed to cast %s to matrix: min (%d) != max (%d)\n";
	if (!V_rect)
	{
		printf(format, "V", igl::min_size(vV), igl::max_size(vV));
		return false;
	}
	bool F_rect = igl::list_to_matrix(vF, F);
	if (!F_rect)
	{
		printf(format, "F", igl::min_size(vF), igl::max_size(vF));
		return false;
	}
	if (!vN.empty())
	{
		bool VN_rect = igl::list_to_matrix(vN, CN);
		if (!VN_rect)
		{
			printf(format, "CN", igl::min_size(vN), igl::max_size(vN));
			return false;
		}
	}

	if (!vFN.empty() && !vFN[0].empty())
	{
		bool FN_rect = igl::list_to_matrix(vFN, FN);
		if (!FN_rect)
		{
			printf(format, "FN", igl::min_size(vFN), igl::max_size(vFN));
			return false;
		}
	}

	if (!vTC.empty())
	{

		bool T_rect = igl::list_to_matrix(vTC, TC);
		if (!T_rect)
		{
			printf(format, "TC", igl::min_size(vTC), igl::max_size(vTC));
			return false;
		}
	}
	if (!vFTC.empty() && !vFTC[0].empty())
	{

		bool FTC_rect = igl::list_to_matrix(vFTC, FTC);
		if (!FTC_rect)
		{
			printf(format, "FTC", igl::min_size(vFTC), igl::max_size(vFTC));
			return false;
		}
	}

	VC.resize(V.rows(), 4);
	VC.setOnes();
	VC *= 255;
	if (C.length() > 0)
	{
		// convert to Matrix Style
		for (int v = 0; v < VC.rows(); ++v)
		{
			for (int mrgb = 0; mrgb < 4; ++mrgb)
			{
				VC(v, mrgb) = std::stoi(C.substr(v * 8 + mrgb * 2, 2), nullptr, 16);
			}
		}
	}

	FG.resize(F.rows(), 1);
	FG.setZero();
	for (int f = 0; f < F.rows(); ++f)
	{
		FG(f, 0) = groupStrToId[vGstr.at(f)];
	}

	return true;
}
