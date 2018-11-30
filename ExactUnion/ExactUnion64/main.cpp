#include <cstring>
#include "main.h"

#include <cstdlib>
#include <vector>
#include <map>
#include <unordered_map>

#include "Eigen/Core"

#undef THIS
#include "igl/readOBJ.h"
#include "igl/copyleft/cgal/mesh_boolean.h"
#include "igl/writeOBJ.h"
#define THIS void

// a sample exported function
float DLL_EXPORT Version(char* pDontCare, double optValue, char* pOptBuffer1, int optBuffer1Size,
	char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	return 1.0f;
}

void readOBJWithG(
	const char* fileName,
	std::vector< std::vector<double> >& QuadV,
	std::vector< std::vector<   int> >& QuadF,
	std::vector<int>& QuadG,
	std::unordered_map< std::string, int >& groupName2Id,
	std::unordered_map< int, std::string >& id2GroupName)
{
	int currentGId = -1;
	int freshGId = 0;

	////////
	// originally from readOBJ in libigl, but I remove some part that I don't use and add support for 'g'.
	////////
	FILE * obj_file = fopen(fileName, "r");
	if (NULL == obj_file)
	{
		return;
	}
	// File open was successful so clear outputs
	QuadV.clear();
	QuadF.clear();
	// set some dummy
	std::vector< std::vector<double> > TC, N;
	std::vector< std::vector<int> > FTC, FN;

	std::string v("v");
	std::string g("g");
	std::string f("f");
#  define IGL_LINE_MAX 2048

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
					return;
				}

				QuadV.push_back(vertex);
			}
			else if (type == g)
			{
				char gn[IGL_LINE_MAX];
				sscanf(line + 1, "%s", gn);
				std::string groupName(gn);
				if (groupName2Id.find(groupName) == groupName2Id.end())
				{
					groupName2Id[groupName] = ++freshGId;
					id2GroupName[groupName2Id[groupName]] = groupName;
				}
				currentGId = groupName2Id[groupName];
			}
			else if (type == f)
			{
				if (currentGId == -1)
				{
					char buf[100];
					sprintf(buf, "%d", freshGId);
					std::string groupName(buf);
					groupName2Id[groupName] = ++freshGId;
					id2GroupName[groupName2Id[groupName]] = groupName;
					currentGId = groupName2Id[groupName];
				}
				const auto & shift = [&QuadV](const int i)->int
				{
					return i < 0 ? i + QuadV.size() : i - 1;
				};
				const auto & shift_t = [&TC](const int i)->int
				{
					return i < 0 ? i + TC.size() : i - 1;
				};
				const auto & shift_n = [&N](const int i)->int
				{
					return i < 0 ? i + N.size() : i - 1;
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
						return;
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
					QuadF.push_back(f);
					QuadG.push_back(currentGId);
				}
				else
				{
					fprintf(stderr,
						"Error: readOBJ() face on line %d has invalid format\n", line_no);
					fclose(obj_file);
					return;
				}
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
}

void triangulate(
	const std::vector< std::vector<double> >& QuadV,
	const std::vector< std::vector<   int> >& QuadF,
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& V,
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& F,
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& Tri2Quad,
	std::vector< std::vector<int> >& Quad2Tri)
{
	V.resize(QuadV.size(), 3);
	for (int v = 0; v < V.rows(); ++v)
	{
		V.row(v) << QuadV.at(v).at(0), QuadV.at(v).at(1), QuadV.at(v).at(2);
	}

	int fCount = 0;
	for (const auto& f : QuadF)
	{
		fCount += (f.size() - 2);
	}
	F.resize(fCount, 3);
	Tri2Quad.resize(fCount, 1);
	Quad2Tri.clear();
	Quad2Tri.reserve(QuadF.size());
	fCount = 0;
	for (int f = 0; f < QuadF.size(); ++f)
	{
		Quad2Tri.push_back(std::vector<int>({}));
		for (int vi = 0; vi < QuadF.at(f).size() - 2; ++vi)
		{
			Quad2Tri.back().push_back(fCount);
			F.row(fCount) << QuadF.at(f).at(0), QuadF.at(f).at(vi + 1), QuadF.at(f).at(vi + 2);
			Tri2Quad(fCount, 0) = f;
			++fCount;
		}
	}
}

void writeOBJWithG(
	const char* fileName,
	const std::vector< std::vector<   int> >& QuadF,
	const std::vector<int>& QuadG,
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& F,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& J,
	const std::unordered_map< int, std::string >& id2GroupName,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& Tri2Quad
)
{
	FILE * obj_file = fopen(fileName, "w");
	if (NULL == obj_file)
	{
		printf("IOError: %s could not be opened for writing...", fileName);
		return;
	}
	// Loop over V
	for (int i = 0; i < (int)V.rows(); i++)
	{
		fprintf(obj_file, "v");
		for (int j = 0; j < (int)V.cols(); ++j)
		{
			fprintf(obj_file, " %0.17g", V(i, j));
		}
		fprintf(obj_file, "\n");
	}

	// restore quads
	// use triangle 2 quad
	std::vector< std::vector<int> > restoredQuad(QuadF.size());
	for (int f = 0; f < F.rows(); ++f)
	{
		restoredQuad.at(Tri2Quad(J(f, 0), 0)).push_back(f);
	}

	// loop over restoredQuad
	int currentGroup = -1;
	int count = 0;
	for (int qidx = 0; qidx < restoredQuad.size(); ++qidx) {
		auto& q = restoredQuad.at(qidx);
		if (q.size() == 0)
		{
			// do nothing!
		}
		else {
			if (currentGroup != QuadG.at(q.at(0)))
			{
				currentGroup = QuadG.at(q.at(0));
				fprintf(obj_file, "g %s\n", id2GroupName.at(currentGroup).c_str());
			}

			if (q.size() == 2)
			{
				// write as quad (two triangles)
				std::map<int, int> count;
				for (const auto& f : q)
				{
					for (int vi = 0; vi < (int)F.cols(); ++vi)
					{
						count[F(f, vi)] += 1;
					}
				}

				int sharedCount = 0;
				for (const auto& se : count)
				{
					if (se.second >= 2)
					{
						sharedCount++;
					}
				}
				if (sharedCount >= 2)
				{
					std::vector<int> vv(4, -1);

					int f = q.at(0);
					for (int vi = 0; vi < (int)F.cols(); ++vi)
					{
						if (count[F(f, vi)] < 2)
						{
							vv.at(0) = F(f, vi);
							vv.at(1) = F(f, (vi + 1) % 3);
							vv.at(3) = F(f, (vi + 2) % 3);
							break;
						}
					}
					f = q.at(1);
					for (int vi = 0; vi < (int)F.cols(); ++vi)
					{
						if (count[F(f, vi)] < 2)
						{
							vv.at(2) = F(f, vi);
							break;
						}
					}
					fprintf(obj_file, "f");
					for (const auto& vi : vv)
					{
						// OBJ is 1-indexed
						fprintf(obj_file, " %u", vi + 1);
					}
					fprintf(obj_file, "\n");
				}
				else
				{
					// write as set of triangles
					for (const auto& f : q)
					{
						fprintf(obj_file, "f");
						for (int vi = 0; vi < (int)F.cols(); ++vi)
						{
							// OBJ is 1-indexed
							fprintf(obj_file, " %u", F(f, vi) + 1);
						}
						fprintf(obj_file, "\n");
					}
				}
			}
			else
			{
				// write as set of triangles
				for (const auto& f : q)
				{
					fprintf(obj_file, "f");
					for (int vi = 0; vi < (int)F.cols(); ++vi)
					{
						// OBJ is 1-indexed
						fprintf(obj_file, " %u", F(f, vi) + 1);
					}
					fprintf(obj_file, "\n");
				}
			}
		}
	}
	fclose(obj_file);
}


float DLL_EXPORT computeUnion(char* someText, double optValue, char* outputBuffer, int optBuffer1Size,
	char* pOptBuffer2, int optBuffer2Size, char** zData)
{
	////////
	// import
	////////
	std::vector< std::vector<double> > QuadV;
	std::vector< std::vector<   int> > QuadF;
	std::vector<int> QuadG;
	std::unordered_map< std::string, int > groupName2Id;
	std::unordered_map< int, std::string > id2GroupName;
	readOBJWithG(someText, QuadV, QuadF, QuadG, groupName2Id, id2GroupName);

	MessageBoxA(0, someText, "DLL received Message", MB_OK | MB_ICONINFORMATION);
	////////
	// convert it to Matrix style.
	////////
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> Tri2Quad;
	std::vector< std::vector<int> > Quad2Tri;

	triangulate(QuadV, QuadF, V, F, Tri2Quad, Quad2Tri);

	MessageBoxA(0, someText, "DLL received Message", MB_OK | MB_ICONINFORMATION);
	////////
	// compute union
	////////
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V0, Vout;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F0, Fout;
	Eigen::Matrix<int, Eigen::Dynamic, 1> J;
	V0.resize(0, 3);
	Vout.resize(0, 3);
	F0.resize(0, 3);
	Fout.resize(0, 3);
	igl::copyleft::cgal::mesh_boolean(V, F, V0, F0, igl::MESH_BOOLEAN_TYPE_UNION, Vout, Fout, J);
	igl::writeOBJ(someText, Vout, Fout);

	MessageBoxA(0, someText, "DLL received Message", MB_OK | MB_ICONINFORMATION);
	strcpy(outputBuffer, "Hello from my DLL");
	return 0.0f;

	////////
	// export
	////////
	//writeOBJWithG(someText, QuadF, QuadG, Vout, Fout, J, id2GroupName, Tri2Quad);

	return 0.0f;
}

BOOL WINAPI DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
	switch (fdwReason)
	{
	case DLL_PROCESS_ATTACH:
		// attach to process
		// return FALSE to fail DLL load
		break;

	case DLL_PROCESS_DETACH:
		// detach from process
		break;

	case DLL_THREAD_ATTACH:
		// attach to thread
		break;

	case DLL_THREAD_DETACH:
		// detach from thread
		break;
	}
	return TRUE; // succesful
}
