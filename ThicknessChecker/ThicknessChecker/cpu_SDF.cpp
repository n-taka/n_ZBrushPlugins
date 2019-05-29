// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "ThicknessChecker.h"

#include "igl/per_face_normals.h"
#include "igl/triangle_triangle_adjacency.h"
#include "igl/AABB.h"

#include <vector>
#include <unordered_set>
#include <thread>
#include <mutex>

////
// implementation
////
#define PI (3.14159265359)
// 30: original paper
// 25: CGAL implementation
#define RAYCOUNT (25)
//
// [0, pi]
#define CONE_OPENING_ANGLE (2.0f / 3.0f * PI)

namespace {
	std::mutex m0, m1;
}

float CPU_rayMeshIntersections(
	const Eigen::Matrix<float, 1, 3> & orig,
	const Eigen::Matrix<float, 1, 3> & dir,
	igl::AABB<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, 3> & aabb,
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& F,
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& FN)
{
	float min_t = -1.0f;
	std::vector<igl::Hit> hits;
	aabb.intersect_ray(V, F, orig, dir, hits);

	for (int h = 0; h < hits.size(); ++h)
	{
		if (FN.row(hits.at(h).id).dot(dir) > 0.0f)
		{
			float t = hits.at(h).t;
			if (t > 0.0 && (t < min_t || min_t < 0.0))
			{
				min_t = t;
			}
		}
	}

	return min_t;
}

void CPU_computeSDF(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& F,
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& FaceSDF)
{
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> FN;
	igl::per_face_normals(V, F, FN);

	//////
	// construct AABBTree
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> bb_mins;
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> bb_maxs;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> elements;
	igl::AABB<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, 3> aabb;
	aabb.init(V, F);

	//////
	// generate uniform disk sampling (with weighting) (similar to the CGAL)
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> SW;
	SW.resize(RAYCOUNT, 3);
	const float golden_ratio = 3.0 - std::sqrt(5.0);
	const float diskRadius = tan(CONE_OPENING_ANGLE / 2.0f);
	for (int sw = 0; sw < RAYCOUNT; ++sw)
	{
		float Q = sw * golden_ratio * PI;
		float R = std::sqrt(static_cast<float>(sw) / RAYCOUNT);
		float weight = exp(-0.5f * (std::pow(R / 3.0f, 2)));

		SW(sw, 0) = diskRadius * R * cos(Q);
		SW(sw, 1) = diskRadius * R * sin(Q);
		SW(sw, 2) = weight;
	}
	//////

	//////
	// #triangle x #ray array. compute ray-mesh intersection and write to this vector in parallel
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T;
	T.resize(int(F.rows()), RAYCOUNT);

	const int numOfThreads = std::max(static_cast<int>(std::thread::hardware_concurrency()), 1);

	{
		int taskId = 0;
		std::vector<std::thread> threads;
		for (int th = 0; th < numOfThreads; ++th) {
			threads.push_back(std::thread([&] {
				// todo get my ID
				while (true) {
					int tri = -1;
					int r = -1;
					{
						std::lock_guard<std::mutex> guard(m0);
						tri = taskId / RAYCOUNT;
						r = taskId % RAYCOUNT;
						++taskId;
					}
					if (tri < F.rows())
					{
						// source point
						Eigen::Matrix<float, 1, 3> source = (V.row(F(tri, 0)) + V.row(F(tri, 1)) + V.row(F(tri, 2))) / 3.0f;
						// generate dir (use pre-defined??)
						Eigen::Matrix<float, 1, 3> dir = -FN.row(tri);

						// directions on disk
						Eigen::Matrix<float, 1, 3> base1, base2;
						base1 = V.row(F(tri, 0)) - source;
						base1.normalize();
						base2 = dir.cross(base1);
						base1 *= SW(r, 0);
						base2 *= SW(r, 1);

						dir += base1;
						dir += base2;

						dir.normalize();

						// do computation
						float t = CPU_rayMeshIntersections(source, dir, aabb, V, F, FN);
						// write-back
						T(tri, r) = t;
					}
					else
					{
						break;
					}
				}
				}));
		}
		for (std::thread& th : threads) {
			th.join();
		}
	}

	std::cout << "intersection done." << std::endl;
	//for (int t = 0; t < F.rows(); ++t)
	//{
	//	for (int r = 0; r < RAYCOUNT; ++r) {
	//		std::cout << AMP_T[concurrency::index<2>(t, r)] << " ";
	//	}
	//	std::cout << std::endl;
	//}

	FaceSDF.resize(F.rows(), 1);
	FaceSDF.setZero();

	const float number_of_mad = 1.5f;

	{
		int taskId = 0;
		std::vector<std::thread> threads;
		for (int th = 0; th < numOfThreads; ++th) {
			threads.push_back(std::thread([&] {
				// todo get my ID
				while (true) {
					int tri = -1;
					{
						std::lock_guard<std::mutex> guard(m1);
						tri = taskId;
						++taskId;
					}
					if (tri < F.rows())
					{
						float avg = 0.0f;
						float validCount = 0.0f;
						for (int r = 0; r < RAYCOUNT; ++r)
						{
							if (T(tri, r) >= 0.0f)
							{
								avg += T(tri, r);
								validCount += 1.0f;
							}
						}
						if (validCount < 0.5f)
						{
							FaceSDF(tri) = -1.0f;
							return;
						}
						avg /= validCount;

						// use standard deviation and ratio of 1.5; (different from CGAL (median absolute))
						float standardDeviation = 0.0f;
						float acceptCount = 0.0f;
						for (int r = 0; r < RAYCOUNT; ++r)
						{
							if (T(tri, r) >= 0.0f)
							{
								float diff = T(tri, r) - avg;
								standardDeviation += (diff * diff);
								acceptCount += 1.0f;
							}
						}

						if (acceptCount > 0.5f)
						{
							standardDeviation /= acceptCount;
							standardDeviation = sqrt(standardDeviation);

							float weightedSDF = 0.0f;
							float weightSum = 0.0f;
							for (int r = 0; r < RAYCOUNT; ++r)
							{
								if (T(tri, r) >= 0.0f)
								{
									const float deviation = fabs(T(tri, r) - avg);
									if (deviation <= number_of_mad * standardDeviation)
									{
										weightedSDF += (SW(r, 2) * T(tri, r));
										weightSum += SW(r, 2);
									}
								}
							}
							if (weightSum > 0.0f)
							{
								FaceSDF(tri) = weightedSDF / weightSum;
							}
							else
							{
								FaceSDF(tri) = -1.0f;
							}
						}
						else
						{
							FaceSDF(tri) = -1.0f;
						}
					}
					else
					{
						break;
					}
				}
				}));
		}
		for (std::thread& th : threads) {
			th.join();
		}
	}


	/////////////////
	std::cout << "raw SDF done." << std::endl;

	//////
	// post process
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> TT;
	igl::triangle_triangle_adjacency(F, TT);

	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> FaceSDF_backup = FaceSDF;
	std::unordered_set<int> idSet, idSetNext;
	for (int f = 0; f < FaceSDF.rows(); ++f)
	{
		if (FaceSDF(f, 0) < 0.0f)
		{
			idSet.insert(f);
		}
	}
	while (!(idSet.empty()))
	{
		for (const auto& idx : idSet)
		{
			float val = 0.0f;
			int count = 0;
			for (int e = 0; e < 3; ++e)
			{
				if (FaceSDF_backup(TT(idx, e)) >= 0.0f)
				{
					val += FaceSDF_backup(TT(idx, e));
					count++;
				}
			}
			if (count > 0)
			{
				FaceSDF(idx, 0) = val / count;
			}
			else
			{
				idSetNext.insert(idx);
			}
		}
		idSet = idSetNext;
		idSetNext.clear();
		FaceSDF_backup = FaceSDF;
	}
	//////
}
