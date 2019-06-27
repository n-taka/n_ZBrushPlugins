#define OCLSTRINGIFY(...) #__VA_ARGS__

#include "ThicknessChecker.h"

#pragma warning(push)
#pragma warning(disable : 4018 4129 4244 4267 4305 4566 4819 4996)
#include "igl/per_face_normals.h"
#include "igl/triangle_triangle_adjacency.h"
#include "igl/AABB.h"
#pragma warning(pop)

#include <unordered_set>

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

#if 0
bool AMP_rayBoxIntersection(
	const concurrency::graphics::float_3 &orig,
	const concurrency::graphics::float_3 &dir,
	const concurrency::graphics::float_3 &bb_min,
	const concurrency::graphics::float_3 &bb_max) restrict(amp, cpu)
{
	// This should be precomputed and provided as input
	concurrency::graphics::float_3 inv_dir(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	bool signX, signY, signZ;
	signX = (inv_dir.x < 0.0f);
	signY = (inv_dir.y < 0.0f);
	signZ = (inv_dir.z < 0.0f);
	// http://people.csail.mit.edu/amy/papers/box-jgt.pdf
	// "An Efficient and Robust Ray–Box Intersection Algorithm"
	float tmin, tmax, tymin, tymax, tzmin, tzmax;

	tmin = (((signX) ? bb_max : bb_min).x - orig.x) * inv_dir.x;
	tmax = (((signX) ? bb_min : bb_max).x - orig.x) * inv_dir.x;
	tymin = (((signY) ? bb_max : bb_min).y - orig.y) * inv_dir.y;
	tymax = (((signY) ? bb_min : bb_max).y - orig.y) * inv_dir.y;
	if ((tmin > tymax) || (tymin > tmax))
	{
		return false;
	}
	if (tymin > tmin)
	{
		tmin = tymin;
	}
	if (tymax < tmax)
	{
		tmax = tymax;
	}
	tzmin = (((signZ) ? bb_max : bb_min).z - orig.z) * inv_dir.z;
	tzmax = (((signZ) ? bb_min : bb_max).z - orig.z) * inv_dir.z;
	if ((tmin > tzmax) || (tzmin > tmax))
	{
		return false;
	}
	if (tzmin > tmin)
	{
		tmin = tzmin;
	}
	if (tzmax < tmax)
	{
		tmax = tzmax;
	}
	// t1 == inf
	// if(!( (tmin < t1) && (tmax > t0) ))
	if (!(tmax > 0.0f))
	{
		return false;
	}
	return true;
}

#define IGL_RAY_TRI_EPSILON 0.000001
#define IGL_RAY_TRI_CROSS(dest, v1, v2)         \
	dest.x = (v1).y * (v2).z - (v1).z * (v2).y; \
	dest.y = (v1).z * (v2).x - (v1).x * (v2).z; \
	dest.z = (v1).x * (v2).y - (v1).y * (v2).x;
#define IGL_RAY_TRI_DOT(v1, v2) ((v1).x * (v2).x + (v1).y * (v2).y + (v1).z * (v2).z)
#define RAY_TRI_ADD(dest, v1, v2) \
	dest.x = (v1).x + (v2).x;     \
	dest.y = (v1).y + (v2).y;     \
	dest.z = (v1).z + (v2).z;
#define IGL_RAY_TRI_SUB(dest, v1, v2) \
	dest.x = (v1).x - (v2).x;         \
	dest.y = (v1).y - (v2).y;         \
	dest.z = (v1).z - (v2).z;

float AMP_rayTriangleIntersection(
	concurrency::graphics::float_3 orig,
	concurrency::graphics::float_3 dir,
	concurrency::graphics::float_3 vert0,
	concurrency::graphics::float_3 vert1,
	concurrency::graphics::float_3 vert2) restrict(amp, cpu)
{
	float t, u, v;
	concurrency::graphics::float_3 edge1, edge2, tvec, pvec, qvec;
	float det, inv_det;

	/* find vectors for two edges sharing vert0 */
	IGL_RAY_TRI_SUB(edge1, vert1, vert0);
	IGL_RAY_TRI_SUB(edge2, vert2, vert0);

	/* begin calculating determinant - also used to calculate U parameter */
	IGL_RAY_TRI_CROSS(pvec, dir, edge2);

	/* if determinant is near zero, ray lies in plane of triangle */
	det = IGL_RAY_TRI_DOT(edge1, pvec);

	if (det > IGL_RAY_TRI_EPSILON)
	{
		/* calculate distance from vert0 to ray origin */
		IGL_RAY_TRI_SUB(tvec, orig, vert0);

		/* calculate U parameter and test bounds */
		u = IGL_RAY_TRI_DOT(tvec, pvec);
		if (u < 0.0f || u > det)
			return -1.0f;

		/* prepare to test V parameter */
		IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		v = IGL_RAY_TRI_DOT(dir, qvec);
		if (v < 0.0f || u + v > det)
			return -1.0f;
	}
	else if (det < -IGL_RAY_TRI_EPSILON)
	{
		/* calculate distance from vert0 to ray origin */
		IGL_RAY_TRI_SUB(tvec, orig, vert0);

		/* calculate U parameter and test bounds */
		u = IGL_RAY_TRI_DOT(tvec, pvec);
		/*      printf("*u=%f\n",(float)*u); */
		/*      printf("det=%f\n",det); */
		if (u > 0.0f || u < det)
			return -1.0f;

		/* prepare to test V parameter */
		IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		v = IGL_RAY_TRI_DOT(dir, qvec);
		if (v > 0.0f || u + v < det)
			return -1.0f;
	}
	else
		return -1.0f; /* ray is parallel to the plane of the triangle */

	inv_det = 1.0f / det;

	/* calculate t, ray intersects triangle */
	t = IGL_RAY_TRI_DOT(edge2, qvec) * inv_det;
	//u *= inv_det;
	//v *= inv_det;

	return t;
}

float AMP_rayMeshIntersections(
	const concurrency::graphics::float_3 &orig,
	const concurrency::graphics::float_3 &dir,
	const concurrency::array_view<concurrency::graphics::float_3, 1> &bb_mins,
	const concurrency::array_view<concurrency::graphics::float_3, 1> &bb_maxs,
	const concurrency::array_view<int, 1> &elements,
	const concurrency::array_view<concurrency::graphics::float_3, 1> &V,
	const concurrency::array_view<concurrency::graphics::int_3, 1> &F,
	const concurrency::array_view<concurrency::graphics::float_3, 1> &FN) restrict(amp, cpu)
{
	float min_t = -1.0;
	concurrency::index<1> currentIdx(0);
	// do traverse...
	while (unsigned(currentIdx[0]) < bb_mins.get_extent().size())
	{
		// check ray-box intersection (currentIdx)
		if (AMP_rayBoxIntersection(orig, dir, bb_mins[currentIdx], bb_maxs[currentIdx]))
		{
			// check this box is leaf or non-leaf
			if (elements[currentIdx] < 0)
			{
				// non-leaf: traverse
				currentIdx[0] = currentIdx[0] * 2 + 1;
			}
			else
			{
				// leaf: ray-triangle intersection
				// ignore false intersection (see original paper)
				// as bonus, intersection with itself is also ignored.
				if (IGL_RAY_TRI_DOT(FN[elements[currentIdx]], dir) > 0.0f)
				{
					const concurrency::graphics::float_3 &vert0 = V[F[elements[currentIdx]].x];
					const concurrency::graphics::float_3 &vert1 = V[F[elements[currentIdx]].y];
					const concurrency::graphics::float_3 &vert2 = V[F[elements[currentIdx]].z];
					float t = AMP_rayTriangleIntersection(orig, dir, vert0, vert1, vert2);
					if (t > 0.0 && (t < min_t || min_t < 0.0))
					{
						min_t = t;
					}
				}
				// then move to
				// sibling (odd nodes)
				// sibling of the parent, grand parent, ... (even nodes)
				while (currentIdx[0] % 2 == 0 && currentIdx[0] != 0)
				{
					currentIdx[0] = (currentIdx[0] - 1) / 2;
				}
				if (currentIdx[0] == 0)
				{
					break;
				}
				currentIdx[0] = currentIdx[0] + 1;
			}
		}
		else
		{
			// then move to
			// sibling (odd nodes)
			// sibling of the parent, grand parent, ... (even nodes)
			while (currentIdx[0] % 2 == 0 && currentIdx[0] != 0)
			{
				currentIdx[0] = (currentIdx[0] - 1) / 2;
			}
			if (currentIdx[0] == 0)
			{
				break;
			}
			currentIdx += 1;
		}
	}
	return min_t;
}
#endif

void openCL_computeSDF(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F,
	const int &chunkSize,
	const cl::Device &device,
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &FaceSDF)
{
	cl_int err = CL_SUCCESS;
	try
	{
		//////
		// setup buffers for parallel computation
		std::vector<float> cl_V(V.rows() * 3);
		for (int v = 0; v < V.rows(); ++v)
		{
			for (int i = 0; i < 3; ++i)
			{
				cl_V.at(v * 3 + i) = V(v, i);
			}
		}

		std::vector<int> cl_F(F.rows() * 3);
		for (int f = 0; f < F.rows(); ++f)
		{
			for (int i = 0; i < 3; ++i)
			{
				cl_F.at(f * 3 + i) = F(f, i);
			}
		}

		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> FN;
		igl::per_face_normals(V, F, FN);
		std::vector<float> cl_FN(int(FN.rows()) * 3);
		for (int fn = 0; fn < FN.rows(); ++fn)
		{
			for (int i = 0; i < 3; ++i)
			{
				cl_FN.at(fn * 3 + i) = FN(fn, i);
			}
		}
		// end setup.
		//////
		//////
		// construct AABBTree
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> bb_mins;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> bb_maxs;
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> elements;
		igl::AABB<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, 3> aabb;
		aabb.init(V, F);
		aabb.serialize(bb_mins, bb_maxs, elements);
		// construct serialized AABB
		std::vector<float> cl_bb_mins(int(bb_mins.rows()) * 3);
		std::vector<float> cl_bb_maxs(int(bb_maxs.rows()) * 3);
		std::vector<int> cl_elements(int(elements.rows()));
		for (int bb = 0; bb < bb_mins.rows(); ++bb)
		{
			for (int i = 0; i < 3; ++i)
			{
				cl_bb_mins.at(bb * 3 + i) = bb_mins(bb, i);
				cl_bb_maxs.at(bb * 3 + i) = bb_maxs(bb, i);
			}
			cl_elements.at(bb) = elements(bb, 0);
		}
		//////

		//////
		// generate uniform disk sampling (with weighting) (similar to the CGAL)
		std::vector<float> SW_vec(RAYCOUNT * 3);
		const float golden_ratio = 3.0f - static_cast<float>(std::sqrt(5.0));
		const float diskRadius = static_cast<float>(std::tan(CONE_OPENING_ANGLE / 2.0f));
		for (int sw = 0; sw < RAYCOUNT; ++sw)
		{
			float Q = sw * golden_ratio * static_cast<float>(PI);
			float R = std::sqrt(static_cast<float>(sw) / RAYCOUNT);
			float weight = exp(-0.5f * (std::pow(R / 3.0f, 2)));

			SW_vec.at(sw * 3 + 0) = diskRadius * R * cos(Q);
			SW_vec.at(sw * 3 + 1) = diskRadius * R * sin(Q);
			SW_vec.at(sw * 3 + 2) = weight;
		}
		//////

		//////
		// OpenCL setup and execution
		cl::Context context({device});

		cl::Program::Sources sources;
		// do something for source
		const char source[] =
#include "kernel.cl"
			;

		sources.push_back(std::make_pair(source, std::strlen(source)));

		cl::Program program = cl::Program(context, sources);
		if (program.build({device}) != CL_SUCCESS)
		{
			std::cout << " Error building: " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device) << "\n";
			exit(1);
		}

		cl::Kernel test1(program, "test1", &err);
		cl::Kernel test2(program, "test2", &err);

		std::vector<int> result(15, 0); // 3*5

		cl::Buffer buffer_A(context, CL_MEM_READ_WRITE, result.size() * sizeof(int), result.data());
		int sizeX, sizeY;
		sizeX = 3;
		sizeY = 5;
		test1.setArg(0, buffer_A);
		test1.setArg(1, sizeX);
		test2.setArg(0, buffer_A);
		test2.setArg(1, sizeX);

		cl::Event event;
		cl::CommandQueue queue(context, device, 0, &err);
		queue.enqueueNDRangeKernel(
			test1, cl::NullRange, cl::NDRange(3, 5), cl::NullRange, NULL, &event);
		event.wait();

		queue.enqueueReadBuffer(
			buffer_A,
			true,
			0,
			result.size() * sizeof(int),
			result.data());

		for (int y = 0; y < sizeY; ++y)
		{
			for (int x = 0; x < sizeX; ++x)
			{
				std::cout << result.at(sizeX * y + x) << " ";
			}
			std::cout << std::endl;
		}
	}
	catch (cl::Error err)
	{
		std::cerr << "ERROR: " << err.what() << "(" << err.err() << ")" << std::endl;
	}
	return;

#if 0

	//////
	// #triangle x #ray array. compute ray-mesh intersection and write to this vector in parallel
	std::vector<float> T_vec(int(F.rows()) * RAYCOUNT, 0.0f);
	concurrency::array_view<float, 2> AMP_T(int(F.rows()), RAYCOUNT, T_vec);

	for (int offset = 0; offset < int(F.rows()); offset += chunkSize)
	{
		int endIdx = std::min(offset + chunkSize, int(F.rows()));
		concurrency::array_view<float, 2> AMP_T_partial(endIdx - offset, RAYCOUNT, &T_vec[offset * RAYCOUNT]);
		//////
		concurrency::parallel_for_each(acc.get_default_view(),
									   AMP_T_partial.extent,
									   [=](concurrency::index<2> idx) restrict(amp) {
										   // source point
										   concurrency::graphics::float_3 source =
											   (AMP_V[AMP_F[concurrency::index<1>(idx[0]) + offset].x] + AMP_V[AMP_F[concurrency::index<1>(idx[0]) + offset].y] + AMP_V[AMP_F[concurrency::index<1>(idx[0]) + offset].z]) / 3.0f;

										   // generate dir (use pre-defined??)
										   concurrency::graphics::float_3 dir = -AMP_FN[idx[0] + offset];
										   // directions on disk
										   concurrency::graphics::float_3 base1, base2;
										   IGL_RAY_TRI_SUB(base1, AMP_V[concurrency::index<1>(AMP_F[concurrency::index<1>(idx[0]) + offset].x)], source);
										   float norm2_1 = IGL_RAY_TRI_DOT(base1, base1);
										   float norm_1 = concurrency::fast_math::sqrtf(norm2_1);
										   base1 /= norm_1;
										   IGL_RAY_TRI_CROSS(base2, dir, base1);
										   base1 *= AMP_SW[idx[1]].x;
										   base2 *= AMP_SW[idx[1]].y;

										   RAY_TRI_ADD(dir, dir, base1);
										   RAY_TRI_ADD(dir, dir, base2);

										   float norm2_d = IGL_RAY_TRI_DOT(dir, dir);
										   float norm_d = concurrency::fast_math::sqrtf(norm2_d);
										   dir /= norm_d;

										   // do computation
										   float t = AMP_rayMeshIntersections(source, dir, AMP_bb_mins, AMP_bb_maxs, AMP_elements, AMP_V, AMP_F, AMP_FN);

										   // write-back
										   AMP_T_partial[idx] = t;
									   });
		AMP_T_partial.synchronize();
	}

	std::cout << "intersection done." << std::endl;

	std::vector<float> FaceSDF_vec(int(F.rows()), 0.0f);
	concurrency::array_view<float, 1> AMP_FaceSDF(int(F.rows()), FaceSDF_vec);

	const float number_of_mad = 1.5f;
	for (int offset = 0; offset < int(F.rows()); offset += chunkSize)
	{
		int endIdx = std::min(offset + chunkSize, int(F.rows()));
		concurrency::array_view<float, 1> AMP_FaceSDF_partial(endIdx - offset, &FaceSDF_vec[offset]);
		concurrency::parallel_for_each(acc.get_default_view(),
									   AMP_FaceSDF_partial.extent,
									   [=](concurrency::index<1> idx) restrict(amp) {
										   float avg = 0.0f;
										   float validCount = 0.0f;
										   for (int r = 0; r < RAYCOUNT; ++r)
										   {
											   if (AMP_T(idx[0] + offset, r) >= 0.0f)
											   {
												   avg += AMP_T(idx[0] + offset, r);
												   validCount += 1.0f;
											   }
										   }
										   if (validCount < 0.5f)
										   {
											   AMP_FaceSDF_partial[idx] = -1.0f;
											   return;
										   }
										   avg /= validCount;

										   // use standard deviation and ratio of 1.5; (different from CGAL (median absolute))
										   float standardDeviation = 0.0f;
										   float acceptCount = 0.0f;
										   for (int r = 0; r < RAYCOUNT; ++r)
										   {
											   if (AMP_T(idx[0] + offset, r) >= 0.0f)
											   {
												   float diff = AMP_T(idx[0] + offset, r) - avg;
												   standardDeviation += (diff * diff);
												   acceptCount += 1.0f;
											   }
										   }

										   if (acceptCount > 0.5f)
										   {
											   standardDeviation /= acceptCount;
											   standardDeviation = concurrency::fast_math::sqrt(standardDeviation);

											   float weightedSDF = 0.0f;
											   float weightSum = 0.0f;
											   for (int r = 0; r < RAYCOUNT; ++r)
											   {
												   if (AMP_T(idx[0] + offset, r) >= 0.0f)
												   {
													   const float deviation = concurrency::fast_math::fabs(AMP_T(idx[0] + offset, r) - avg);
													   if (deviation <= number_of_mad * standardDeviation)
													   {
														   weightedSDF += (AMP_SW[r].z * AMP_T(idx[0] + offset, r));
														   weightSum += AMP_SW[r].z;
													   }
												   }
											   }
											   if (weightSum > 0.0f)
											   {
												   AMP_FaceSDF_partial[idx] = weightedSDF / weightSum;
											   }
											   else
											   {
												   AMP_FaceSDF_partial[idx] = -1.0f;
											   }
										   }
										   else
										   {
											   AMP_FaceSDF_partial[idx] = -1.0f;
										   }
									   });
		AMP_FaceSDF_partial.synchronize();
	}
	FaceSDF.resize(F.rows(), 1);
	for (int f = 0; f < F.rows(); ++f)
	{
		FaceSDF(f, 0) = AMP_FaceSDF[f];
	}

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
		for (const auto &idx : idSet)
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
#endif
}
