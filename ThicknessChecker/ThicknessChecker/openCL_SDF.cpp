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

void openCL_computeSDF(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &F,
	const int &chunkSize,
	const cl::Device &device,
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &FaceSDF)
{
	try
	{
		//////
		// Note: due to memory alignment, I explicitly use float"4"
		//////
		//////
		// setup buffers for parallel computation
		const size_t aligned_VCount = static_cast<size_t>(std::ceil(static_cast<float>(V.rows()) / 4.0f) * 4.0f);
		std::vector<cl_float4> cl_V(aligned_VCount, {0.0f, 0.0f, 0.0f, 0.0f});
		for (int v = 0; v < V.rows(); ++v)
		{
			cl_V.at(v).x = V(v, 0);
			cl_V.at(v).y = V(v, 1);
			cl_V.at(v).z = V(v, 2);
		}

		const size_t aligned_FCount = static_cast<size_t>(std::ceil(static_cast<float>(F.rows()) / 4.0f) * 4.0f);
		std::vector<cl_int4> cl_F(aligned_FCount, {0, 0, 0, 0});
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> FN;
		igl::per_face_normals(V, F, FN);
		std::vector<cl_float4> cl_FN(aligned_FCount, {0.0f, 0.0f, 0.0f, 0.0f});
		for (int f = 0; f < F.rows(); ++f)
		{
			cl_F.at(f).x = F(f, 0);
			cl_F.at(f).y = F(f, 1);
			cl_F.at(f).z = F(f, 2);
			cl_FN.at(f).x = FN(f, 0);
			cl_FN.at(f).y = FN(f, 1);
			cl_FN.at(f).z = FN(f, 2);
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
		const size_t aligned_BBCount = static_cast<size_t>(std::ceil(static_cast<float>(bb_mins.rows()) / 4.0f) * 4.0f);
		const size_t aligned_ElemCount = static_cast<size_t>(std::ceil(static_cast<float>(elements.rows()) / 16.0f) * 16.0f);
		std::vector<cl_float4> cl_bb_mins(aligned_BBCount, {0.0f, 0.0f, 0.0f, 0.0f});
		std::vector<cl_float4> cl_bb_maxs(aligned_BBCount, {0.0f, 0.0f, 0.0f, 0.0f});
		std::vector<cl_int> cl_elements(aligned_ElemCount, 0);
		for (int bb = 0; bb < bb_mins.rows(); ++bb)
		{
			cl_bb_mins.at(bb).x = bb_mins(bb, 0);
			cl_bb_mins.at(bb).y = bb_mins(bb, 1);
			cl_bb_mins.at(bb).z = bb_mins(bb, 2);
			cl_bb_maxs.at(bb).x = bb_maxs(bb, 0);
			cl_bb_maxs.at(bb).y = bb_maxs(bb, 1);
			cl_bb_maxs.at(bb).z = bb_maxs(bb, 2);
			cl_elements.at(bb) = elements(bb, 0);
		}
		//////

		//////
		// generate uniform disk sampling (with weighting) (similar to the CGAL)
		const int aligned_RayCount = static_cast<int>(std::ceil(static_cast<float>(RAYCOUNT) / 16.0f) * 16.0f);
		std::vector<cl_float4> cl_SW(aligned_RayCount, {0.0f, 0.0f, 0.0f, 0.0f});
		const float golden_ratio = 3.0f - static_cast<float>(std::sqrt(5.0));
		const float diskRadius = static_cast<float>(std::tan(CONE_OPENING_ANGLE / 2.0f));
		for (int sw = 0; sw < RAYCOUNT; ++sw)
		{
			float Q = sw * golden_ratio * static_cast<float>(PI);
			float R = std::sqrt(static_cast<float>(sw) / RAYCOUNT);
			float weight = exp(-0.5f * (std::pow(R / 3.0f, 2)));

			cl_SW.at(sw).x = diskRadius * R * cos(Q);
			cl_SW.at(sw).y = diskRadius * R * sin(Q);
			cl_SW.at(sw).z = weight;
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

		const size_t aligned_TCount = static_cast<size_t>(std::ceil(static_cast<float>(aligned_FCount * aligned_RayCount) / 16.0f) * 16.0f);
		std::vector<cl_float> cl_T(aligned_TCount, 0.0f);

		cl::Buffer buffer_V(context, (CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR), sizeof(cl_float4) * cl_V.size(), cl_V.data());
		cl::Buffer buffer_F(context, (CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR), sizeof(cl_int4) * cl_F.size(), cl_F.data());
		cl::Buffer buffer_FN(context, (CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR), sizeof(cl_float4) * cl_FN.size(), cl_FN.data());
		cl::Buffer buffer_SW(context, (CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR), sizeof(cl_float4) * cl_SW.size(), cl_SW.data());
		cl::Buffer buffer_BB_min(context, (CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR), sizeof(cl_float4) * cl_bb_mins.size(), cl_bb_mins.data());
		cl::Buffer buffer_BB_max(context, (CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR), sizeof(cl_float4) * cl_bb_maxs.size(), cl_bb_maxs.data());
		cl::Buffer buffer_elements(context, (CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR), sizeof(cl_int) * cl_elements.size(), cl_elements.data());
		cl::Buffer buffer_T(context, (CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR), sizeof(cl_float) * cl_T.size(), cl_T.data());

		cl::Kernel computeIntersection_partial(program, "computeIntersection_partial");

		cl_int err;
		cl::CommandQueue queue(context, device, 0, &err);
		// todo: tweak chunkSize to be aligned!!
		// make sure that chunkSize * sizeof(cl_float) is aligned (64 byte)
		const size_t aligned_chunkSize = static_cast<size_t>(std::ceil(static_cast<float>(chunkSize) / 16.0f) * 16.0f);
		for (size_t offset = 0; offset < static_cast<size_t>(F.rows()); offset += aligned_chunkSize)
		{
			const size_t alignedCount = std::min((aligned_FCount - offset), aligned_chunkSize);
			const size_t triCountToCompute = std::min((F.rows() - offset), aligned_chunkSize);

			const cl_buffer_region t_partial{offset * aligned_RayCount * sizeof(cl_float), alignedCount * aligned_RayCount * sizeof(cl_float)};
			cl::Buffer buffer_T_partial = buffer_T.createSubBuffer((CL_MEM_READ_WRITE), CL_BUFFER_CREATE_TYPE_REGION, &t_partial);
			computeIntersection_partial.setArg(0, buffer_V);
			computeIntersection_partial.setArg(1, buffer_F);
			computeIntersection_partial.setArg(2, buffer_FN);
			computeIntersection_partial.setArg(3, buffer_SW);
			computeIntersection_partial.setArg(4, buffer_BB_min);
			computeIntersection_partial.setArg(5, buffer_BB_max);
			computeIntersection_partial.setArg(6, buffer_elements);
			computeIntersection_partial.setArg(7, aligned_RayCount);
			computeIntersection_partial.setArg(8, static_cast<int>(elements.rows()));
			computeIntersection_partial.setArg(9, buffer_T_partial);

			queue.enqueueNDRangeKernel(
				computeIntersection_partial, cl::NullRange, cl::NDRange(triCountToCompute, RAYCOUNT), cl::NullRange);
		}
		std::cout << "intersection enqueued." << std::endl;

		//////
		const size_t aligned_FaceSDFCount = static_cast<size_t>(std::ceil(static_cast<float>(aligned_FCount) / 16.0f) * 16.0f);
		std::vector<cl_float> cl_FaceSDF(aligned_FaceSDFCount, 0.0f);
		cl::Buffer buffer_FaceSDF(context, (CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR), sizeof(cl_float) * cl_FaceSDF.size(), cl_FaceSDF.data());
		cl::Kernel computeSDF_partial(program, "computeSDF_partial");
		for (size_t offset = 0; offset < static_cast<size_t>(F.rows()); offset += aligned_chunkSize)
		{
			const size_t alignedCount = std::min((aligned_FCount - offset), aligned_chunkSize);
			const size_t triCountToCompute = std::min((F.rows() - offset), aligned_chunkSize);

			const cl_buffer_region t_partial{offset * aligned_RayCount * sizeof(cl_float), alignedCount * aligned_RayCount * sizeof(cl_float)};
			const cl_buffer_region sdf_partial{offset * sizeof(cl_float), alignedCount * sizeof(cl_float)};
			cl::Buffer buffer_T_partial = buffer_T.createSubBuffer((CL_MEM_READ_WRITE), CL_BUFFER_CREATE_TYPE_REGION, &t_partial);
			cl::Buffer buffer_FaceSDF_partial = buffer_FaceSDF.createSubBuffer((CL_MEM_WRITE_ONLY), CL_BUFFER_CREATE_TYPE_REGION, &sdf_partial);
			computeSDF_partial.setArg(0, buffer_SW);
			computeSDF_partial.setArg(1, buffer_T_partial);
			computeSDF_partial.setArg(2, static_cast<int>(RAYCOUNT));
			computeSDF_partial.setArg(3, aligned_RayCount);
			computeSDF_partial.setArg(4, buffer_FaceSDF_partial);

			queue.enqueueNDRangeKernel(
				computeSDF_partial, cl::NullRange, cl::NDRange(triCountToCompute, 1), cl::NullRange);
		}
		queue.finish();

		cl_float *result;
		result = static_cast<cl_float *>(queue.enqueueMapBuffer(buffer_FaceSDF, CL_TRUE, CL_MAP_READ, 0, sizeof(cl_float) * cl_FaceSDF.size()));

		FaceSDF.resize(F.rows(), 1);
		for (int f = 0; f < F.rows(); ++f)
		{
			FaceSDF(f, 0) = result[f];
		}
		queue.enqueueUnmapMemObject(buffer_FaceSDF, result);

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
	}
	catch (cl::Error err)
	{
		std::cerr << "ERROR: " << err.what() << "(" << err.err() << ")" << std::endl;
	}
	return;
}
