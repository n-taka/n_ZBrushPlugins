// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include "ThicknessChecker.h"

#include <amp.h>

////
// implementation
////
#define RAYCOUNT (25)

float computeSDF(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& V,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& F
)
{
	std::vector<float> V_vec(V.rows() * V.cols());
	for (int v = 0; v < V.rows(); ++v)
	{
		V_vec.at(v * 3 + 0) = V(v, 0);
		V_vec.at(v * 3 + 1) = V(v, 1);
		V_vec.at(v * 3 + 2) = V(v, 2);
	}
	concurrency::array_view<float, 2> AMP_V(V.rows(), V.cols(), V_vec);
	std::vector<int> F_vec(F.rows() * F.cols());
	for (int f = 0; f < F.rows(); ++f)
	{
		F_vec.at(f * 3 + 0) = F(f, 0);
		F_vec.at(f * 3 + 1) = F(f, 1);
		F_vec.at(f * 3 + 2) = F(f, 2);
	}
	concurrency::array_view<int, 2> AMP_F(F.rows(), F.cols(), F_vec);

	// #triangle x #ray array. compute ray-mesh intersection and write to this vector in parallel
	std::vector<float> T_vec(F.rows() * RAYCOUNT, 0.0f);
	concurrency::array_view<float, 2> AMP_T(F.rows(), RAYCOUNT, T_vec);

	////
	concurrency::parallel_for_each(
		AMP_T.extent.tile<1, RAYCOUNT>(),
		[=](concurrency::tiled_index<1, RAYCOUNT> idx) restrict(amp) {
		// source point
		const int fIdx = idx.tile[0];
		float source[3] = { 0.0f, 0.0f, 0.0f };
		for (int v = 0; v < 3; ++v)
		{
			for (int xyz = 0; xyz < 3; ++xyz)
			{
				source[xyz] += AMP_V[concurrency::index<2>(AMP_F[concurrency::index<2>(fIdx, v)], xyz)];
			}
		}

		// generate dir (use pre-defined??)

		// do computation
		float t = rayMeshIntersection(AMP_V, AMP_F);

		// write-back
		AMP_T[idx.global] = t;
	});
}


// maybe change...
// use AABB
float rayMeshIntersection(
	concurrency::array_view<float, 2> V,
	concurrency::array_view<  int, 2> F
) restrict(amp)
{
	return 0.0f;
}


#define IGL_RAY_TRI_EPSILON 0.000001
#define IGL_RAY_TRI_CROSS(dest,v1,v2) \
          dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
          dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
          dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
#define IGL_RAY_TRI_DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
#define IGL_RAY_TRI_SUB(dest,v1,v2) \
          dest[0]=v1[0]-v2[0]; \
          dest[1]=v1[1]-v2[1]; \
          dest[2]=v1[2]-v2[2]; 

float rayTriangleIntersection(
	float orig[3],
	float dir[3],
	float vert0[3],
	float vert1[3],
	float vert2[3]) restrict(amp)
{
	float t, u, v;
	float edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
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
	else return -1.0f;  /* ray is parallel to the plane of the triangle */


	inv_det = 1.0f / det;

	/* calculate t, ray intersects triangle */
	t = IGL_RAY_TRI_DOT(edge2, qvec) * inv_det;
	//u *= inv_det;
	//v *= inv_det;

	return t;
}
