// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include "ThicknessChecker.h"

#include "igl/per_face_normals.h"
#include "igl/AABB.h"

#include <amp.h>
#include <amp_graphics.h>

////
// implementation
////
#define RAYCOUNT (25)

bool rayBoxIntersection(
	const concurrency::graphics::float_3& orig,
	const concurrency::graphics::float_3& dir,
	const concurrency::graphics::float_3& bb_min,
	const concurrency::graphics::float_3& bb_max
) restrict(amp)
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

float rayMeshIntersections(
	const concurrency::graphics::float_3& orig,
	const concurrency::graphics::float_3& dir,
	const concurrency::array_view<concurrency::graphics::float_3, 1>& bb_mins,
	const concurrency::array_view<concurrency::graphics::float_3, 1>& bb_maxs,
	const concurrency::array_view<int, 1>& elements,
	const concurrency::array_view<concurrency::graphics::float_3, 1>& V,
	const concurrency::array_view<concurrency::graphics::int_3, 1>& F
) restrict(amp)
{
	float min_t = -1.0;
	concurrency::index<1> currentIdx(0);
	// do traverse...
	while (unsigned(currentIdx[0]) < bb_mins.get_extent().size())
	{
		// check ray-box intersection (currentIdx)
		if (rayBoxIntersection(orig, dir, bb_mins[currentIdx], bb_maxs[currentIdx]))
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
				// todo
				float t = 0.0f; // todo
				if (t < min_t)
				{
					min_t = t;
				}

				// then move to
				// sibling (odd nodes)
				// sibling of the parent, grand parent, ... (even nodes)
				while (currentIdx[0] % 2 == 0 && currentIdx[0] != 0)
				{
					currentIdx[0] = (currentIdx[0] - 1) / 2;
				}
				if (currentIdx[0] == 0) {
					break;
				}
				currentIdx += 1;
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
			if (currentIdx[0] == 0) {
				break;
			}
			currentIdx += 1;
		}
	}
	return min_t;
}

float rayTriangleIntersection(
	float pos[3],
	float dir[3],
	float v0[3],
	float v1[3],
	float v2[3]
) restrict(amp);

float computeSDF(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& V,
	const Eigen::Matrix<  int, Eigen::Dynamic, Eigen::Dynamic>& F
)
{
	//////
	// setup buffers for parallel computation
	std::vector< concurrency::graphics::float_3 > V_vec(V.rows());
	for (int v = 0; v < V.rows(); ++v)
	{
		V_vec.at(v) = concurrency::graphics::float_3(V(v, 0), V(v, 1), V(v, 2));
	}
	concurrency::array_view<concurrency::graphics::float_3, 1> AMP_V(int(V.rows()), V_vec);

	std::vector<concurrency::graphics::int_3> F_vec(F.rows());
	for (int f = 0; f < F.rows(); ++f)
	{
		F_vec.at(f) = concurrency::graphics::int_3(F(f, 0), F(f, 1), F(f, 2));
	}
	concurrency::array_view<concurrency::graphics::int_3, 1> AMP_F(int(F.rows()), F_vec);

	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> FN;
	igl::per_face_normals(V, F, FN);
	std::vector<concurrency::graphics::float_3> FN_vec(int(FN.rows()));
	for (int fn = 0; fn < FN.rows(); ++fn)
	{
		FN_vec.at(fn) = concurrency::graphics::float_3(FN(fn, 0), FN(fn, 1), FN(fn, 2));
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
	// construct serialized AABB todo.
	std::vector<concurrency::graphics::float_3> bb_mins_vec(int(bb_mins.rows()));
	std::vector<concurrency::graphics::float_3> bb_maxs_vec(int(bb_maxs.rows()));
	std::vector<int> elements_vec(int(elements.rows()));
	for (int bb = 0; bb < bb_mins.rows(); ++bb)
	{
		bb_mins_vec.at(bb) = concurrency::graphics::float_3(bb_mins(bb, 0), bb_mins(bb, 1), bb_mins(bb, 2));
		bb_maxs_vec.at(bb) = concurrency::graphics::float_3(bb_maxs(bb, 0), bb_maxs(bb, 1), bb_maxs(bb, 2));
		elements_vec.at(bb) = elements(bb, 0);
	}
	concurrency::array_view<concurrency::graphics::float_3, 1> AMP_bb_mins(int(bb_mins.rows()), bb_mins_vec);
	concurrency::array_view<concurrency::graphics::float_3, 1> AMP_bb_maxs(int(bb_maxs.rows()), bb_maxs_vec);
	concurrency::array_view<int, 1> AMP_elements(int(elements.rows()), elements_vec);
	//////

	//////
	// #triangle x #ray array. compute ray-mesh intersection and write to this vector in parallel
	std::vector<float> T_vec(F.rows() * RAYCOUNT, 0.0f);
	concurrency::array_view<float, 2> AMP_T(int(F.rows()), RAYCOUNT, T_vec);
	//////

	concurrency::parallel_for_each(
		AMP_T.extent,
		[=](concurrency::index<2> idx) restrict(amp) {
		// source point
		concurrency::graphics::float_3 source(0.0f, 0.0f, 0.0f);
		source =
			AMP_V[concurrency::index<1>(AMP_F[concurrency::index<1>(idx[0])].x)]
			+ AMP_V[concurrency::index<1>(AMP_F[concurrency::index<1>(idx[0])].y)]
			+ AMP_V[concurrency::index<1>(AMP_F[concurrency::index<1>(idx[0])].z)];

		// generate dir (use pre-defined??)
		concurrency::graphics::float_3 dir(0.0f, 0.0f, 0.0f);

		// do computation
		float t = rayMeshIntersections(source, dir, AMP_bb_mins, AMP_bb_maxs, AMP_elements, AMP_V, AMP_F);

		// write-back
		AMP_T[idx] = t;
	});
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

float debug(
	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& V,
	const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& F
)
{
	igl::AABB<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, 3> aabb;

	aabb.init(V, F);
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> bb_mins;
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> bb_maxs;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> elements;
	aabb.serialize(bb_mins, bb_maxs, elements);

	std::cout << "bb_mins" << std::endl;
	std::cout << bb_mins << std::endl << std::endl;
	std::cout << "bb_maxs" << std::endl;
	std::cout << bb_maxs << std::endl << std::endl;
	std::cout << "elements: " << elements.rows() << std::endl;
	std::cout << elements << std::endl << std::endl;

	return 1.0f;
}