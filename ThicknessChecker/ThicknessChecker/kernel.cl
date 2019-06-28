OCLSTRINGIFY(
    bool rayBoxIntersection(
        const float4 orig,
        const float4 dir,
        const float4 bb_min,
        const float4 bb_max) {
        // This should be precomputed and provided as input
        float4 inv_dir = (float4)(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z, 1.0f);
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
    } float rayTriangleIntersection(const float4 orig,
                                    const float4 dir,
                                    const float4 vert0,
                                    const float4 vert1,
                                    const float4 vert2) {
        float t, u, v;
        float4 edge1, edge2, tvec, pvec, qvec;
        float det, inv_det;

        /* find vectors for two edges sharing vert0 */
        edge1 = vert1 - vert0;
        edge2 = vert2 - vert0;

        /* begin calculating determinant - also used to calculate U parameter */
        pvec = cross(dir, edge2);

        /* if determinant is near zero, ray lies in plane of triangle */
        det = dot(edge1, pvec);

        if (det > 0.000001)
        {
            /* calculate distance from vert0 to ray origin */
            tvec = orig - vert0;

            /* calculate U parameter and test bounds */
            u = dot(tvec, pvec);
            if (u < 0.0f || u > det)
                return -1.0f;

            /* prepare to test V parameter */
            qvec = cross(tvec, edge1);

            /* calculate V parameter and test bounds */
            v = dot(dir, qvec);
            if (v < 0.0f || u + v > det)
                return -1.0f;
        }
        else if (det < -0.000001)
        {
            /* calculate distance from vert0 to ray origin */
            tvec = orig - vert0;

            /* calculate U parameter and test bounds */
            u = dot(tvec, pvec);
            /*      printf("*u=%f\n",(float)*u); */
            /*      printf("det=%f\n",det); */
            if (u > 0.0f || u < det)
                return -1.0f;

            /* prepare to test V parameter */
            qvec = cross(tvec, edge1);

            /* calculate V parameter and test bounds */
            v = dot(dir, qvec);
            if (v > 0.0f || u + v < det)
                return -1.0f;
        }
        else
            return -1.0f; /* ray is parallel to the plane of the triangle */

        inv_det = 1.0f / det;

        /* calculate t, ray intersects triangle */
        t = dot(edge2, qvec) * inv_det;
        //u *= inv_det;
        //v *= inv_det;

        return t;
    }

    float rayMeshIntersections(const float4 orig,
                               const float4 dir,
                               __constant float4 *V,
                               __constant int4 *F,
                               __constant float4 *FN,
                               __constant float4 *bb_min,
                               __constant float4 *bb_max,
                               const int BBCount,
                               __constant int *elements) {
        float min_t = -1.0;
        int currentIdx = 0;
        // do traverse...
        while (currentIdx < BBCount)
        {
            // check ray-box intersection (currentIdx)
            if (rayBoxIntersection(orig, dir, bb_min[currentIdx], bb_max[currentIdx]))
            {
                // check this box is leaf or non-leaf
                if (elements[currentIdx] < 0)
                {
                    // non-leaf: traverse
                    currentIdx = currentIdx * 2 + 1;
                }
                else
                {
                    // leaf: ray-triangle intersection
                    // ignore false intersection (see original paper)
                    // as bonus, intersection with itself is also ignored.
                    if (dot(FN[elements[currentIdx]], dir) > 0.0f)
                    {
                        const float4 vert0 = V[F[elements[currentIdx]].x];
                        const float4 vert1 = V[F[elements[currentIdx]].y];
                        const float4 vert2 = V[F[elements[currentIdx]].z];
                        float t = rayTriangleIntersection(orig, dir, vert0, vert1, vert2);
                        if (t > 0.0 && (t < min_t || min_t < 0.0))
                        {
                            min_t = t;
                        }
                    }
                    // then move to
                    // sibling (odd nodes)
                    // sibling of the parent, grand parent, ... (even nodes)
                    while (currentIdx % 2 == 0 && currentIdx != 0)
                    {
                        currentIdx = (currentIdx - 1) / 2;
                    }
                    if (currentIdx == 0)
                    {
                        break;
                    }
                    currentIdx = currentIdx + 1;
                }
            }
            else
            {
                // then move to
                // sibling (odd nodes)
                // sibling of the parent, grand parent, ... (even nodes)
                while (currentIdx % 2 == 0 && currentIdx != 0)
                {
                    currentIdx = (currentIdx - 1) / 2;
                }
                if (currentIdx == 0)
                {
                    break;
                }
                currentIdx += 1;
            }
        }
        return min_t;
    } __kernel void computeIntersection_partial(__constant float4 *V,
                                                __constant int4 *F,
                                                __constant float4 *FN,
                                                __constant float4 *SW,
                                                __constant float4 *bb_min,
                                                __constant float4 *bb_max,
                                                __constant int *elements,
                                                const int offset,
                                                const int alignedRayCount,
                                                const int BBCount,
                                                __global float *T_partial) {
        int triIdx = get_global_id(0);
        int rayIdx = get_global_id(1);
        float4 source;
        float4 v0 = V[F[triIdx + offset].x];
        float4 v1 = V[F[triIdx + offset].y];
        float4 v2 = V[F[triIdx + offset].z];
        source = (v0 + v1 + v2) / 3.0f;

        float4 dir = -FN[triIdx + offset];

        float4 base1 = v0 - source;

        float norm2_1 = dot(base1, base1);
        float norm_1 = sqrt(norm2_1);
        base1 /= norm_1;
        float4 base2 = cross(dir, base1);
        base1 *= SW[rayIdx].x;
        base2 *= SW[rayIdx].y;

        dir += base1;
        dir += base2;

        float norm2_d = dot(dir, dir);
        float norm_d = sqrt(norm2_d);
        dir /= norm_d;

        // do computation
        float t = rayMeshIntersections(source, dir, V, F, FN, bb_min, bb_max, BBCount, elements);

        // write-back
        T_partial[triIdx * alignedRayCount + rayIdx] = t;
    } __kernel void computeSDF_partial(__constant float4 *SW,
                                       __global float *T_partial,
                                       const int RayCount,
                                       const int alignedRayCount,
                                       __global float *FaceSDF_partial) {
        int triIdx = get_global_id(0);
        const float number_of_mad = 1.5f;

        float avg = 0.0f;
        float validCount = 0.0f;
        for (int r = 0; r < RayCount; ++r)
        {
            if (T_partial[triIdx * alignedRayCount + r] >= 0.0f)
            {
                avg += T_partial[triIdx * alignedRayCount + r];
                validCount += 1.0f;
            }
        }
        if (validCount < 0.5f)
        {
            FaceSDF_partial[triIdx] = -1.0f;
            return;
        }
        avg /= validCount;

        // use standard deviation and ratio of 1.5; (different from CGAL (median absolute))
        float standardDeviation = 0.0f;
        float acceptCount = 0.0f;
        for (int r = 0; r < RayCount; ++r)
        {
            if (T_partial[triIdx * alignedRayCount + r] >= 0.0f)
            {
                float diff = T_partial[triIdx * alignedRayCount + r] - avg;
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
            for (int r = 0; r < RayCount; ++r)
            {
                if (T_partial[triIdx * alignedRayCount + r] >= 0.0f)
                {
                    const float deviation = fabs(T_partial[triIdx * alignedRayCount + r] - avg);
                    if (deviation <= number_of_mad * standardDeviation)
                    {
                        weightedSDF += (SW[r].z * T_partial[triIdx * alignedRayCount + r]);
                        weightSum += SW[r].z;
                    }
                }
            }
            if (weightSum > 0.0f)
            {
                FaceSDF_partial[triIdx] = weightedSDF / weightSum;
            }
            else
            {
                FaceSDF_partial[triIdx] = -1.0f;
            }
        }
        else
        {
            FaceSDF_partial[triIdx] = -1.0f;
        }
    })
