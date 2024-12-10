#pragma once
#ifndef DEPTH_CONVERSIONS_HPP
#define DEPTH_CONVERSIONS_HPP

#ifdef CUDA
#include <cuda_runtime.h>
#endif

namespace octomap_depth_mapping
{

#ifdef CUDA

// kinect v2 cuda
__device__ __forceinline__ void depth_to_meters(ushort raw_depth, double& depth, const double max_distance)
{
    const auto DEPTH_DIVISOR = 255.0 / max_distance; //depth range from 0 to max_distance, scaled from 0 to 255
    depth = raw_depth / DEPTH_DIVISOR;
}

#else

inline double depth_to_meters(ushort raw_depth, double max_distance)
{
    static const auto DEPTH_DIVISOR = 255.0 / max_distance; //depth range from 0 to max_distance, scaled from 0 to 255
    return raw_depth / DEPTH_DIVISOR;
}

#endif

} // octomap_depth_mapping

#endif // DEPTH_CONVERSIONS_HPP
