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
__device__ __forceinline__ void depth_to_meters(ushort raw_depth, double& depth)
{
    static constexpr auto DEPTH_DIVISOR = 255.0 / 10.0; //depth range from 0 to 10, scaled from 0 to 255
    depth = (raw_depth == 255) ? 0 : raw_depth / DEPTH_DIVISOR;
}

#else

// kinect v2
inline double depth_to_meters(ushort raw_depth)
{
    static constexpr auto DEPTH_DIVISOR = 255.0 / 10.0; //depth range from 0 to 10, scaled from 0 to 255
    return (raw_depth == 255) ? 0 : raw_depth / DEPTH_DIVISOR;
}

#endif

} // octomap_depth_mapping

#endif // DEPTH_CONVERSIONS_HPP
