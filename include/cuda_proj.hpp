#pragma once
#ifndef CUDA_PROJ_HPP
#define CUDA_PROJ_HPP

#include <cuda_runtime.h>

#include <stdint.h>

namespace octomap_depth_mapping
{

void project_depth_img(uint8_t*, double*, int, int,
    dim3, dim3,
    double, double, double, double,
    double, double, double,
    double, double, double,
    double, double, double,
    double, double, double);

__global__ void project_kernel(uint8_t*, double*, int, int,
    double, double, double, double,
    double, double, double,
    double, double, double,
    double, double, double,
    double, double, double);

} // octomap_depth_mapping

#endif // CUDA_PROJ_HPP
