#include <cuda.h>
#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/host_vector.h>
#include <thrust/zip_function.h>
#include <cstdio>
#include <ax/math/common.hpp>

using real = double;

__global__ void GpuAddKernel(const int num, real* x, real* y) {
    const int thread_grid_idx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    const int num_threads_in_grid = static_cast<int>(blockDim.x * gridDim.x);
    for (int i = thread_grid_idx; i < num; i += num_threads_in_grid) y[i] += x[i];
}

int main() {
  // Test whether cuda is available.
  int count;
  cudaError_t err = cudaGetDeviceCount(&count);
  if (err == cudaErrorNoDevice) {
    printf("No CUDA device found.\n");
  } else if (err == cudaErrorInsufficientDriver) {
    printf("CUDA driver is insufficient.\n");
  } else if (err == cudaErrorNoDevice) {
    printf("CUDA device is not available.\n");
  } else {
    printf("CUDA is available.\n");
  }

  printf("CUDA device count: %d\n", count);
  int runtime_version;
  cudaRuntimeGetVersion(&runtime_version);
  printf("CUDA runtime version: %d\n", runtime_version);

  int driver_version;
  cudaDriverGetVersion(&driver_version);
  printf("CUDA driver version: %d\n", driver_version);

  real *x, *y;
  err = cudaMalloc(&x, 100 * sizeof(real));
  if (err != cudaSuccess) {
    printf("Failed to allocate memory for x.\n");
    return 1;
  }
  err = cudaMalloc(&y, 100 * sizeof(real));
  if (err != cudaSuccess) {
    printf("Failed to allocate memory for y.\n");
    return 1;
  }
  GpuAddKernel<<<1, 32>>>(100, x, y);
  cudaDeviceSynchronize();


  using namespace ax::math;
  thrust::host_vector<vec2r> h(102400);
  for (int i = 0; i < 102400; ++i) h[i] .setRandom();
  thrust::device_vector<vec2r> a = h;
  thrust::device_vector<vec2r> b = h;
  thrust::transform(
      thrust::make_zip_iterator(thrust::make_tuple(a.begin(), b.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(a.end(), b.end())),
      a.begin(),
      thrust::make_zip_function(thrust::plus<vec2r>()));
  cudaDeviceSynchronize();

  printf("Done.\n");
  return 0;
}
