#include <cuda.h>
#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/host_vector.h>
#include <thrust/sequence.h>
#include <thrust/async/for_each.h>
#include <thrust/zip_function.h>

#include <ax/core/init.hpp>
#include <ax/math/accessor.hpp>
#include <complex>
#include <cstdio>

#include <cublas.h>
template <typename T> auto arg(T x) { return std::arg(x); }


using namespace ax;
using namespace ax::math;

using Real = double;

__global__ void GpuAddKernel(const int num, Real* x, Real* y) {
  const int thread_grid_Index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int num_threads_in_grid = static_cast<int>(blockDim.x * gridDim.x);
  for (int i = thread_grid_Index; i < num; i += num_threads_in_grid) y[i] += x[i];
}

__global__ void gpu_test_accessor(FieldAccessor<Real, 1> accessor) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (accessor->IsValidSub(i)) {
    printf("%d %lf\n", i, accessor(i));
  }
}

int main(int argc, char** argv) {
  // Test whether cuda is available.
  initialize(argc, argv);
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

  cudaDeviceProp props;
  cudaGetDeviceProperties_v2(&props, 0);
  printf("Device name: %s\n", props.name);

  Real *x, *y;
  err = cudaMalloc((void**)&x, 100 * sizeof(Real));
  if (err != cudaSuccess) {
    printf("Failed to allocate memory for x.\n");
    return 1;
  }
  err = cudaMalloc((void**)&y, 100 * sizeof(Real));
  if (err != cudaSuccess) {
    printf("Failed to allocate memory for y.\n");
    return 1;
  }
  GpuAddKernel<<<1, 32>>>(100, x, y);
  cudaDeviceSynchronize();

  cudaPitchedPtr p;
  cudaExtent extent = make_cudaExtent(100 * sizeof(Real), 1, 1);

  thrust::host_vector<RealVector2> h(102400);
  for (int i = 0; i < 102400; ++i) h[i].setRandom();
  thrust::device_vector<RealVector2> a = h;
  thrust::device_vector<RealVector2> b = h;
  thrust::transform(thrust::make_zip_iterator(thrust::make_tuple(a.begin(), b.begin())),
                    thrust::make_zip_iterator(thrust::make_tuple(a.end(), b.end())), a.begin(),
                    thrust::make_zip_function(thrust::plus<RealVector2>()));
  cudaDeviceSynchronize();

  constexpr int dim = 3;

  thrust::device_vector<Index> seq_;
  thrust::device_vector<math::IndexVector<dim + 1>> elements_;
  thrust::device_vector<math::RealMatrix<dim, dim>> deformation_gradient_;
  thrust::device_vector<math::RealMatrix<dim, dim>> rinv_gpu_;
  thrust::device_vector<Real> rest_volume_gpu_;

  HostFieldData<Real> host_field_data(24);
  for (int i = 0; i < 24; ++i) host_field_data[i] = i;
  DeviceFieldData<Real> device_field_data = host_field_data;
  auto accessor = math::make_accessor(device_field_data);
  thrust::for_each(device_field_data.begin(), device_field_data.end(),
                   [] __device__(Real & x) { x += 1; });
  gpu_test_accessor<<<16, 1>>>(accessor);

  seq_.resize(100);
  elements_.resize(100);
  deformation_gradient_.resize(100);
  rinv_gpu_.resize(100);
  rest_volume_gpu_.resize(100);

  thrust::sequence(thrust::device, seq_.begin(), seq_.end());


  printf("Done.\n");
  return 0;
}
