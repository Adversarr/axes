#include <cuda_runtime.h>
#include <openvdb/math/Vec3.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include <ax/math/gpu_accessor.cuh>

#include <ax/core/init.hpp>

using namespace ax::math;

__global__ void read_from_span(Span<const float> buffer) {
  int x = threadIdx.x + blockIdx.x * blockDim.x;
  if (x < buffer.size()) {
    printf("buffer[%d] = %f\n", x, buffer[x]);
  }
}


int main(int argc, char** argv) {
  ax::init(argc, argv);

  thrust::host_vector<float> host(9 * 10);
  for (int i = 0; i < host.size(); i++) {
    host[i] = i;
  }
  thrust::device_vector<float> device = host;
  cudaDeviceSynchronize();
  ax::clean_up();
  openvdb::math::Vec3d v3d;
  constexpr size_t size = 90;

  float* buffer = thrust::raw_pointer_cast(device.data());
  Span<float> chk = Span(buffer, size);

  read_from_span<<<10, 10>>>(chk);

  cudaThreadSynchronize();
  return EXIT_SUCCESS;
}