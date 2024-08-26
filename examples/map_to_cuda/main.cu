#include <cuda_runtime.h>
#include <openvdb/math/Vec3.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include <ax/core/init.hpp>
#include <ax/math/accessor_gpu.cuh>

#include "ax/core/logging.hpp"

using namespace ax::math;

__global__ void read_from_span(Span<const float> buffer) {
  int x = threadIdx.x + blockIdx.x * blockDim.x;
  if (x < buffer.size()) {
    printf("buffer[%d] = %f\n", x, buffer[x]);
  }
}


int main(int argc, char** argv) {
  ax::initialize(argc, argv);

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

  cudaPitchedPtr pitched;
  size_t width = 9, height = 10, depth = 20;


  auto extent = make_cudaExtent(9 * sizeof(float), 10, 20);
  cudaMalloc3D(&pitched, extent);
  auto shape = make_shape<size_t>(depth, height, width);

  ax::PitchedBuffer<float> buf(pitched.ptr, pitched.pitch, pitched.pitch * 10 * 20);
  auto access_buf = make_accessor(buf, shape);

  char* devPtr = static_cast<char*>(pitched.ptr);
  size_t pitch = pitched.pitch;
  for (int z = 0; z < depth; ++z) {
    char* slice = devPtr + z * height * pitch;
    for (int y = 0; y < height; ++y) {
      auto *row = reinterpret_cast<float*>(slice + y * pitch);
      for (int x = 0; x < width; ++x) {
        // !!! Do not load the value because it is on the device.
        float* from_accessor = &access_buf(z, y, x);
        float* from_row = &row[x];
        AX_CHECK(from_accessor == from_row, "from_accessor: {}, from_row: {}", fmt::ptr(from_accessor), fmt::ptr(from_row));
      }
    }
  }

  cudaFree(pitched.ptr);
  cudaDeviceSynchronize();
  return EXIT_SUCCESS;
}
