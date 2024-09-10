#include "ax/core/buffer/device_buffer.cuh"
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/init.hpp"
#include <thrust/host_vector.h>
#include <vector>
using namespace ax;

__global__ void kern_set(BufferView<int> view) {
  unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
  view(x, y) = y * view.Shape().X() + x;
}

__global__ void kern_check(BufferView<int> view) {
  unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
  unsigned int linear = y * view.Shape().X() + x;

  if (view(x, y) != linear) {
    printf("Error at (%d, %d): %d != %d\n", x, y, view(x, y), linear);
  }
}

int main(int argc, char **argv) {
  initialize(argc, argv);

  AX_INFO("Test raw created buffer.");
  auto bp = DeviceBufferPitched<int>::Create({10, 10});
  auto [x, y, z] = *(bp->Stride());
  AX_INFO("stride: {}, {}, {}", x, y, z);
  dim3 block(1, 1);
  dim3 grid(10, 10);
  kern_set<<<grid, block>>>(bp->View());
  kern_check<<<grid, block>>>(bp->View());
  cudaDeviceSynchronize();


  AX_INFO("Test thrust created buffer.");
  auto bp2 = DeviceBuffer<int>::Create({10, 10});
  auto& bp2_data = bp2->GetUnderlying();
  thrust::host_vector<int> host_data(100);
  for (int i = 0; i < 100; i++) {
    host_data[i] = i;
  }
  bp2_data = host_data;
  kern_check<<<grid, block>>>(bp2->View());
  cudaDeviceSynchronize();

  clean_up();
  return 0;
}