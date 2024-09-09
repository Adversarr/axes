#include <imgui.h>
#include <thrust/device_vector.h>
#include <cuda_runtime.h>
#include <ax/gl/colormap.hpp>
#include <ax/gl/events.hpp>
#include <ax/gl/extprim/grid.hpp>
#include <ax/gl/primitives/lines.hpp>
#include "ax/fem/topo.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/gl/utils.hpp"
#include "ax/math/common.hpp"
#include "ax/math/utils/formatting.hpp"
#include "ax/utils/time.hpp"
using namespace ax;
#define BLOCK_DIM_X_IN_USE 128

struct MassSpringGpu {
  // Define the mass spring system on GPU.
  thrust::device_vector<math::FloatVector3> positions_;
  thrust::device_vector<math::FloatVector3> last_positions_;
  thrust::device_vector<math::FloatVector3> inertia_position_;
  thrust::device_vector<math::FloatVector3> temporary_position_;
  thrust::device_vector<Float> masses_;
  thrust::device_vector<math::FloatVector3> external_forces_;
  thrust::device_vector<bool> is_fixed_;
  thrust::device_vector<Float> weights_;

  thrust::device_vector<math::IndexVector2> springs_;
  thrust::device_vector<Float> stiffness_;
  thrust::device_vector<Float> original_lengths_;

  thrust::device_vector<math::FloatMatrix<3, 2>> z_;
  thrust::device_vector<math::FloatMatrix<3, 2>> u_;

  Float dt_ = 1e-2f;
  bool is_updated_ = true;
  Entity render_entity_;

  int max_iterations_ = 10;
};

void run_admm_once(MassSpringGpu& data);

void ui_callback(gl::UiRenderEvent) {
  auto& ms = get_resource<MassSpringGpu>();
  bool need_run = false;
  if (ImGui::Begin("Console")) {
    static bool is_running = false;
    need_run |= ImGui::Button("Run Once");
    ImGui::Checkbox("Run Continuously", &is_running);
    ImGui::InputInt("Iteration", &ms.max_iterations_);
    need_run |= is_running;
  }
  ImGui::End();

  if (need_run) {
    auto start = utils::now();
    run_admm_once(ms);
    auto end = utils::now();
    AX_INFO("Time elapsed: {}", end - start);
    ms.is_updated_ = true;
  }

  if (ms.is_updated_) {
    patch_component<gl::Lines>(ms.render_entity_, [&ms](gl::Lines& lines) {
      math::FloatMatrix<3, math::dynamic> vertices(3, ms.positions_.size());
      cudaMemcpy(vertices.data(), thrust::raw_pointer_cast(ms.positions_.data()),
                 ms.positions_.size() * 3 * sizeof(Float), cudaMemcpyDeviceToHost);
      lines.vertices_ = vertices.cast<Real>();
    });

    ms.is_updated_ = false;
  }
}

__global__ void compute_rest_length(math::FloatVector3* positions, math::IndexVector2* springs,
                                    Float* rest_lengths, size_t num_springs) {
  // TODO.
  unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num_springs) {
    auto spring = springs[tid];
    auto x0 = positions[spring[0]], x1 = positions[spring[1]];
    auto diff = x0 - x1;
    rest_lengths[tid] = diff.norm();
  }
}

__global__ void mark_fixed(bool* is_fixed,
  math::FloatVector3 const* positions,
  size_t num_vertices) {
  unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num_vertices) {
    if (positions[tid].x() > 1 - 1e-3) {
      is_fixed[tid] = true;
    }
  }
}

int main(int argc, char** argv) {
  // Run a large scale mass spring on GPU directly.
  po::add_option({
    po::make_option<int>("resolution", "Resolution of mass spring", "10"),
  });
  gl::init(argc, argv);

  // Initialize the mass spring system.
  int resolution = po::get_parse_result()["resolution"].as<int>();
  gl::prim::Grid g(resolution, resolution);
  gl::Lines grid = g.Draw();
  grid.vertices_ *= 0.01;

  size_t num_vertices = static_cast<size_t>(grid.vertices_.cols());
  size_t num_springs = static_cast<size_t>(grid.indices_.cols());

  auto [fwd, bwd] = fem::optimize_topology<1>(grid.indices_, num_vertices);
  auto float_field = grid.vertices_.cast<Float>().eval();
  for (Index i = 0; i < num_vertices; ++i) {
    float_field.col(fwd[i]) = grid.vertices_.col(i).cast<Float>();
  }
  for (Index i = 0; i < num_springs; ++i) {
    for (Index d = 0; d < 2; ++d) {
      grid.indices_(d, i) = fwd[grid.indices_(d, i)];
    }
  }

  auto& msg = ensure_resource<MassSpringGpu>();
  msg.positions_.resize(num_vertices);

  cudaMemcpy(thrust::raw_pointer_cast(msg.positions_.data()), float_field.data(),
             num_vertices * 3 * sizeof(Float), cudaMemcpyHostToDevice);
  msg.last_positions_ = msg.positions_;
  msg.inertia_position_ = msg.positions_;
  msg.temporary_position_ = msg.positions_;
  msg.masses_.resize(num_vertices, 1);
  msg.external_forces_.resize(num_vertices, math::FloatVector3{0, -9.8, 0});
  msg.is_fixed_.resize(num_vertices, false);
  msg.weights_.resize(num_vertices, 0);

  size_t required_blocks = (num_springs + BLOCK_DIM_X_IN_USE - 1) / BLOCK_DIM_X_IN_USE;
  mark_fixed<<<required_blocks, BLOCK_DIM_X_IN_USE>>>(
      thrust::raw_pointer_cast(msg.is_fixed_.data()),
      thrust::raw_pointer_cast(msg.positions_.data()),
      num_vertices);
  msg.springs_.resize(num_springs);
  cudaMemcpy(thrust::raw_pointer_cast(msg.springs_.data()), grid.indices_.data(),
             num_springs * 2 * sizeof(Index), cudaMemcpyHostToDevice);

  msg.stiffness_.resize(num_springs, 3e4 * resolution);
  msg.original_lengths_.resize(num_springs);
  compute_rest_length<<<required_blocks, BLOCK_DIM_X_IN_USE>>>(thrust::raw_pointer_cast(msg.positions_.data()),
                                                 thrust::raw_pointer_cast(msg.springs_.data()),
                                                 thrust::raw_pointer_cast(msg.original_lengths_.data()),
                                                 num_springs);

  msg.z_.resize(num_springs, math::FloatMatrix<3, 2>::Zero());
  msg.u_.resize(num_springs, math::FloatMatrix<3, 2>::Zero());

  cudaDeviceSynchronize();

  if (auto err = cudaGetLastError(); err != cudaSuccess) {
    AX_CRITICAL("CUDA error: {}", cudaGetErrorString(err));
    abort();
  }

  // Renderer.
  msg.render_entity_ = create_entity();
  auto& l = add_component<gl::Lines>(msg.render_entity_, grid);

  gl::Colormap cmap(0, num_vertices);
  for (Index i = 0; i < num_vertices; ++i) {
    l.colors_.col(i).topRows<3>() = cmap(i);
  }

  // Register the UI callback.
  connect<gl::UiRenderEvent, &ui_callback>();
  return gl::enter_main_loop();
}

__device__ math::FloatMatrix<3, 2> prox(math::FloatVector3 const& x0, math::FloatVector3 const& x1,
                                        math::FloatMatrix<3, 2> const& u, Float stiffness,
                                        Float original_length, Float rho, bool is_x0_fixed,
                                        bool is_x1_fixed) {
  // Solves for:
  //   minimize_z 1/2 k (|z0 - z1| - l)^2 + 1/2 rho ||x-z+u||^2
  // Take the first order condition:
  //        k (|z0 - z1| - l) (z0 - z1)/s + rho (z0 - (x0 + u0)) = 0   <<<< (eq. 1)
  //        k (|z0 - z1| - l) (z1 - z0)/s + rho (z1 - (x1 + u1)) = 0   <<<< (eq. 2)
  // We can have a close form solution. (Use s to represent |z0 - z1|)
  // 1. sum eqs: center point
  //        z0 + z1 = x0 + x1 + u0 + u1
  // 2. difference
  //        [2 k (s - l) + rho s] (z0 - z1)/|z0 - z1| = rho ((x0 + u0) - (x1 + u1))
  //                                                  = rho xu
  //    and (z0 - z1)/|z0 - z1| is a unit vector, therefore
  //        2k (s - l) + rho s = rho |xu|
  //    we now have |z0 - z1| = (rho |xu| + 2k l) / (rho + 2k)
  // 3. the direction:
  //        z0 - z1 // xu
  //    and if [2 k (s - l) + rho s] > 0,
  //        i.e. (2k + rho) s > 2k l
  //        i.e. rho |xu| + 2kl > 2k l
  //        z0 - z1 = xu * s
  //    otherwise, the negative.
  math::FloatMatrix<3, 2> z;
  auto u0 = u.col(0), u1 = u.col(1);

  // if (!is_x0_fixed && !is_x1_fixed) {
  math::FloatVector3 xu = x0 - x1 + u0 - u1;
  Real strength = (rho * xu.norm() + 2 * stiffness * original_length) / (2 * stiffness + rho);
  math::FloatVector3 direction = xu.normalized();
  math::FloatVector3 z0_z1;

  z0_z1 = direction * strength;
  math::FloatVector3 center = 0.5 * (x0 + x1 + u0 + u1);
  z.col(0) = center + 0.5 * z0_z1;
  z.col(1) = center - 0.5 * z0_z1;
  // } else if (is_x0_fixed && !is_x1_fixed) {
  //   z.col(0) = x0;
  //   // We can eliminate most things.
  //   //    k (|z0 - z1| - l) (z1 - z0)/s + rho (x1 - z1 + u1) = 0   <<<< (eq. 2)
  //   // is what we need to solve, but we know z0 = x0.
  //   //    k (|x0 - z1| - l) (x0 - z1)/s + rho (x1 - z1 + u1) = 0
  //   // => k (|x0 - z1| - l) (x0 - z1)/s + rho (x0 - z1) = rho (x0 - x1 - u1)
  //   // => k (s - l) + rho s = rho |...|
  //   // => s = (rho |...| + kl) / (k + rho)
  //   // and z1 = x0 + (x0 - x1 - u1) * s / |x0 - x1 - u1|
  //   math::FloatVector3 xu = x0 - x1 - u1;
  //   Real strength = (rho * xu.norm() + stiffness * original_length) / (stiffness + rho);
  //   math::FloatVector3 direction = xu.normalized();
  //   z.col(1) = x0 - direction * strength;
  // } else if (is_x1_fixed && !is_x0_fixed) {
  //   // similar
  //   z.col(1) = x1;
  //   math::FloatVector3 xu = x1 - x0 - u0;
  //   Real strength = (rho * xu.norm() + stiffness * original_length) / (stiffness + rho);
  //   math::FloatVector3 direction = xu.normalized();
  //   z.col(0) = x1 - direction * strength;
  // } else {
  //   z.col(0) = x0;
  //   z.col(1) = x1;
  // }
  return z;
}

__global__ void compute_inertia_position(const math::FloatVector3* positions,
                                         const math::FloatVector3* last_positions,
                                         const math::FloatVector3* external_forces,
                                         math::FloatVector3* inertia_position, const bool* is_fixed,
                                         Float dt, size_t num_vertices) {
  unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num_vertices && !is_fixed[tid]) {
    inertia_position[tid]
        = 2 * positions[tid] - last_positions[tid] + external_forces[tid] * dt * dt / 2;
  }
}

__device__ void do_atomic_add(
    math::FloatVector3* weighted_position,
    Float* weights,
    const math::FloatVector3& value,
    Float weight) {
  atomicAdd(&weighted_position->x(), weight * value.x());
  atomicAdd(&weighted_position->y(), weight * value.y());
  atomicAdd(&weighted_position->z(), weight * value.z());
  atomicAdd(weights, weight);
}

__global__ void do_local_step(
    math::FloatVector3* weighted_position,
    math::FloatMatrix<3, 2> *z,
    Float* weights,
    const Float* mass,
    const math::FloatVector3* x,
    const math::FloatMatrix<3, 2>* u,
    const math::IndexVector2* springs,
    const Float* stiffness,
    const Float* original_lengths,
    const bool* is_fixed,
    size_t num_springs,
    Float dt) {
  unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num_springs) {
    auto spring = springs[tid];
    auto x0 = x[spring[0]], x1 = x[spring[1]];
    const Float l = original_lengths[tid];
    const Float k = stiffness[tid];
    // const Float rho = mass[spring[0]] / dt / dt;
    const Float rho = k;
    z[tid] = prox(x0, x1, u[tid], k, l, rho, is_fixed[spring[0]], is_fixed[spring[1]]);

    math::FloatMatrix<3, 2> zu = z[tid] - u[tid];
    const Float w = rho * dt * dt;
    do_atomic_add(&weighted_position[spring[0]], &weights[spring[0]], zu.col(0), w);
    do_atomic_add(&weighted_position[spring[1]], &weights[spring[1]], zu.col(1), w);
  }
}

__global__ void do_global_step(
  math::FloatVector3* result,
  math::FloatVector3* weighted_position,
  Float* weights,
  const math::FloatVector3* inertia_position,
  const Float* mass,
  const bool* is_fixed,
  size_t num_vertices) {
  unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num_vertices) {
    if (!is_fixed[tid]) {
      result[tid] = (weighted_position[tid] + inertia_position[tid] * mass[tid])
                    / (weights[tid] + mass[tid]);
    }

    weighted_position[tid].setZero();
    weights[tid] = 0;
  }
}

__global__ void do_complementary_step(
    const math::FloatVector3* x,
    math::FloatMatrix<3, 2>* u,
    const math::FloatMatrix<3, 2> *z,
    const math::IndexVector2* springs,
    size_t num_springs) {
  unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num_springs) {
    auto spring = springs[tid];
    auto& zi = z[tid];
    auto& ui = u[tid];
    const auto& x0 = x[spring[0]], x1 = x[spring[1]];
    ui.col(0) += x0 - zi.col(0);
    ui.col(1) += x1 - zi.col(1);
  }
}

void run_admm_once(MassSpringGpu& data) {
  size_t num_vertices = data.positions_.size();
  size_t num_springs = data.springs_.size();
  size_t num_block_launch_vert = (num_vertices + BLOCK_DIM_X_IN_USE - 1) / BLOCK_DIM_X_IN_USE;
  size_t num_block_launch_spring = (num_springs + BLOCK_DIM_X_IN_USE - 1) / BLOCK_DIM_X_IN_USE;

  compute_inertia_position<<<num_block_launch_vert, BLOCK_DIM_X_IN_USE>>>(
      thrust::raw_pointer_cast(data.positions_.data()),
      thrust::raw_pointer_cast(data.last_positions_.data()),
      thrust::raw_pointer_cast(data.external_forces_.data()),
      thrust::raw_pointer_cast(data.inertia_position_.data()),
      thrust::raw_pointer_cast(data.is_fixed_.data()), data.dt_, num_vertices);

  // backup the last position.
  cudaMemcpy(thrust::raw_pointer_cast(data.last_positions_.data()),
             thrust::raw_pointer_cast(data.positions_.data()),
             num_vertices * sizeof(math::FloatVector3), cudaMemcpyHostToHost);
  cudaMemcpy(raw_pointer_cast(data.positions_.data()),
             thrust::raw_pointer_cast(data.inertia_position_.data()),
             num_vertices * sizeof(math::FloatVector3), cudaMemcpyHostToHost);
  cudaMemset(thrust::raw_pointer_cast(data.weights_.data()), 0, num_vertices * sizeof(Float));
  cudaMemset(thrust::raw_pointer_cast(data.temporary_position_.data()), 0,
             num_vertices * 3 * sizeof(Float));

  // We use:
  //  temporary buffer to store the weighted_position
  //  position buffer to store the current solution.

  for (int i = 0; i < data.max_iterations_; ++i) {
    do_complementary_step<<<num_block_launch_spring, BLOCK_DIM_X_IN_USE>>>(
        thrust::raw_pointer_cast(data.positions_.data()),
        thrust::raw_pointer_cast(data.u_.data()),
        thrust::raw_pointer_cast(data.z_.data()),
        thrust::raw_pointer_cast(data.springs_.data()),
        num_springs);

    do_local_step<<<num_block_launch_spring, BLOCK_DIM_X_IN_USE>>>(
        thrust::raw_pointer_cast(data.temporary_position_.data()),
        thrust::raw_pointer_cast(data.z_.data()),
        thrust::raw_pointer_cast(data.weights_.data()),
        thrust::raw_pointer_cast(data.masses_.data()),
        thrust::raw_pointer_cast(data.positions_.data()),
        thrust::raw_pointer_cast(data.u_.data()),
        thrust::raw_pointer_cast(data.springs_.data()),
        thrust::raw_pointer_cast(data.stiffness_.data()),
        thrust::raw_pointer_cast(data.original_lengths_.data()),
        thrust::raw_pointer_cast(data.is_fixed_.data()),
        num_springs, data.dt_);

    do_global_step<<<num_block_launch_vert, BLOCK_DIM_X_IN_USE>>>(
        thrust::raw_pointer_cast(data.positions_.data()),
        thrust::raw_pointer_cast(data.temporary_position_.data()),
        thrust::raw_pointer_cast(data.weights_.data()),
        thrust::raw_pointer_cast(data.inertia_position_.data()),
        thrust::raw_pointer_cast(data.masses_.data()),
        thrust::raw_pointer_cast(data.is_fixed_.data()),
        num_vertices);
  }
}