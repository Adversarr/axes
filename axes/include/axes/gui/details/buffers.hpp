#pragma once
#include "axes/core/math/common.hpp"
#include "axes/gui/details/vkcontext.hpp"
#include "axes/gui/tags.hpp"
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

namespace axes::gui {
namespace details {
struct alignas(16) SceneUniform {
  glm::mat4 view_;
  glm::mat4 projection_;
  glm::vec3 eye_position_;
  glm::vec3 point_light_pos_;
  glm::vec4 point_light_color_;
  glm::vec3 parallel_light_dir_;
  glm::vec4 parallel_light_color_;
  glm::vec4 ambient_light_color_;
};

struct SceneVertex {
  glm::vec3 position_;
  glm::vec4 color_;
  glm::vec3 normal_;

  static std::vector<vk::VertexInputBindingDescription> GetBindingDescriptions();
  static std::vector<vk::VertexInputAttributeDescription>
  GetAttributeDescriptions();
};

struct MeshInstance {
  glm::vec3 position_;
  glm::vec4 rotation_;
  glm::vec4 color_;
  static std::vector<vk::VertexInputBindingDescription> GetBindingDescriptions();
  static std::vector<vk::VertexInputAttributeDescription>
  GetAttributeDescriptions();
};

struct MeshPushConstants {
  // 64B
  glm::mat4 model_;
  // 4 * 4B
  // Options[0] => use double side lighting
  // Options[1] => specular shineness
  // Options[2] => use pc color
  int options_[4] = {0, 32, 0, 0};
};

struct SceneRenderDataSharedGpu {
  VmaAllocBuffer vertex_buffer_;
  uint32_t vertex_count_;

  std::shared_ptr<VkContext> vkc_;
  explicit SceneRenderDataSharedGpu(std::shared_ptr<VkContext> vkc) : vkc_(vkc) {
    vertex_buffer_.buffer_ = VK_NULL_HANDLE;
  }
  ~SceneRenderDataSharedGpu();
};

struct MeshRenderDataGpu {
  bool is_valid_{false};
  VmaAllocBuffer instance_buffer_;
  VmaAllocBuffer index_buffer_;
  UInt32 instance_count_;
  UInt32 index_count_;
  MeshPushConstants pc_;

  std::shared_ptr<VkContext> vkc_;

  explicit MeshRenderDataGpu(std::shared_ptr<VkContext> vkc) : vkc_(vkc) {
    instance_buffer_.buffer_ = nullptr;
    index_buffer_.buffer_ = nullptr;
  }

  ~MeshRenderDataGpu();
};

template <int d> glm::vec<d, GpuReal> to_glm(Vector<Real, d> vec) {
  glm::vec<d, GpuReal> gl;
  for (int i = 0; i < d; ++i) {
    gl[i] = vec[i];
  }
  return gl;
}
}  // namespace details

struct SimplicalRenderData {
  /****************************************
   * Render data for CPU
   ****************************************/
  Field<details::SceneVertex> vertices_;

  // Assert `face` has element size that is multiple of 3
  Field<uint32_t> faces_;
  Field<details::MeshInstance> instances_;

  // Assert `segments` has element size that is multiple of 2
  Field<uint32_t> segments_;
  Field<uint32_t> points_;

  glm::vec4 color_bias_;

  Mat4x4<Real> model_matrix_{RealMat4x4::Ones()};
  /****************************************
   * Flags
   ****************************************/

  /**
   * @brief Flush the data from CPU to GPU.
   *
   */
  bool flush_{false};

  /**
   * @brief Indicates the buffer flush is lazy, the renderer will only flush
   * data form this structure when `flush_` is true
   *
   */
  bool lazy_load_{true};
};

}  // namespace axes::gui
