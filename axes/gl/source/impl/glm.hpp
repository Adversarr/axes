#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "ax/math/common.hpp"
namespace ax::gl::details {

template <typename Scalar, int rows>
glm::vec<rows, float> to_glm(math::Vector<Scalar, rows> const& m) {
  glm::vec<rows, float> result;
  for (int i = 0; i < rows; ++i) {
    result[i] = static_cast<float>(m[i]);
  }
  return result;
}

inline glm::mat4 lookat(math::RealVector3 const& eye, math::RealVector3 const& center, math::RealVector3 const& up) {
  return glm::lookAt(to_glm(eye), to_glm(center), to_glm(up));
}

inline glm::mat4 perspective(real fov, real aspect, real near, real far) {
  return glm::perspective(fov, aspect, near, far);
}

inline glm::mat4 ortho(real left, real right, real bottom, real top, real near, real far) {
  return glm::ortho(left, right, bottom, top, near, far);
}

}  // namespace ax::gl::details
