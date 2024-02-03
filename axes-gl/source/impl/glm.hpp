#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "axes/math/common.hpp"
namespace ax::gl::details {

inline glm::mat4 lookat(math::vec3r const& eye, math::vec3r const& center, math::vec3r const& up) {
  return glm::lookAt(glm::vec3(eye.x(), eye.y(), eye.z()),
                     glm::vec3(center.x(), center.y(), center.z()),
                     glm::vec3(up.x(), up.y(), up.z()));
}

inline glm::mat4 perspective(real fov, real aspect, real near, real far) {
  return glm::perspective(fov, aspect, near, far);
}

inline glm::mat4 ortho(real left, real right, real bottom, real top, real near, real far) {
  return glm::ortho(left, right, bottom, top, near, far);
}

}  // namespace ax::gl::details
