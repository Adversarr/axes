#include "axes/geometry/transforms.hpp"

#include "axes/math/linalg.hpp"

namespace ax::geo {

math::mat4r ortho(real left, real right, real bottom, real top, real near, real far) {
  using namespace math;
  mat4r result = math::eye<4>();
  result(0, 0) = 2.0f / (right - left);
  result(1, 1) = 2.0f / (top - bottom);
  result(2, 2) = -2.0f / (far - near);
  result(3, 0) = -(right + left) / (right - left);
  result(3, 1) = -(top + bottom) / (top - bottom);
  result(3, 2) = -(far + near) / (far - near);
  return result;
}

math::mat4r perspective(real fovy, real aspect, real near, real far) {
  // OpenGL:
  real tan_half_fovy = math::tan(fovy / 2.0f);
  math::mat4r result = math::zeros<4, 4>();
  result(0, 0) = 1.0f / (aspect * tan_half_fovy);
  result(1, 1) = 1.0f / (tan_half_fovy);
  result(2, 2) = -(far + near) / (far - near);
  result(2, 3) = -1.0f;
  result(3, 2) = -(2.0f * far * near) / (far - near);
  return result;
}
math::mat4r look_at(math::vec3r const& eye, math::vec3r const& center, math::vec3r const& up) {
  // OpenGL
  using namespace math;
  vec3r f = normalized(center - eye);
  vec3r s = normalized(cross(f, up));
  vec3r u = cross(s, f);

  mat4r result = zeros<4, 4>();
  result(0, 0) = s.x();
  result(1, 0) = s.y();
  result(2, 0) = s.z();
  result(0, 1) = u.x();
  result(1, 1) = u.y();
  result(2, 1) = u.z();
  result(0, 2) = -f.x();
  result(1, 2) = -f.y();
  result(2, 2) = -f.z();
  result(3, 0) = -dot(s, eye);
  result(3, 1) = -dot(u, eye);
  result(3, 2) = dot(f, eye);
  result(3, 3) = 1.0f;
  return result;
}

math::mat4r rotate_x(real angle_rad) {
  math::mat4r result = math::eye<4>();
  result(1, 1) = cos(angle_rad);
  result(1, 2) = -sin(angle_rad);
  result(2, 1) = sin(angle_rad);
  result(2, 2) = cos(angle_rad);
  return result;
}

math::mat4r rotate_y(real angle_rad) {
  math::mat4r result = math::eye<4>();
  result(0, 0) = cos(angle_rad);
  result(0, 2) = sin(angle_rad);
  result(2, 0) = -sin(angle_rad);
  result(2, 2) = cos(angle_rad);
  return result;
}

math::mat4r rotate_z(real angle_rad) {
  math::mat4r result = math::eye<4>();
  result(0, 0) = cos(angle_rad);
  result(0, 1) = -sin(angle_rad);
  result(1, 0) = sin(angle_rad);
  result(1, 1) = cos(angle_rad);
  return result;
}

math::mat4r translate(math::vec3r const& v) {
  math::mat4r result = math::eye<4>();
  result.col(3) = v.homogeneous();
  return result;
}

}  // namespace ax::geo
