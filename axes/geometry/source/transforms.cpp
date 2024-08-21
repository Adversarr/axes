#include "ax/geometry/transforms.hpp"

#include "ax/math/linalg.hpp"

namespace ax::geo {

math::RealMatrix4 ortho(Real left, Real right, Real bottom, Real top, Real near, Real far) {
  using namespace math;
  RealMatrix4 result = math::eye<4>();
  result(0, 0) = 2.0f / (right - left);
  result(1, 1) = 2.0f / (top - bottom);
  result(2, 2) = -2.0f / (far - near);
  result(3, 0) = -(right + left) / (right - left);
  result(3, 1) = -(top + bottom) / (top - bottom);
  result(3, 2) = -(far + near) / (far - near);
  return result;
}

math::RealMatrix4 perspective(Real fovy, Real aspect, Real near, Real far) {
  // OpenGL:
  Real tan_half_fovy = math::tan(fovy / 2.0f);
  math::RealMatrix4 result = math::zeros<4, 4>();
  result(0, 0) = 1.0f / (aspect * tan_half_fovy);
  result(1, 1) = 1.0f / (tan_half_fovy);
  result(2, 2) = -(far + near) / (far - near);
  result(2, 3) = -1.0f;
  result(3, 2) = -(2.0f * far * near) / (far - near);
  return result;
}
math::RealMatrix4 look_at(math::RealVector3 const& eye, math::RealVector3 const& center, math::RealVector3 const& up) {
  // OpenGL
  using namespace math;
  RealVector3 f = normalized(center - eye);
  RealVector3 s = normalized(cross(f, up));
  RealVector3 u = cross(s, f);

  RealMatrix4 result = zeros<4, 4>();
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

math::RealMatrix4 rotate_x(Real angle_rad) {
  math::RealMatrix4 result = math::eye<4>();
  result(1, 1) = cos(angle_rad);
  result(1, 2) = -sin(angle_rad);
  result(2, 1) = sin(angle_rad);
  result(2, 2) = cos(angle_rad);
  return result;
}

math::RealMatrix4 rotate_y(Real angle_rad) {
  math::RealMatrix4 result = math::eye<4>();
  result(0, 0) = cos(angle_rad);
  result(0, 2) = sin(angle_rad);
  result(2, 0) = -sin(angle_rad);
  result(2, 2) = cos(angle_rad);
  return result;
}

math::RealMatrix4 rotate_z(Real angle_rad) {
  math::RealMatrix4 result = math::eye<4>();
  result(0, 0) = cos(angle_rad);
  result(0, 1) = -sin(angle_rad);
  result(1, 0) = sin(angle_rad);
  result(1, 1) = cos(angle_rad);
  return result;
}

math::RealMatrix4 translate(math::RealVector3 const& v) {
  math::RealMatrix4 result = math::eye<4>();
  result.col(3) = v.homogeneous();
  return result;
}

}  // namespace ax::geo
