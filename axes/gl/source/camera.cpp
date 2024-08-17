#include "ax/gl/camera.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

#include "ax/geometry/transforms.hpp"
#include "ax/math/functional.hpp"
#include "ax/math/linalg.hpp"

using namespace std;

namespace ax::gl {

Camera::Camera() : use_perspective_(true), yaw_(0.0f), pitch_(0.0f), fov_(45.0f) {
  SetRotate(-135.0f, -10.0f);
  SetPosition(math::FloatVector3(3.0f, 1.0f, 3.0f));
  aspect_ = 1.0;

  ortho_vertical_ = math::FloatVector2(-1.0f, 1.0f);
  ortho_horizontal_ = math::FloatVector2(-1.0f, 1.0f);
}

math::FloatMatrix4 Camera::Ortho() const {
  real left = ortho_horizontal_.x();
  real right = ortho_horizontal_.y();
  real bottom = ortho_vertical_.x();
  real top = ortho_vertical_.y();
  if (aspect_ < 1.0f) {
    real new_height = (top - bottom) / aspect_;
    bottom = (top + bottom - new_height) / 2.0f;
    top = (top + bottom + new_height) / 2.0f;
  } else {
    real new_width = (right - left) * aspect_;
    left = (left + right - new_width) / 2.0f;
    right = (left + right + new_width) / 2.0f;
  }

  auto ortho = geo::ortho(left, right, bottom, top, 0.1f, 100.0f);
  return ortho.cast<f32>();
}

math::FloatMatrix4 Camera::GetProjectionMatrix() const {
  if (use_perspective_) {
    return Perspective();
  } else {
    return Ortho();
  }
}

void Camera::Rotate(f32 yaw, f32 pitch) {
  yaw_ += yaw;
  pitch_ += pitch;

  if (pitch_ > 89.0f) {
    pitch_ = 89.0f;
  }
  if (pitch_ < -89.0f) {
    pitch_ = -89.0f;
  }

  if (yaw_ > 360.0f) {
    yaw_ -= 360.0f;
  }
  if (yaw_ < -360.f) {
    yaw_ += 360.f;
  }

  SetRotate(yaw_, pitch_);
}

void Camera::SetRotate(f32 yaw, f32 pitch) {
  yaw_ = yaw;
  pitch_ = pitch;

  // Update front, right and up Vectors using the updated yaw and pitch
  front_.x() = cos(math::radians(yaw_)) * cos(math::radians(pitch_));
  front_.y() = sin(math::radians(pitch_));
  front_.z() = sin(math::radians(yaw_)) * cos(math::radians(pitch_));
  front_ = math::normalized(front_);

  right_ = math::normalized(math::cross(front_, math::FloatVector3(0.0f, 1.0f, 0.0f)));
  up_ = math::normalized(math::cross(right_, front_));
}

void Camera::Move(math::FloatVector3 const& direction) { position_ += direction; }

void Camera::SetPosition(math::FloatVector3 const& position) { position_ = position; }

math::FloatVector3& Camera::GetPosition() { return position_; }
math::FloatVector3& Camera::GetFront() { return front_; }
math::FloatVector3& Camera::GetRight() { return right_; }
math::FloatVector3& Camera::GetUp() { return up_; }
f32 Camera::GetYaw() const { return yaw_; }
f32 Camera::GetPitch() const { return pitch_; }
f32 Camera::GetFov() const { return fov_; }

void Camera::SetAspect(f32 aspect) { aspect_ = aspect; }

void Camera::SetAspect(Index x, Index y) {
  aspect_ = static_cast<f32>(x) / static_cast<f32>(y);
  SetAspect(aspect_);
}
f32 Camera::GetAspect() const { return aspect_; }

math::FloatMatrix4 Camera::LookAt() const {
  auto lookat
      = geo::look_at(position_.cast<real>(), (position_ + front_).cast<real>(), up_.cast<real>());
  return lookat.cast<f32>();
}

math::FloatMatrix4 Camera::Perspective() const {
  auto persp = geo::perspective(fov_, aspect_, 0.1f, 100.0f);
  return persp.cast<f32>();
}

void Camera::SetOrthoVertical(math::FloatVector2 const& value) { ortho_vertical_ = value; }
void Camera::SetOrthoHorizontal(math::FloatVector2 const& value) { ortho_horizontal_ = value; }
void Camera::SetFov(f32 fov) { fov_ = fov; }
void Camera::SetProjectionMode(bool use_perspective) { use_perspective_ = use_perspective; }
}  // namespace ax::gl
