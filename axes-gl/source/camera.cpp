#include "axes/gl/camera.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

#include "axes/core/echo.hpp"
#include "axes/geometry/transforms.hpp"
#include "axes/math/functional.hpp"
#include "axes/math/linalg.hpp"

namespace ax::gl {

Camera::Camera() : yaw_(0.0f), pitch_(0.0f), fov_(45.0f) {
  SetRotate(-90.0f, 0.0f);
  SetPosition(math::vec3r(0.0f, 0.0f, 3.0f));
  aspect_ = 1.0;
}

void Camera::Rotate(real yaw, real pitch) {
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

void Camera::SetRotate(real yaw, real pitch) {
  yaw_ = yaw;
  pitch_ = pitch;

  DLOG(INFO) << "Yaw=" << yaw_ << " Pitch=" << pitch_;

  // Update front, right and up Vectors using the updated yaw and pitch
  front_.x() = cos(math::radians(yaw_)) * cos(math::radians(pitch_));
  front_.y() = sin(math::radians(pitch_));
  front_.z() = sin(math::radians(yaw_)) * cos(math::radians(pitch_));
  front_ = math::normalized(front_);

  right_ = math::normalized(math::cross(front_, math::vec3r(0.0f, 1.0f, 0.0f)));
  up_ = math::normalized(math::cross(right_, front_));
}

void Camera::Move(math::vec3r const& direction) { position_ += direction; }

void Camera::SetPosition(math::vec3r const& position) { position_ = position; }

math::vec3r const& Camera::GetPosition() const { return position_; }
math::vec3r const& Camera::GetFront() const { return front_; }
math::vec3r const& Camera::GetRight() const { return right_; }
math::vec3r const& Camera::GetUp() const { return up_; }
real Camera::GetYaw() const { return yaw_; }
real Camera::GetPitch() const { return pitch_; }
real Camera::GetFov() const { return fov_; }

void Camera::SetAspect(real aspect) { aspect_ = aspect; }
real Camera::GetAspect() const { return aspect_; }

math::mat4r Camera::LookAt() const {
  auto lookat = geo::look_at(position_, position_ + front_, up_);
  // auto lookat_glm = glm::lookAt(glm::vec3(position_.x(), position_.y(), position_.z()),
  //                               glm::vec3(position_.x() + front_.x(), position_.y() + front_.y(), position_.z() +
  //                               front_.z()), glm::vec3(up_.x(), up_.y(), up_.z()));
  // for (int i = 0; i < 4; ++i) {
  //   for (int j = 0; j < 4; ++j) {
  //     DLOG(INFO) << "lookat[" << i << "][" << j << "]: " << lookat(i, j) << " glm: " << lookat_glm[i][j];
  //   }
  // }
  return lookat;
}

math::mat4r Camera::Perspective() const {
  auto persp = geo::perspective(fov_, aspect_, 0.1f, 100.0f);
  // auto persp_glm = glm::perspective<f32>(fov_, aspect_, 0.1f, 100.0f);
  // for (int i = 0; i < 4; ++i) {
  //   for (int j = 0; j < 4; ++j) {
  //     DLOG(INFO) << "persp[" << i << "][" << j << "]: " << persp(i, j) << " glm: " << persp_glm[i][j];
  //   }
  // }
  return persp;
}

}  // namespace ax::gl
