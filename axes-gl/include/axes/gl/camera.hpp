#pragma once
#include "axes/math/common.hpp"
namespace ax::gl {

class Camera {
public:
  Camera();

  ~Camera() = default;

  /****************************** Methods ******************************/
  void Rotate(real yaw, real pitch);

  void SetRotate(real yaw, real pitch);

  void Move(math::vec3r const& direction);

  void SetPosition(math::vec3r const& position);

  void SetAspect(real aspect);

  void SetAspect(idx x, idx y);

  math::mat4r LookAt() const;
  math::mat4r Perspective() const;
  math::mat4r Ortho() const;
  math::mat4r GetProjectionMatrix() const;

  void SetFov(real fov);
  void SetOrthoHorizontal(math::vec2r const& value);
  void SetOrthoVertical(math::vec2r const& value);
  void SetProjectionMode(bool use_perspective);

  /****************************** Getters ******************************/
  math::vec3r const& GetPosition() const;

  math::vec3r const& GetFront() const;

  math::vec3r const& GetRight() const;

  math::vec3r const& GetUp() const;

  real GetYaw() const;

  real GetPitch() const;

  real GetFov() const;

  real GetAspect() const;

private:
  // External Parameters
  math::vec3r position_;

  // These will sync with rotation
  math::vec3r up_;
  math::vec3r right_;
  math::vec3r front_;
  real yaw_;
  real pitch_;

  bool use_perspective_;
  math::vec2r ortho_horizontal_;
  math::vec2r ortho_vertical_;

  // Internal Parameters
  real fov_;
  real aspect_;
};

}  // namespace ax::gl
