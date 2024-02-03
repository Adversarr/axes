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

  math::mat4r LookAt() const;
  math::mat4r Perspective() const;

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

  // Internal Parameters
  real fov_;
  real aspect_;
};

}  // namespace ax::gl
