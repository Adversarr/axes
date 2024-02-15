#pragma once
#include "axes/math/common.hpp"
namespace ax::gl {

class Camera {
public:
  Camera();

  ~Camera() = default;

  /****************************** Methods ******************************/
  void Rotate(f32 yaw, f32 pitch);

  void SetRotate(f32 yaw, f32 pitch);

  void Move(math::vec3f const& direction);

  void SetPosition(math::vec3f const& position);

  void SetAspect(f32 aspect);

  void SetAspect(idx x, idx y);

  math::mat4f LookAt() const;
  math::mat4f Perspective() const;
  math::mat4f Ortho() const;
  math::mat4f GetProjectionMatrix() const;

  void SetFov(f32 fov);
  void SetOrthoHorizontal(math::vec2f const& value);
  void SetOrthoVertical(math::vec2f const& value);
  void SetProjectionMode(bool use_perspective);

  /****************************** Getters ******************************/
  math::vec3f& GetPosition();

  math::vec3f& GetFront();

  math::vec3f& GetRight();

  math::vec3f& GetUp();

  f32 GetYaw() const;

  f32 GetPitch() const;

  f32 GetFov() const;

  f32 GetAspect() const;

  bool use_perspective_;
private:
  // External Parameters
  math::vec3f position_;

  // These will sync with rotation
  math::vec3f up_;
  math::vec3f right_;
  math::vec3f front_;
  f32 yaw_;
  f32 pitch_;

  math::vec2f ortho_horizontal_;
  math::vec2f ortho_vertical_;

  // Internal Parameters
  f32 fov_;
  f32 aspect_;
};

}  // namespace ax::gl
