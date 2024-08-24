#pragma once
#include "ax/math/common.hpp"
namespace ax::gl {

class Camera {
public:
  Camera();

  ~Camera() = default;

  /****************************** Methods ******************************/
  void Rotate(f32 yaw, f32 pitch);

  void SetRotate(f32 yaw, f32 pitch);

  void Move(math::FloatVector3 const& direction);

  void SetPosition(math::FloatVector3 const& position);

  void SetAspect(f32 aspect);

  void SetAspect(Index x, Index y);

  void ApplyTransform(math::FloatMatrix4 const& transform);

  math::FloatMatrix4 LookAt() const;
  math::FloatMatrix4 Perspective() const;
  math::FloatMatrix4 Ortho() const;
  math::FloatMatrix4 GetProjectionMatrix() const;

  void SetFov(f32 fov);
  void SetOrthoHorizontal(math::FloatVector2 const& value);
  void SetOrthoVertical(math::FloatVector2 const& value);
  void SetProjectionMode(bool use_perspective);

  /****************************** Getters ******************************/
  math::FloatVector3& GetPosition();

  math::FloatVector3& GetFront();

  math::FloatVector3& GetRight();

  math::FloatVector3& GetUp();

  f32 GetYaw() const;

  f32 GetPitch() const;

  f32 GetFov() const;

  f32 GetAspect() const;

  bool use_perspective_;
private:
  // External Parameters
  math::FloatVector3 position_;

  // These will sync with rotation
  math::FloatVector3 up_;
  math::FloatVector3 right_;
  math::FloatVector3 front_;
  f32 yaw_;
  f32 pitch_;

  math::FloatVector2 ortho_horizontal_;
  math::FloatVector2 ortho_vertical_;

  // Internal Parameters
  f32 fov_;
  f32 aspect_;
};

}  // namespace ax::gl
