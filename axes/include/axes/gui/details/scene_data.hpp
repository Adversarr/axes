#pragma once
#include <absl/container/flat_hash_map.h>
#include "axes/core/math/common.hpp"

namespace axes::gui {

using UiWindowCallback = std::function<void(void)>;

struct UiWindows {
  absl::flat_hash_map<std::string, UiWindowCallback> callbacks_;

  std::function<void(void)> menu_bar_;

  static void InitResource();
};

struct SceneCamera {
  RealVector3 position_;
  RealVector3 up_;
  RealVector3 front_;
  Real aspect_;
  bool update_{true};
  static void InitResource();
};

struct SceneLight {
  RealVector3 point_light_pos_;
  RealVector4 point_light_color_;
  RealVector3 parallel_light_dir_;
  RealVector4 parallel_light_color_;
  RealVector4 ambient_light_color_;
  bool update_{true};
  static void InitResource();
};

enum class ProjectionMode : int { kPerspective, kOrtho };

struct SceneProjection {
  RealMat4x4 projection_;
  ProjectionMode mode_;
  bool update_{true};
  static void InitResource();
};

struct SceneOrthoProjection {
  Real left_, right_, bottom_, top_, near_, far_;
  bool update_{true};
  static void InitResource();
};

struct ScenePerspectiveProjection {
  Real wh_ratio_, near_, far_, fovy_;
  bool update_{true};
  static void InitResource();
};

RealMat4x4 compute_perspective(ScenePerspectiveProjection proj);

}  // namespace axes::gui
