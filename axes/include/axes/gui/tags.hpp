#pragma once
#include "axes/core/math/field.hpp"

namespace axes::gui {

/**
 * @class SimplicalRenderConfig
 * @brief If entity has this component, `RenderSystem` will check this config
 * when `TickLogic`. If `load_from_geometry`, the entity's `SimplicalRenderData`
 * will be set according to the config, and update the buffers.
 *
 */
struct SimplicalRenderConfig {
  /**
   * @brief If true, a component `SimplicalRenderObject` is created/modified
   *  automatically according to Geometry. This var also force the renderer
   *  to fetch data from geometry every frame.
   */
  bool load_from_geometry_{false};

  /**
   * @brief use perface normal instead of averaging.
   *
   */
  bool per_face_normal_{true};
};

struct Face {
  uint32_t indices_[3];
};

struct WireframeCpu {
  uint32_t indices_[2];
};

struct InstanceInfo {
  Vector3<Float32> position_;
  Vector4<Float32> rotation_;
};

/**
 * @class SimplicalRenderData
 * @brief If entity has this component, the `RenderSystem` will render it to the
 * screen according to the data. Note that, this component does not have to be
 * created by the `SimplicalRenderConfig`.
 *
 */
struct SimplicalRenderData;

} // namespace axes::gui
