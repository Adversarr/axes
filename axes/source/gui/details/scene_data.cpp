#include "axes/gui/details/scene_data.hpp"

#include "axes/core/ecs/resource_manager.hpp"
#include "axes/core/utils/log.hpp"

// BUG on Windows.
#ifndef M_PI
#  define M_PI (3.141592653589793)
#endif

namespace axes::gui {

void SceneCamera::InitResource() {
  auto rc = ecs::RMan::Construct<SceneCamera>();
  rc->position_ = RealVector3{3, 0, 1};
  rc->up_ = RealVector3{0, 0, 3};
  rc->front_ = -rc->position_.normalized();
  rc->aspect_ = 1366.0 / 768.0;
}

void SceneLight::InitResource() {
  auto rc = ecs::RMan::Construct<SceneLight>();
  rc->ambient_light_color_ = RealVector4{.2, .2, .2, .05};
  rc->parallel_light_color_.setZero();
  rc->point_light_color_ = RealVector4{1, 1, 1, 0.9};
  rc->point_light_pos_.setConstant(3);
  rc->parallel_light_dir_.setConstant(-1);
}

void SceneProjection::InitResource() {
  auto rc = ecs::RMan::Construct<SceneProjection>();
  rc->mode_ = ProjectionMode::kPerspective;
  auto pp = ecs::Rc<ScenePerspectiveProjection>{}.MakeValid();
  rc->projection_ = compute_perspective(*pp);

  pp.Subscribe([]() {
    auto rc = ecs::RMan::Get<SceneProjection>();
    auto sp = ecs::RMan::Get<ScenePerspectiveProjection>();
    rc->projection_ = compute_perspective(*sp);
    rc->update_ = true;
  });
}

void ScenePerspectiveProjection::InitResource() {
  auto rc = ecs::RMan::Construct<ScenePerspectiveProjection>();
  rc->far_ = 100;
  rc->near_ = 0.1;
  // TODO: M_PI is invalid under Windows.
  rc->fovy_ = M_PI * 0.25;
  rc->wh_ratio_ = 1366.0 / 768.0;
  auto cam = ecs::Rc<SceneCamera>{}.MakeValid();
  cam.Subscribe([]() {
    auto sp = ecs::RMan::Get<ScenePerspectiveProjection>();
    auto ca = ecs::RMan::Get<SceneCamera>();
    sp->wh_ratio_ = ca->aspect_;
    sp.Publish();
  });
}

RealMat4x4 compute_perspective(ScenePerspectiveProjection proj) {
  RealMat4x4 m = RealMat4x4::Identity();
  Real tan_fovy = tan(0.5 * proj.fovy_);
  m(1, 1) = static_cast<Real>(1) / tan_fovy;
  m(0, 0) = m(1, 1) / proj.wh_ratio_;
  m(2, 2) = -proj.far_ / (proj.far_ - proj.near_);
  m(2, 3) = -1;
  m(3, 2) = -(proj.far_ * proj.near_) / (proj.far_ - proj.near_);
  m(1, 1) = -m(1, 1);
  m(3, 3) = 0;
  return m;
}

void UiWindows::InitResource() { ecs::RMan::Construct<UiWindows>(); }

}  // namespace axes::gui
