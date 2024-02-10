#pragma once
#include "axes/math/functional.hpp"

namespace ax::gl {

using cmap = float const[256][3];

// This is taken from DiffFR.
extern float const colormap_bwr[256][3];
extern float const colormap_coolwarm[256][3];
extern float const colormap_jet[256][3];
extern float const colormap_plasma[256][3];
extern float const colormap_seismic[256][3];

class Colormap {
public:
  Colormap(real low, real high, bool periodic = false, cmap& colormap = colormap_jet)
      : low_(low), high_(high), periodic_(periodic), colormap_(colormap) {}

  math::vec3r operator()(real value) const {
    if (periodic_) {
      value = math::fmod(value - low_, high_ - low_) + low_;
    } else {
      value = math::clamp(value, low_, high_);
    }
    value = (value - low_) / (high_ - low_);
    return math::vec3f{colormap_[static_cast<int>(value * 255)]}.cast<real>();
  }

  math::field3r operator()(math::vecxr const& values) const {
    math::field3r colors(3, values.size());
    for (idx i = 0; i < values.size(); ++i) {
      colors.col(i) = operator()(values[i]);
    }
    return colors;
  }

private:
  real low_;
  real high_;
  bool periodic_;
  cmap& colormap_;
};

}  // namespace ax::gl