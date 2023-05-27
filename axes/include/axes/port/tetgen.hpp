#pragma once

#include <filesystem>

#include "axes/core/math/math.hpp"

namespace axes::port {

// TODO: Implementation

class TetgenLoader {
public:
  void Clear();

  void LoadTetra(std::istream& input);

  void LoadNodes(std::istream& input);

private:
  Field<IndexVector4> tetras_;
  Field<IndexVector3> faces_;
  Field<IndexVector2> edges_;
  Field<RealVector3> node_positions_;
};

}  // namespace axes::port
