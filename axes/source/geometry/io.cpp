//
// Created by Yang Jerry on 2024/3/3.
//
#include "axes/geometry/io.hpp"
#include "axes/utils/status.hpp"

#include <igl/readOBJ.h>


namespace ax::geo {

StatusOr<SurfaceMesh> read_obj(std::string const& path) {
  math::matx3r vertices;
  math::matx3i indices;
  if (!igl::readOBJ(path, vertices, indices)) {
    return utils::NotFoundError("Failed to read obj file");
  }
  return SurfaceMesh(vertices.transpose(), indices.transpose());
}

}  // namespace ax::geo