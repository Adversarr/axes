#pragma once
#include "common.hpp"

namespace ax::geo {

/**
 * @brief Reads an OBJ file and returns a SurfaceMesh object.
 *
 * This function reads an OBJ file from the specified path and converts it into a SurfaceMesh
 * object.
 *
 * @param path The path to the OBJ file.
 * @return A StatusOr object containing the SurfaceMesh if the file was successfully read, or an
 * error message if there was a problem.
 */
SurfaceMesh read_obj(std::string const& path);

struct EleFileReadResult {
  math::IndexMatrixX elements_;
  EleFileReadResult(math::IndexMatrixX elements) : elements_(std::move(elements)) {}
  AX_DECLARE_CONSTRUCTOR(EleFileReadResult, default, default);
};
EleFileReadResult read_ele(std::string const& ele_file);

struct NodeFileReadResult {
  math::RealMatrixX vertices_;
  math::IndexMatrixX boundary_markers_;

  NodeFileReadResult(math::RealMatrixX vertices, math::IndexMatrixX boundary_markers)
      : vertices_(std::move(vertices)), boundary_markers_(std::move(boundary_markers)) {}

  AX_DECLARE_CONSTRUCTOR(NodeFileReadResult, default, default);
};

NodeFileReadResult read_node(std::string const& ele_file);



}  // namespace ax::geo