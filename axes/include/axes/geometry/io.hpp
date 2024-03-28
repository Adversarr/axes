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
StatusOr<SurfaceMesh> read_obj(std::string const& path);

struct EleFileReadResult {
  math::matxxi elements_;
  EleFileReadResult(math::matxxi elements) : elements_(std::move(elements)) {}
  AX_DECLARE_CONSTRUCTOR(EleFileReadResult, default, default);
};
StatusOr<EleFileReadResult> read_ele(std::string const& ele_file);

struct NodeFileReadResult {
  math::matxxr vertices_;
  math::matxxi boundary_markers_;

  NodeFileReadResult(math::matxxr vertices, math::matxxi boundary_markers)
      : vertices_(std::move(vertices)), boundary_markers_(std::move(boundary_markers)) {}

  AX_DECLARE_CONSTRUCTOR(NodeFileReadResult, default, default);
};

StatusOr<NodeFileReadResult> read_node(std::string const& ele_file);



}  // namespace ax::geo