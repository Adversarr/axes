//
// Created by Yang Jerry on 2024/3/3.
//
#include "axes/geometry/io.hpp"
#include "axes/utils/iota.hpp"
#include "axes/utils/status.hpp"

#include <igl/readOBJ.h>
#include <igl/readNODE.h>
#include <igl/readMESH.h>


namespace ax::geo {

StatusOr<SurfaceMesh> read_obj(std::string const& path) {
  math::matx3r vertices;
  math::matx3i indices;
  if (!igl::readOBJ(path, vertices, indices)) {
    return utils::NotFoundError("Failed to read obj file");
  }
  return SurfaceMesh(vertices.transpose(), indices.transpose());
}

StatusOr<EleFileReadResult> read_ele(std::string const& ele_file){
  // Ignore all the attributes.
  // 1st line: <# of tetrahedra> <nodes per tetrahedron> <# of attributes>
  // <tetrahedron #> <node> <node> <node> <node> ... [attributes]
  // ...
  std::fstream file(ele_file);
  if (! file) {
    return utils::NotFoundError("File not found.");
  }
  math::matxxi ele;
  idx n_tet, node_per_tet, n_attr;
  file >> n_tet >> node_per_tet >> n_attr;
  if (node_per_tet != 4) {
    return utils::InvalidArgumentError("read ele only support 3d mesh with node_per_tet=4.");
  }
  ele.resize(node_per_tet, n_tet);
  for (idx i = 0; i < n_tet; ++i) {
    idx i_tet;
    for (idx j: utils::iota(node_per_tet)) {
      file >> i_tet;
      ele(j, i) = i_tet - 1;
    }
  }
  return EleFileReadResult{ele};
}

StatusOr<NodeFileReadResult> read_node(std::string const& path) {
  math::matxxr V;
  math::matxxi I;
  if (! igl::readNODE(path, V, I)) {
    return utils::NotFoundError("File invalid.");
  }
  return NodeFileReadResult{V.transpose(), I.transpose()};
}

}  // namespace ax::geo