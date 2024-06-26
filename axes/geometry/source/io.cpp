//
// Created by Yang Jerry on 2024/3/3.
//
#include "ax/geometry/io.hpp"
#include "ax/core/excepts.hpp"
#include "ax/utils/iota.hpp"
#include "ax/utils/status.hpp"

#include <igl/readOBJ.h>
#include <igl/readNODE.h>
#include <igl/readMESH.h>


namespace ax::geo {

SurfaceMesh read_obj(std::string const& path) {
  math::matx3r vertices;
  math::matx3i indices;
  AX_THROW_IF_FALSE(igl::readOBJ(path, vertices, indices), "Failed to read obj file: " + path);
  return SurfaceMesh(vertices.transpose(), indices.transpose());
}

EleFileReadResult read_ele(std::string const& ele_file){
  // Ignore all the attributes.
  // 1st line: <# of tetrahedra> <nodes per tetrahedron> <# of attributes>
  // <tetrahedron #> <node> <node> <node> <node> ... [attributes]
  // ...
  std::fstream file(ele_file);
  // if (! file) {
  //   return utils::NotFoundError("File not found.");
  // }
  AX_THROW_IF_FALSE(file, "File not found: " + ele_file);
  math::matxxi ele;
  idx n_tet, node_per_tet, n_attr;
  file >> n_tet >> node_per_tet >> n_attr;
  // if (node_per_tet != 4) {
  //   return utils::InvalidArgumentError("read ele only support 3d mesh with node_per_tet=4.");
  // }
  AX_THROW_IF_NE(node_per_tet, 4, "read ele only support 3d mesh with node_per_tet=4.");
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

NodeFileReadResult read_node(std::string const& path) {
  math::matxxr V;
  math::matxxi I;
  AX_THROW_IF_FALSE(igl::readNODE(path, V, I), "Failed to read node file: " + path);
  return NodeFileReadResult{V.transpose(), I.transpose()};
}

}  // namespace ax::geo