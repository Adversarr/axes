#include "ax/fem/mesh.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"

namespace ax::fem {

Mesh::Mesh(size_t n_dof_per_vertex, size_t n_vert_per_element, BufferDevice device)
    : n_dof_per_vertex_(n_dof_per_vertex),
      n_vert_per_element_(n_vert_per_element),
      device_(device) {}

void Mesh::SetData(ConstRealBufferView vertices, ConstSizeBufferView elements) {
  size_t n_vertices = vertices.Shape().Y();
  size_t in_n_dof = vertices.Shape().X();
  size_t n_elements = elements.Shape().Y();
  size_t in_n_vert_per_element = elements.Shape().X();
  AX_THROW_IF(n_dof_per_vertex_ != in_n_dof, "Invalid number of DOF per vertex, got {} expect {}",
              in_n_dof, n_dof_per_vertex_);

  AX_THROW_IF(n_vert_per_element_ != in_n_vert_per_element,
              "Invalid number of vertices per element, got {} expect {}", in_n_vert_per_element,
              n_vert_per_element_);

  vertices_ = create_buffer<Real>(device_, {n_dof_per_vertex_, n_vertices});
  elements_ = create_buffer<size_t>(device_, {n_vert_per_element_, n_elements});

  copy(vertices_->View(), vertices);
  copy(elements_->View(), elements);
}

BufferPtr<Real> Mesh::GetVertices() const { return vertices_; }

BufferPtr<size_t> Mesh::GetElements() const { return elements_; }

size_t Mesh::GetNumVertices() const { return vertices_->Shape().Y(); }

size_t Mesh::GetNumElements() const { return elements_->Shape().Y(); }

size_t Mesh::GetNumDOFPerVertex() const { return n_dof_per_vertex_; }

size_t Mesh::GetNumVerticesPerElement() const { return n_vert_per_element_; }

}  // namespace ax::fem