#pragma once
#include "ax/core/buffer/buffer.hpp"
#include "ax/core/buffer/buffer_view.hpp"

namespace ax::fem {

/**
 * @brief Defines the Finite Element Mesh, but not recording its
 *
 */
class Mesh {
  Mesh() = default;
public:
  Mesh(const Mesh&) = default;
  Mesh(Mesh&&) = default;
  Mesh& operator=(const Mesh&) = default;
  Mesh& operator=(Mesh&&) = default;

  Mesh(size_t n_dof_per_vertex, size_t n_vert_per_element,
         BufferDevice device = BufferDevice::Host);

  void SetData(ConstRealBufferView vertices, ConstSizeBufferView elements);

  BufferPtr<Real> GetVertices() const;
  BufferPtr<size_t> GetElements() const;

  size_t GetNumVertices() const;
  size_t GetNumElements() const;
  size_t GetNumDOFPerVertex() const;
  size_t GetNumVerticesPerElement() const;

private:
  BufferPtr<Real> vertices_;       ///< coordinates of the vertices
  BufferPtr<size_t> elements_;     ///< describes the topology of the mesh
  size_t n_dof_per_vertex_ = 0;    ///< 2 for 2D, 3 for 3D
  size_t n_vert_per_element_ = 0;  ///< 3 for triangle, 4 for tetrahedron(3D)/quad(2D)
  BufferDevice device_;            ///< Device to store the mesh data.
};

}  // namespace ax::fem