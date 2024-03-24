#include "axes/pde/fem/p1mesh.hpp"

#ifdef AX_PLATFORM_WINDOWS
#undef ERROR
#endif
namespace ax::pde::fem {

template <idx dim> MeshBase<dim>::MeshBase(MeshType type) : type_(type) {}

template <idx dim>
utils::uptr<MeshBase<dim>> MeshBase<dim>::Create(MeshType type) {
  if (type == MeshType::kP1) {
    return std::make_unique<P1Mesh<dim>>();
  } else {
    AX_LOG(ERROR) << "Unknown mesh type.";
    return nullptr;
  }
  AX_UNREACHABLE();
}

template <idx dim> MeshType MeshBase<dim>::GetType() const noexcept { return type_; }

template <idx dim>
Status MeshBase<dim>::SetMesh(element_list_t const& elements, vertex_list_t const& vertices) {
  if (elements.rows() != GetNumVerticesPerElement()) {
    return utils::InvalidArgumentError("The number of elements does not match.");
  }
  elements_ = elements;
  vertices_ = vertices;
  ResetAllBoundaries();
  AX_RETURN_OK();
}

template <idx dim> Status MeshBase<dim>::SetVertices(vertex_list_t const& vertices) {
  if (vertices.size() != vertices_.size()) {
    return utils::InvalidArgumentError("The number of vertices does not match.");
  }
  vertices_ = vertices;
  AX_RETURN_OK();
}

template <idx dim> void MeshBase<dim>::SetVertex(idx i, vertex_t const& vertex) {
  AX_DCHECK(0 <= i && i < vertices_.size()) << "Index out of range.";
  vertices_.col(i) = vertex;
}

template <idx dim> auto MeshBase<dim>::GetVertices() const noexcept
    -> MeshBase<dim>::vertex_list_t const& {
  return vertices_;
}

template <idx dim> 
typename MeshBase<dim>::element_list_t const& MeshBase<dim>::GetElements() const noexcept {
  return elements_;
}

template <idx dim> math::vecxr MeshBase<dim>::GetVerticesFlattened() const noexcept {
  math::vecxr vertices_flattened(vertices_.size());
  for (idx i = 0; i < vertices_.cols(); ++i) {
    for (idx j = 0; j < dim; ++j) {
      vertices_flattened[i * dim + j] = vertices_(j, i);
    }
  }
  return vertices_flattened;
}

template <idx dim> void MeshBase<dim>::ResetBoundary(idx i) {
  AX_DCHECK(0 <= i && i < (idx) boundary_types_.size()) << "Index out of range.";
  boundary_types_[i] = BoundaryType::kNone;
  dirichlet_boundary_mask_(0, i) = 1;
}

template <idx dim> void MeshBase<dim>::MarkDirichletBoundary(idx i, const boundary_value_t& value) {
  AX_DCHECK(0 <= i && i < (idx) boundary_types_.size()) << "Index out of range.";
  boundary_types_[i] = BoundaryType::kDirichlet;
  boundary_values_.col(i) = value;
  dirichlet_boundary_mask_(0, i) = 0;
}

template <idx dim> void MeshBase<dim>::MarkNeumannBoundary(idx i, const boundary_value_t& value) {
  AX_DCHECK(0 <= i && i < (idx) boundary_types_.size()) << "Index out of range.";
  boundary_types_[i] = BoundaryType::kNeumann;
  boundary_values_.col(i) = value;
  dirichlet_boundary_mask_(0, i) = 1;
}

template <idx dim> void MeshBase<dim>::ResetAllBoundaries() {
  boundary_types_.resize((std::size_t) vertices_.cols());
  boundary_values_.resize(dim, vertices_.cols());
  for (idx i = 0; i < vertices_.cols(); ++i) {
    boundary_types_[i] = BoundaryType::kNone;
  }
  dirichlet_boundary_mask_.setOnes(1, vertices_.cols());
}

template <idx dim>
typename MeshBase<dim>::boundary_value_t MeshBase<dim>::GetBoundaryValue(idx i) const noexcept {
  AX_DCHECK(0 <= i && i < boundary_values_.size()) << "Index out of range.";
  return boundary_values_.col(i);
}

template <idx dim> bool MeshBase<dim>::IsDirichletBoundary(idx i) const noexcept {
  AX_DCHECK(0 <= i && i < (idx) boundary_types_.size()) << "Index out of range.";
  return boundary_types_[i] == BoundaryType::kDirichlet;
}

template <idx dim> bool MeshBase<dim>::IsNeumannBoundary(idx i) const noexcept {
  AX_DCHECK(0 <= i && i < (idx) boundary_types_.size()) << "Index out of range.";
  return boundary_types_[i] == BoundaryType::kNeumann;
}


template class MeshBase<2>;
template class MeshBase<3>;

}  // namespace ax::pde::fem